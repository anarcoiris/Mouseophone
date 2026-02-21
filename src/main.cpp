/*
 * main.cpp — Mouseofono v1.3 (3-Thread Real-Time Pipeline)
 *
 * Architecture:
 *
 *   [Thread 1 – RawInput]                     Windows message loop (HWND)
 *       | ONLY: QPC timestamp, push to SPSC
 *       | update_poll() called HERE (at-source timestamps)
 *       v
 *   SPSC<MouseEvent>  (g_raw_queue)
 *       |
 *   [Thread 2 – DSP]
 *       | Drains g_raw_queue
 *       | Runs sinc reconstruction OR raw accumulation
 *       | Applies Wiener filter
 *       | Pushes PcmBlock to g_pcm_queue
 *       v
 *   SPSC<PcmBlock>  (g_pcm_queue, 512-float fixed blocks)
 *       |
 *   [Thread 3 – WAV Writer]
 *       | Drains g_pcm_queue
 *       | Writes to WavWriter (buffered file I/O)
 *
 * Rules:
 *   Thread 1 callback budget: < 5µs. No allocations, no locks.
 *   Thread 2 owns all DSP state (no shared DSP objects).
 *   Thread 3 owns WavWriter.
 *
 * Workflow:
 *   mouseofono.exe --calibrate         (10s noise profile)
 *   mouseofono.exe 15.0 --wiener       (record with spectral subtraction)
 */

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "core/processor_chain.h"
#include "core/ring_buffer.h"
#include "core/signal_reconstructor.h"
#include "core/wav_writer.h"
#include "platforms/windows/raw_input_reader.h"

using namespace mouseofono::core;

// ═══════════════════════════════════════════════════════════════
//  CONSTANTS & DEFAULTS
// ═══════════════════════════════════════════════════════════════
static const char *NOISE_PROFILE_PATH = "noise_profile.bin";
static constexpr double KERNEL_HALFWIN = 0.003;
static constexpr int CALIB_SECS = 10;

// PCM block exchanged between DSP and WAV Writer threads.
// Fixed-size avoids heap alloc in the hot DSP path.
struct PcmBlock {
  static constexpr int FRAMES = 256; // stereo frames per block
  float data[FRAMES * 2];            // interleaved L R
  int count;                         // actual valid frames (≤ FRAMES)
};

// ═══════════════════════════════════════════════════════════════
//  QPC TIMING  (global, read by all threads)
// ═══════════════════════════════════════════════════════════════
static LARGE_INTEGER g_qpc_freq;
static void timing_init() { QueryPerformanceFrequency(&g_qpc_freq); }
static inline double qpc_now() {
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  return (double)t.QuadPart / (double)g_qpc_freq.QuadPart;
}

// ═══════════════════════════════════════════════════════════════
//  POLL RATE ESTIMATOR  (called from Thread 1 callback)
// ═══════════════════════════════════════════════════════════════
static std::atomic<double> g_poll_hz{1000.0};
static std::atomic<bool> g_poll_ready{false};

// NOTE: called from Thread 1 only — no synchronisation needed on locals.
static void update_poll(double t) {
  static double iv[512] = {};
  static int idx = 0;
  static double prev = 0.0;
  if (prev > 0.0) {
    double dt = t - prev;
    if (dt > 5e-5 && dt < 0.02) { // 50µs – 20ms window
      iv[idx & 511] = dt;
      ++idx;
      if (idx >= 128 && (idx & 63) == 0) { // re-estimate every 64 new samples
        double s[512];
        int n = std::min(idx, 512);
        memcpy(s, iv, n * sizeof(double));
        std::sort(s, s + n);
        g_poll_hz.store(1.0 / s[n / 2], std::memory_order_relaxed);
        g_poll_ready.store(true, std::memory_order_relaxed);
      }
    }
  }
  prev = t;
}

// ═══════════════════════════════════════════════════════════════
//  PIPELINE QUEUES
// ═══════════════════════════════════════════════════════════════
static std::atomic<bool> g_running{true};
static RingBuffer<MouseEvent> g_raw_queue; // Thread 1 -> Thread 2
static RingBuffer<PcmBlock> g_pcm_queue;   // Thread 2 -> Thread 3

// Stats
static std::atomic<long long> g_events_total{0};

// ═══════════════════════════════════════════════════════════════
//  RUNTIME OPTIONS
// ═══════════════════════════════════════════════════════════════
static bool g_csv_mode = false;
static bool g_raw_mode = true;
static bool g_use_agc = false;
static bool g_use_wiener = false;
static bool g_calibrate = false;
static double g_bw_override = -1.0;
static int g_fs = 16000;
static int g_fft_n = 512;
static float g_alpha = 1.5f;
static float g_beta = 0.02f;

// ═══════════════════════════════════════════════════════════════
//  CSV
// ═══════════════════════════════════════════════════════════════
static constexpr int CSV_MAX = 20000;
static MouseEvent g_csv_buf[CSV_MAX];
static std::atomic<int> g_csv_n{0};

static void write_csv(const char *path) {
  int n = g_csv_n.load();
  std::ofstream f(path);
  f << "qpc_sec,dx,dy,delta_ms\n";
  for (int i = 0; i < n; ++i) {
    double d = (i > 0) ? (g_csv_buf[i].t - g_csv_buf[i - 1].t) * 1000.0 : 0.0;
    f << g_csv_buf[i].t << "," << (int)g_csv_buf[i].dx << ","
      << (int)g_csv_buf[i].dy << "," << d << "\n";
  }
  if (n < 2) {
    printf("[CSV] Too few events.\n");
    return;
  }
  std::vector<double> dts;
  for (int i = 1; i < n; ++i)
    dts.push_back(g_csv_buf[i].t - g_csv_buf[i - 1].t);
  std::sort(dts.begin(), dts.end());
  double med = dts[dts.size() / 2];
  printf("[CSV] %d events | poll=%.0f Hz | max_safe_bw=%.0f Hz\n", n, 1.0 / med,
         0.45 / med);
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION MODE
// ═══════════════════════════════════════════════════════════════
static void run_calibration() {
  printf("[CALIBRATE] %ds noise capture. Keep mouse STILL.\n\n", CALIB_SECS);
  WienerFilter wf(g_fft_n, g_alpha, g_beta);
  std::vector<float> calib_buf;
  calib_buf.reserve(g_fs * CALIB_SECS + 4096);

  double t_render = -1.0, t_start = -1.0;
  double raw_accum = 0.0;
  DCBlock dc;

  mouseofono::platforms::windows::RawInputReader reader(
      [&](double t, double dx, double dy) {
        update_poll(t);
        g_raw_queue.push({t, dx, dy});
      });
  reader.start();

  const double sample_dt = 1.0 / g_fs;
  const int batch_n = std::max(1, (int)(g_fs * 0.005));

  while (true) {
    MouseEvent e;
    while (g_raw_queue.pop(e)) {
      if (t_render < 0.0) {
        t_render = e.t;
        t_start = e.t;
      }
      raw_accum += e.dx + e.dy;
    }
    if (t_start > 0 && qpc_now() - t_start > CALIB_SECS)
      break;
    if (t_render < 0.0) {
      Sleep(5);
      continue;
    }

    double now = qpc_now();
    if (t_render > now + 0.010) {
      Sleep(1);
      continue;
    }

    for (int i = 0; i < batch_n; ++i) {
      calib_buf.push_back((float)(dc.process(raw_accum / batch_n)));
      t_render += sample_dt;
    }
    raw_accum = 0.0;
  }

  printf("[CALIBRATE] %zu samples collected. Building PSD...\n",
         calib_buf.size());
  wf.feed_calibration(calib_buf.data(), (int)calib_buf.size());
  wf.finalize_calibration();
  if (wf.save_noise_profile(NOISE_PROFILE_PATH))
    printf("[CALIBRATE] Saved -> %s\nNext: mouseofono.exe 15.0 --wiener\n",
           NOISE_PROFILE_PATH);
  else
    printf("[CALIBRATE] ERROR: could not save profile.\n");
}

// ═══════════════════════════════════════════════════════════════
//  THREAD 2 — DSP
// ═══════════════════════════════════════════════════════════════
static void dsp_thread(float gain) {
  // --- Signal chain (owned by this thread only) ---
  DCBlock dcL, dcR;
  AGC agc(g_fs);
  WienerFilter wf(g_fft_n, g_alpha, g_beta);

  if (g_use_wiener) {
    if (!wf.load_noise_profile(NOISE_PROFILE_PATH))
      fprintf(stderr, "[WIENER] WARNING: no profile. Run --calibrate first.\n");
    else
      printf("[WIENER] Profile loaded. alpha=%.2f beta=%.2f fft=%d\n", g_alpha,
             g_beta, g_fft_n);
  }

  SignalReconstructor reconstructor(400.0, KERNEL_HALFWIN, g_fs);

  static constexpr int MAX_EV = 512;
  MouseEvent ev_window[MAX_EV];
  int ev_start = 0, ev_count = 0;

  const double sample_dt = 1.0 / g_fs;
  const int batch_n =
      PcmBlock::FRAMES; // process exactly one PcmBlock at a time

  double t_render = -1.0;
  double current_bw = g_bw_override > 0 ? g_bw_override : 400.0;
  double raw_accX = 0.0, raw_accY = 0.0;

  // Wiener processing buffers (reused, no alloc in loop)
  std::vector<float> batchL(batch_n), batchR(batch_n);

  printf("[DSP] Started @ %d Hz | mode=%s\n", g_fs,
         g_raw_mode ? "RAW" : "SINC");

  while (g_running || !g_raw_queue.empty()) {
    // Drain raw event queue
    {
      MouseEvent e;
      while (g_raw_queue.pop(e)) {
        if (t_render < 0.0)
          t_render = e.t - KERNEL_HALFWIN;
        if (g_raw_mode) {
          raw_accX += e.dx;
          raw_accY += e.dy;
        } else {
          int slot = (ev_start + ev_count) % MAX_EV;
          if (ev_count < MAX_EV)
            ev_count++;
          else
            ev_start = (ev_start + 1) % MAX_EV;
          ev_window[slot] = e;
        }
      }
    }

    if (t_render < 0.0) {
      Sleep(1);
      continue;
    }

    // Adaptive bandwidth update (only after poll is stable)
    if (g_bw_override < 0 && g_poll_ready.load()) {
      double safe_bw = 0.45 * g_poll_hz.load();
      if (std::abs(safe_bw - current_bw) > 5.0) {
        current_bw = safe_bw;
        reconstructor = SignalReconstructor(current_bw, KERNEL_HALFWIN, g_fs);
        printf("[DSP] Bandwidth adapted -> %.0f Hz\n", current_bw);
      }
    }

    // Don't render more than 10ms ahead of real time
    if (t_render > qpc_now() + 0.010) {
      Sleep(1);
      continue;
    }

    // Evict stale events (sinc mode only)
    if (!g_raw_mode) {
      while (ev_count > 0) {
        if ((t_render - ev_window[ev_start].t) > KERNEL_HALFWIN + sample_dt) {
          ev_start = (ev_start + 1) % MAX_EV;
          ev_count--;
        } else
          break;
      }
    }

    // Generate one PcmBlock worth of samples
    for (int i = 0; i < batch_n; ++i) {
      double xL = 0, xR = 0;
      if (g_raw_mode) {
        xL = raw_accX / batch_n;
        xR = raw_accY / batch_n;
      } else {
        for (int j = 0; j < ev_count; ++j) {
          const MouseEvent &e = ev_window[(ev_start + j) % MAX_EV];
          double v1, v2;
          reconstructor.reconstruct(t_render, &e, 1, v1, v2);
          xL += v1;
          xR += v2;
        }
      }
      batchL[i] = (float)(dcL.process(xL) * gain);
      batchR[i] = (float)(dcR.process(xR) * gain);
      t_render += sample_dt;
    }
    if (g_raw_mode) {
      raw_accX = 0.0;
      raw_accY = 0.0;
    }

    // Wiener filter (both channels in-place)
    if (g_use_wiener && wf.is_calibrated()) {
      wf.process_inplace(batchL.data(), batch_n);
      wf.process_inplace(batchR.data(), batch_n);
    }

    // Build PcmBlock and push to writer queue
    PcmBlock blk;
    blk.count = batch_n;
    for (int i = 0; i < batch_n; ++i) {
      float fL = batchL[i], fR = batchR[i];
      if (g_use_agc) {
        float g = agc.process(std::max(std::abs(fL), std::abs(fR)));
        fL *= g;
        fR *= g;
      }
      blk.data[i * 2] = soft_clip(fL);
      blk.data[i * 2 + 1] = soft_clip(fR);
    }
    // Spin-wait if pcm queue is full (rare, writer is fast)
    while (!g_pcm_queue.push(blk) && g_running)
      Sleep(0);
  }

  printf("[DSP] Done. poll=%.0f Hz | bw=%.0f Hz\n", g_poll_hz.load(),
         current_bw);
}

// ═══════════════════════════════════════════════════════════════
//  THREAD 3 — WAV WRITER
// ═══════════════════════════════════════════════════════════════
static void writer_thread() {
  WavWriter writer;
  if (!writer.open("output.wav", g_fs, 2)) {
    fprintf(stderr, "[WRITER] Failed to open output.wav\n");
    return;
  }
  printf("[WRITER] output.wav opened @ %d Hz stereo\n", g_fs);

  std::vector<float> tmp;
  tmp.reserve(PcmBlock::FRAMES * 2 * 8);

  while (g_running || !g_pcm_queue.empty()) {
    PcmBlock blk;
    if (g_pcm_queue.pop(blk)) {
      for (int i = 0; i < blk.count * 2; ++i)
        tmp.push_back(blk.data[i]);
      if ((int)tmp.size() >= PcmBlock::FRAMES * 2 * 4) {
        writer.write(tmp);
        tmp.clear();
      }
    } else {
      Sleep(1);
    }
  }

  if (!tmp.empty())
    writer.write(tmp);
  writer.close();
  printf("[WRITER] Closed.\n");
}

// ═══════════════════════════════════════════════════════════════
//  STATS THREAD
// ═══════════════════════════════════════════════════════════════
static void stats_thread() {
  while (g_running) {
    Sleep(1000);
    if (!g_running)
      break;
    printf("[STATS] poll=%.0f Hz | events=%lld | pcm_queue=%s\n",
           g_poll_hz.load(), g_events_total.load(),
           g_pcm_queue.empty() ? "OK" : "BACKPRESSURE");
  }
}

// ═══════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════
int main(int argc, char *argv[]) {
  timing_init();
  float gain = 15.0f;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--csv") == 0)
      g_csv_mode = true;
    else if (strcmp(argv[i], "--raw") == 0)
      g_raw_mode = true;
    else if (strcmp(argv[i], "--sinc") == 0)
      g_raw_mode = false;
    else if (strcmp(argv[i], "--agc") == 0)
      g_use_agc = true;
    else if (strcmp(argv[i], "--wiener") == 0)
      g_use_wiener = true;
    else if (strcmp(argv[i], "--calibrate") == 0)
      g_calibrate = true;
    else if (strcmp(argv[i], "--bw") == 0 && i + 1 < argc)
      g_bw_override = atof(argv[++i]);
    else if (strcmp(argv[i], "--fs") == 0 && i + 1 < argc)
      g_fs = atoi(argv[++i]);
    else if (strcmp(argv[i], "--fft") == 0 && i + 1 < argc)
      g_fft_n = atoi(argv[++i]);
    else if (strcmp(argv[i], "--alpha") == 0 && i + 1 < argc)
      g_alpha = (float)atof(argv[++i]);
    else if (strcmp(argv[i], "--beta") == 0 && i + 1 < argc)
      g_beta = (float)atof(argv[++i]);
    else
      gain = (float)atof(argv[i]);
  }

  printf("Mouseofono v1.3 (3-Thread Pipeline)\n\n");

  // CSV
  if (g_csv_mode) {
    mouseofono::platforms::windows::RawInputReader reader(
        [](double t, double dx, double dy) {
          update_poll(t);
          int i = g_csv_n.load(std::memory_order_relaxed);
          if (i < CSV_MAX) {
            g_csv_buf[i] = {t, dx, dy};
            g_csv_n.store(i + 1);
          }
        });
    reader.start();
    printf("Move mouse.. capturing %d events\n", CSV_MAX);
    while (g_csv_n.load() < CSV_MAX)
      Sleep(10);
    write_csv("events.csv");
    return 0;
  }

  // Calibrate
  if (g_calibrate) {
    run_calibration();
    return 0;
  }

  // Audio mode
  printf("gain=%.2f  fs=%d Hz  mode=%s  wiener=%s\n\n", gain, g_fs,
         g_raw_mode ? "RAW" : "SINC", g_use_wiener ? "ON" : "OFF");

  // Thread 1 callback: ONLY push + update_poll
  mouseofono::platforms::windows::RawInputReader reader(
      [](double t, double dx, double dy) {
        update_poll(t); // at-source timestamp
        g_raw_queue.push({t, dx, dy});
        g_events_total.fetch_add(1, std::memory_order_relaxed);
      });

  if (!reader.start()) {
    fprintf(stderr, "Failed to start reader\n");
    return 1;
  }

  std::thread dsp_t(dsp_thread, gain);
  std::thread writer_t(writer_thread);
  std::thread stats_t(stats_thread);

  printf("Recording... Press Enter to stop.\n");
  getchar();

  g_running = false;
  dsp_t.join();
  writer_t.join();
  stats_t.join();

  printf("Done! Saved to output.wav\n");
  return 0;
}
