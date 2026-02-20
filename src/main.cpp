/*
 * main.cpp — Mouseofono v1.2 (Wiener Filter Edition)
 *
 * New in v1.2:
 *   --calibrate  Capture 10s of mouse jitter noise -> saves noise_profile.bin
 *   --wiener     Load noise_profile.bin and apply spectral subtraction filter
 *   --fs <hz>    Override sample rate (default 16000 for vibration sensing)
 *   --fft <n>    FFT frame size for Wiener filter (default 512, must be power
 * of 2)
 *   --alpha <f>  Over-subtraction factor (default 1.5). Higher = more
 * aggressive noise removal.
 *   --beta <f>   Spectral floor (default 0.02). Prevents musical noise
 * artifacts.
 *
 * Recommended workflow (based on arXiv:2509.13581v2):
 *   1. mouseofono.exe --calibrate          (mouse still, measuring sensor
 * jitter)
 *   2. mouseofono.exe 15.0 --raw --wiener  (record with noise removal active)
 *   3. Import output.wav in Audacity for SNR analysis
 *
 * NOTE: Ensure mouse is NOT in usbipd 'Shared' state before running.
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
#include <iostream>
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
//  CONFIG DEFAULTS
// ═══════════════════════════════════════════════════════════════
static const char *NOISE_PROFILE_PATH = "noise_profile.bin";
static constexpr double KERNEL_HALFWIN = 0.003;
static constexpr int DEFAULT_FS =
    16000; // optimised for speech/vibration per paper
static constexpr int CALIB_SECS = 10;

// ═══════════════════════════════════════════════════════════════
//  QPC TIMING
// ═══════════════════════════════════════════════════════════════
static LARGE_INTEGER g_qpc_freq;
static void timing_init() { QueryPerformanceFrequency(&g_qpc_freq); }
static inline double qpc_now() {
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  return (double)t.QuadPart / (double)g_qpc_freq.QuadPart;
}

// ═══════════════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════════════
static std::atomic<bool> g_running{true};
static RingBuffer<MouseEvent> g_queue;
static std::atomic<long long> g_events_total{0};
static std::atomic<double> g_poll_hz{1000.0};
static std::atomic<bool> g_poll_ready{false};

static void update_poll(double t) {
  static double iv[256] = {};
  static int idx = 0;
  static double prev = 0.0;
  if (prev > 0.0) {
    double dt = t - prev;
    if (dt > 0.0 && dt < 0.05) {
      iv[idx++ & 255] = dt;
      if (idx >= 64) {
        double s[256];
        int n = std::min(idx, 256);
        memcpy(s, iv, n * sizeof(double));
        std::sort(s, s + n);
        g_poll_hz.store(1.0 / s[n / 2], std::memory_order_relaxed);
        g_poll_ready.store(true, std::memory_order_relaxed);
      }
    }
  }
  prev = t;
}

// CSV
static constexpr int CSV_MAX = 20000;
static MouseEvent g_csv_buf[CSV_MAX];
static std::atomic<int> g_csv_n{0};

// Runtime options
static bool g_csv_mode = false;
static bool g_raw_mode = true; // default raw for vibration
static bool g_use_agc = false; // off by default (preserve amplitude for SNR)
static bool g_use_wiener = false;
static bool g_calibrate = false;
static double g_bw_override = -1.0;
static int g_fs = DEFAULT_FS;
static int g_fft_n = 512;
static float g_alpha = 1.5f;
static float g_beta = 0.02f;

// ═══════════════════════════════════════════════════════════════
//  CSV WRITER
// ═══════════════════════════════════════════════════════════════
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
  printf("[CSV] %d events -> %s\n  poll=%.0f Hz | max_bw=%.0f Hz -> use --bw "
         "%.0f\n",
         n, path, 1.0 / med, 0.45 / med, 0.45 / med);
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION MODE — captures noise profile from idle mouse
// ═══════════════════════════════════════════════════════════════
static void run_calibration() {
  printf("[CALIBRATE] Capturing %ds of sensor noise. DO NOT MOVE THE MOUSE.\n",
         CALIB_SECS);
  printf("[CALIBRATE] This will characterise the optical sensor jitter at "
         "current DPI.\n\n");

  WienerFilter wf(g_fft_n, g_alpha, g_beta);
  WavWriter recorder;
  // Record raw signal into a temp buffer for noise PSD estimation
  std::vector<float> calib_buf;
  calib_buf.reserve(g_fs * CALIB_SECS);

  // Use a simple raw-amplitude renderer for calibration recording
  double t_render = -1.0;
  double t_start = -1.0;

  mouseofono::platforms::windows::RawInputReader reader(
      [](double t, double dx, double dy) { g_queue.push({t, dx, dy}); });
  reader.start();

  const double sample_dt = 1.0 / g_fs;
  const int batch_n = (int)(g_fs * 0.005);
  DCBlock dcL;
  double raw_accum = 0.0;

  printf("[CALIBRATE] Recording...\n");
  while (g_running) {
    MouseEvent e;
    while (g_queue.pop(e)) {
      if (t_render < 0.0) {
        t_render = e.t - 0.01;
        t_start = e.t;
      }
      raw_accum += e.dx + e.dy;
      update_poll(e.t);
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
      float s = (float)(dcL.process(raw_accum / batch_n));
      calib_buf.push_back(s);
      t_render += sample_dt;
    }
    raw_accum = 0.0;
  }

  printf("[CALIBRATE] Captured %zu samples. Building noise PSD...\n",
         calib_buf.size());
  wf.feed_calibration(calib_buf.data(), (int)calib_buf.size());
  wf.finalize_calibration();

  if (wf.save_noise_profile(NOISE_PROFILE_PATH)) {
    printf("[CALIBRATE] Noise profile saved to '%s'\n", NOISE_PROFILE_PATH);
    printf("[CALIBRATE] Run: mouseofono.exe 15.0 --raw --wiener\n");
  } else {
    printf("[CALIBRATE] ERROR: Failed to save noise profile.\n");
  }
}

// ═══════════════════════════════════════════════════════════════
//  STATS THREAD
// ═══════════════════════════════════════════════════════════════
static void stats_thread() {
  while (g_running) {
    Sleep(1000);
    if (!g_running)
      break;
    printf("[STATS] poll=~%.0f Hz | events=%lld\n", g_poll_hz.load(),
           g_events_total.load());
  }
}

// ═══════════════════════════════════════════════════════════════
//  AUDIO RENDERER THREAD
// ═══════════════════════════════════════════════════════════════
static void audio_thread(float gain) {
  DCBlock dcL, dcR;
  AGC agc(g_fs);
  WavWriter writer;
  WienerFilter wf(g_fft_n, g_alpha, g_beta);

  if (g_use_wiener) {
    if (!wf.load_noise_profile(NOISE_PROFILE_PATH)) {
      fprintf(stderr,
              "[WIENER] WARNING: Could not load '%s'. Run --calibrate first.\n",
              NOISE_PROFILE_PATH);
      fprintf(stderr, "[WIENER] Continuing without noise filter.\n");
    } else {
      printf("[WIENER] Noise profile loaded. Spectral subtraction active.\n");
      printf("[WIENER] alpha=%.2f beta=%.2f fft_n=%d\n", g_alpha, g_beta,
             g_fft_n);
    }
  }

  if (!writer.open("output.wav", g_fs, 2)) {
    std::cerr << "Failed to open output.wav\n";
    return;
  }

  SignalReconstructor reconstructor(g_bw_override > 0 ? g_bw_override
                                                      : 0.45 * g_poll_hz.load(),
                                    KERNEL_HALFWIN, g_fs);

  static constexpr int MAX_EV = 256;
  MouseEvent ev_window[MAX_EV];
  int ev_start = 0, ev_count = 0;

  std::vector<float> out_buf;
  out_buf.reserve(4096);

  const double sample_dt = 1.0 / g_fs;
  const int batch_n = std::max(1, (int)(g_fs * 0.005));

  double t_render = -1.0;
  double current_bw = g_bw_override > 0 ? g_bw_override : 400.0;
  double raw_accX = 0.0, raw_accY = 0.0;

  printf("[AUDIO] Recording @ %d Hz | %s | wiener=%s\n", g_fs,
         g_raw_mode ? "RAW" : "SINC", g_use_wiener ? "ON" : "OFF");

  while (g_running) {
    {
      MouseEvent e;
      while (g_queue.pop(e)) {
        update_poll(e.t);
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
        g_events_total.fetch_add(1, std::memory_order_relaxed);
      }
    }

    if (t_render < 0.0) {
      Sleep(1);
      continue;
    }

    // Adaptive bandwidth update
    if (g_bw_override < 0 && g_poll_ready.load()) {
      double safe_bw = 0.45 * g_poll_hz.load();
      if (std::abs(safe_bw - current_bw) > 1.0) {
        current_bw = safe_bw;
        reconstructor = SignalReconstructor(current_bw, KERNEL_HALFWIN, g_fs);
      }
    }

    if (t_render > qpc_now() + 0.010) {
      Sleep(1);
      continue;
    }

    // Evict stale sinc events
    if (!g_raw_mode) {
      while (ev_count > 0) {
        const MouseEvent &oldest = ev_window[ev_start];
        if ((t_render - oldest.t) > KERNEL_HALFWIN + sample_dt) {
          ev_start = (ev_start + 1) % MAX_EV;
          ev_count--;
        } else
          break;
      }
    }

    // Generate batch
    std::vector<float> batchL(batch_n), batchR(batch_n);
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

    // Apply Wiener filter to each channel independently
    if (g_use_wiener && wf.is_calibrated()) {
      wf.process_inplace(batchL.data(), batch_n);
      wf.process_inplace(batchR.data(), batch_n);
    }

    // AGC + clip, then interleave
    for (int i = 0; i < batch_n; ++i) {
      float fL = batchL[i], fR = batchR[i];
      if (g_use_agc) {
        float g = agc.process(std::max(std::abs(fL), std::abs(fR)));
        fL *= g;
        fR *= g;
      }
      out_buf.push_back(soft_clip(fL));
      out_buf.push_back(soft_clip(fR));
      if ((int)out_buf.size() >= 2048) {
        writer.write(out_buf);
        out_buf.clear();
      }
    }
  }

  if (!out_buf.empty())
    writer.write(out_buf);
  writer.close();
  printf("[AUDIO] Done. poll=%.0f Hz | bw=%.0f Hz | fs=%d\n", g_poll_hz.load(),
         current_bw, g_fs);
}

// ═══════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════
int main(int argc, char *argv[]) {
  timing_init();
  float gain = 15.0f; // default higher for vibration sensing

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

  printf("Mouseofono v1.2\n\n");

  // ── CSV MODE ─────────────────────────────────────────────────
  if (g_csv_mode) {
    printf("--csv: Capturing %d events...\n", CSV_MAX);
    mouseofono::platforms::windows::RawInputReader reader(
        [](double t, double dx, double dy) {
          int i = g_csv_n.load(std::memory_order_relaxed);
          if (i < CSV_MAX) {
            g_csv_buf[i] = {t, dx, dy};
            g_csv_n.store(i + 1);
          }
        });
    reader.start();
    printf("Move mouse continuously...\n");
    while (g_csv_n.load() < CSV_MAX)
      Sleep(50);
    write_csv("events.csv");
    return 0;
  }

  // ── CALIBRATE MODE ────────────────────────────────────────────
  if (g_calibrate) {
    run_calibration();
    return 0;
  }

  // ── AUDIO MODE ────────────────────────────────────────────────
  printf("gain=%.2f  fs=%d Hz  mode=%s  wiener=%s  agc=%s\n", gain, g_fs,
         g_raw_mode ? "RAW" : "SINC", g_use_wiener ? "ON" : "OFF",
         g_use_agc ? "ON" : "OFF");
  printf("\nWorkflow:\n");
  printf("  1. mouseofono.exe --calibrate    (noise profile, mouse still)\n");
  printf("  2. mouseofono.exe 15.0 --wiener  (record with filter active)\n\n");

  mouseofono::platforms::windows::RawInputReader reader(
      [](double t, double dx, double dy) { g_queue.push({t, dx, dy}); });
  if (!reader.start()) {
    fprintf(stderr, "Failed to start reader\n");
    return 1;
  }

  std::thread audio_t(audio_thread, gain);
  std::thread stats_t(stats_thread);

  printf("Recording... Press Enter to stop.\n");
  getchar();

  g_running = false;
  audio_t.join();
  stats_t.join();
  printf("Done! Saved to output.wav\n");
  return 0;
}
