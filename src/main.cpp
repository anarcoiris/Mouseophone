/*
 * main.cpp — Mouseofono: Mouse as Vibration Sensor (Hybrid Windows/Linux)
 *
 * Architecture:
 *   RawInput thread -> SPSC RingBuffer -> QPC-aligned Renderer -> WavWriter
 *
 * Sensitivity improvements applied:
 *   - Adaptive bandwidth: auto-set to 0.45 * measured_poll_hz after warmup
 *   - Raw amplitude mode (--raw): bypass sinc reconstruction for sparse events;
 *     accumulates dx/dy directly per audio sample bucket (better for vibration)
 *   - AGC disabled by default in raw mode (--no-agc) to preserve amplitude info
 *   - CLI flags: gain, --csv, --raw, --bw <hz>, --no-agc
 *
 * Pre-flight checklist (maximize sensitivity):
 *   1. G HUB: set DPI to maximum (8000 DPI) and polling to 1000 Hz
 *   2. Windows: Mouse -> Pointer Options -> uncheck "Enhance pointer precision"
 *   3. Run --csv first: mouseofono.exe --csv -> inspect events.csv median ~1ms
 *   4. Place mouse on resonant surface (speaker cone, thin board)
 *   5. mouseofono.exe 10.0 --raw
 *
 * NOTE: If output.wav is empty, ensure the mouse is NOT in usbipd 'Shared'
 * state. Run 'usbipd unbind --busid <X>' as Administrator first.
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
static constexpr int FS = 48000;
static constexpr double KERNEL_HALFWIN = 0.003; // sinc ±3ms window
static constexpr double DEFAULT_BANDWIDTH = 400.0;

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

// Poll rate estimator
static std::atomic<double> g_poll_hz{1000.0};
static std::atomic<bool> g_poll_ready{false}; // true after 64 samples

static void update_poll(double t) {
  static double intervals[256] = {};
  static int idx = 0;
  static double prev = 0.0;
  if (prev > 0.0) {
    double dt = t - prev;
    if (dt > 0.0 && dt < 0.05) {
      intervals[idx++ & 255] = dt;
      if (idx >= 64) {
        double s[256];
        int n = std::min(idx, 256);
        memcpy(s, intervals, n * sizeof(double));
        std::sort(s, s + n);
        g_poll_hz.store(1.0 / s[n / 2], std::memory_order_relaxed);
        g_poll_ready.store(true, std::memory_order_relaxed);
      }
    }
  }
  prev = t;
}

// CSV mode
static constexpr int CSV_MAX = 20000;
static MouseEvent g_csv_buf[CSV_MAX];
static std::atomic<int> g_csv_n{0};
static bool g_csv_mode = false;
static bool g_raw_mode = false; // bypass sinc, use direct amplitude
static bool g_use_agc = true;
static double g_bw_override = -1.0; // <0 = auto

// ═══════════════════════════════════════════════════════════════
//  CSV WRITER
// ═══════════════════════════════════════════════════════════════
static void write_csv(const char *path) {
  int n = g_csv_n.load();
  {
    std::ofstream f(path);
    f << "qpc_sec,dx,dy,delta_ms\n";
    for (int i = 0; i < n; ++i) {
      double d = (i > 0) ? (g_csv_buf[i].t - g_csv_buf[i - 1].t) * 1000.0 : 0.0;
      f << g_csv_buf[i].t << "," << (int)g_csv_buf[i].dx << ","
        << (int)g_csv_buf[i].dy << "," << d << "\n";
    }
  }
  std::vector<double> dts;
  dts.reserve(n - 1);
  for (int i = 1; i < n; ++i)
    dts.push_back(g_csv_buf[i].t - g_csv_buf[i - 1].t);
  std::sort(dts.begin(), dts.end());
  double med = dts[dts.size() / 2];
  double p5 = dts[dts.size() * 5 / 100];
  double p95 = dts[dts.size() * 95 / 100];
  double maxbw = 0.45 / med;
  printf("[CSV] %d events -> %s\n", n, path);
  printf("[CSV] Inter-event: median=%.3f ms  p5=%.3f ms  p95=%.3f ms\n",
         med * 1e3, p5 * 1e3, p95 * 1e3);
  printf("[CSV] Poll rate:   %.0f Hz\n", 1.0 / med);
  printf("[CSV] Max safe bandwidth: %.0f Hz  ->  use --bw %.0f\n", maxbw,
         maxbw);
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

  // --- Signal chain objects ---
  DCBlock dcL, dcR;
  AGC agc(FS);
  WavWriter writer;

  if (!writer.open("output.wav", FS, 2)) {
    std::cerr << "Failed to open output.wav\n";
    return;
  }

  // Sinc reconstructor (only used in non-raw mode)
  SignalReconstructor reconstructor(DEFAULT_BANDWIDTH, KERNEL_HALFWIN, FS);

  // Event window for sinc mode
  static constexpr int MAX_EV = 256;
  MouseEvent ev_window[MAX_EV];
  int ev_start = 0, ev_count = 0;

  std::vector<float> out_buf;
  out_buf.reserve(4096);

  const double sample_dt = 1.0 / FS;
  const int batch_n = (int)(FS * 0.005); // 5ms batches

  double t_render = -1.0;
  double current_bw = (g_bw_override > 0) ? g_bw_override : DEFAULT_BANDWIDTH;

  // RAW mode accumulators (dx/dy sum per audio sample bucket)
  double raw_accumX = 0.0, raw_accumY = 0.0;
  double raw_last_event_t = -1.0;

  printf("[AUDIO] Recording... (QPC-aligned @ %d Hz | mode=%s)\n", FS,
         g_raw_mode ? "RAW-AMPLITUDE" : "SINC-RECONSTRUCT");

  while (g_running) {
    // ── Drain SPSC queue ──────────────────────────────────────
    {
      MouseEvent e;
      while (g_queue.pop(e)) {
        update_poll(e.t);

        // Bootstrap render position on first event
        if (t_render < 0.0) {
          t_render = e.t - KERNEL_HALFWIN;
        }

        if (g_raw_mode) {
          // Accumulate directly — will be consumed by renderer below
          raw_accumX += e.dx;
          raw_accumY += e.dy;
          raw_last_event_t = e.t;
        } else {
          // Insert into circular event window for sinc
          int slot = (ev_start + ev_count) % MAX_EV;
          if (ev_count < MAX_EV) {
            ev_count++;
          } else {
            ev_start = (ev_start + 1) % MAX_EV;
          }
          ev_window[slot] = e;
        }
      }
    }

    if (t_render < 0.0) {
      Sleep(1);
      continue;
    }

    // ── Adaptive bandwidth (auto after poll rate stabilises) ──
    if (g_bw_override < 0 && g_poll_ready.load()) {
      double safe_bw = 0.45 * g_poll_hz.load();
      if (safe_bw != current_bw) {
        current_bw = safe_bw;
        reconstructor = SignalReconstructor(current_bw, KERNEL_HALFWIN, FS);
      }
    }

    // ── Avoid runaway: don't render more than 10ms ahead ──────
    double now = qpc_now();
    if (t_render > now + 0.010) {
      Sleep(1);
      continue;
    }

    // ── Evict expired events (sinc mode only) ─────────────────
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

    // ── Generate audio batch ───────────────────────────────────
    for (int i = 0; i < batch_n; ++i) {
      double xL = 0, xR = 0;

      if (g_raw_mode) {
        // Distribute accumulated deltas over all samples in this batch
        // This is essentially a zero-order hold over the sinc window
        xL = raw_accumX / (double)batch_n;
        xR = raw_accumY / (double)batch_n;
        // Clear accumulator once per batch (done after loop below)
      } else {
        for (int j = 0; j < ev_count; ++j) {
          const MouseEvent &e = ev_window[(ev_start + j) % MAX_EV];
          double xval, yval;
          reconstructor.reconstruct(t_render, &e, 1, xval, yval);
          xL += xval;
          xR += yval;
        }
      }

      float fL = (float)(dcL.process(xL) * gain);
      float fR = (float)(dcR.process(xR) * gain);

      if (g_use_agc) {
        float g = agc.process(std::max(std::abs(fL), std::abs(fR)));
        fL = soft_clip(fL * g);
        fR = soft_clip(fR * g);
      } else {
        fL = soft_clip(fL);
        fR = soft_clip(fR);
      }

      out_buf.push_back(fL);
      out_buf.push_back(fR);
      t_render += sample_dt;

      if ((int)out_buf.size() >= 2048) {
        writer.write(out_buf);
        out_buf.clear();
      }
    }

    // Clear raw accumulators after batch
    if (g_raw_mode) {
      raw_accumX = 0.0;
      raw_accumY = 0.0;
    }
  }

  if (!out_buf.empty())
    writer.write(out_buf);
  writer.close();
  printf("[AUDIO] Done. poll=%.0f Hz | bw=%.0f Hz | mode=%s\n",
         g_poll_hz.load(), current_bw, g_raw_mode ? "RAW" : "SINC");
}

// ═══════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════
int main(int argc, char *argv[]) {
  timing_init();

  float gain = 1.0f;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--csv") == 0)
      g_csv_mode = true;
    else if (strcmp(argv[i], "--raw") == 0)
      g_raw_mode = true;
    else if (strcmp(argv[i], "--no-agc") == 0)
      g_use_agc = false;
    else if (strcmp(argv[i], "--bw") == 0 && i + 1 < argc)
      g_bw_override = atof(argv[++i]);
    else
      gain = (float)atof(argv[i]);
  }

  printf("Mouseofono v1.1\n");
  printf("NOTE: If output.wav is empty, run 'usbipd unbind --busid <X>' as "
         "Administrator.\n\n");

  // ── CSV MODE ────────────────────────────────────────────────
  if (g_csv_mode) {
    printf("--csv mode: Capturing %d events...\n", CSV_MAX);
    mouseofono::platforms::windows::RawInputReader reader(
        [](double t, double dx, double dy) {
          int i = g_csv_n.load(std::memory_order_relaxed);
          if (i < CSV_MAX) {
            g_csv_buf[i] = {t, dx, dy};
            g_csv_n.store(i + 1, std::memory_order_relaxed);
          }
        });
    reader.start();
    printf("Move mouse continuously...\n");
    while (g_csv_n.load() < CSV_MAX)
      Sleep(50);
    write_csv("events.csv");
    return 0;
  }

  // ── AUDIO MODE ──────────────────────────────────────────────
  printf("gain=%.2f  mode=%s  bw=%s  agc=%s\n", gain,
         g_raw_mode ? "RAW-AMPLITUDE" : "SINC-RECONSTRUCT",
         g_bw_override > 0 ? std::to_string((int)g_bw_override).c_str()
                           : "auto",
         g_use_agc ? "ON" : "OFF");
  printf("Stereo: L=dx, R=dy\n\n");
  printf("Sensitivity tips:\n");
  printf("  - Set DPI to max in G HUB (8000 DPI)\n");
  printf("  - Disable 'Enhance pointer precision' in Windows Mouse settings\n");
  printf("  - Place mouse on resonant/vibrating surface\n");
  printf("  - Try: mouseofono.exe 15.0 --raw --no-agc\n\n");

  mouseofono::platforms::windows::RawInputReader reader(
      [](double t, double dx, double dy) {
        g_queue.push({t, dx, dy});
        g_events_total.fetch_add(1, std::memory_order_relaxed);
      });

  if (!reader.start()) {
    fprintf(stderr, "Failed to start RawInput reader\n");
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
