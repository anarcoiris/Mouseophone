/*
 * mouse_mic.cpp — Logitech G203 (any HID mouse) as vibration sensor
 *
 * Architecture:
 *   RawInput thread -> SPSC event queue -> Audio callback (PortAudio)
 *   Signal chain:  dx (L) / dy (R) -> band-limited reconstruction
 *                  -> DC block -> AGC -> soft clip -> stereo PCM out
 *
 * Modes:
 *   mouse_mic.exe --csv        Dump 20k events to events.csv (polling analysis)
 *   mouse_mic.exe [gain]       Real-time stereo audio + output.wav (default)
 *
 * Dependencies:
 *   vcpkg install portaudio
 *
 * Build:
 *   cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
 *   cmake --build build --config Release
 *
 * Pre-flight checklist:
 *   1. G HUB: polling rate 1000 Hz, DPI fixed at 800
 *   2. Windows: Mouse -> Pointer Options -> uncheck "Enhance pointer precision"
 *   3. Run --csv first, check events.csv median delta ~1ms
 *   4. Run audio mode with mouse on speaker cone / resonant surface
 */

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <windows.h>
#include <portaudio.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <thread>
#include <vector>

// ═══════════════════════════════════════════════════════════════════
//  CONFIGURATION  (tune these first)
// ═══════════════════════════════════════════════════════════════════

static constexpr int    FS             = 48000;  // Audio sample rate (Hz)
static constexpr double BANDWIDTH      = 400.0;  // Reconstruction bandwidth (Hz)
                                                 // max safe = 0.45 * poll_rate
                                                 // @ 1000 Hz mouse -> 450 Hz max
static constexpr double KERNEL_HALFWIN = 0.003;  // Sinc half-window (seconds)
                                                 // ±3ms -> ~6 events @ 1000Hz
static constexpr int    RING_CAP       = 1 << 14;// SPSC capacity (power of 2)
static constexpr int    PA_FRAMES      = 512;    // PortAudio buffer (~10ms @ 48k)

// AGC
static constexpr float  AGC_TARGET     = 0.40f;  // Target RMS output level
static constexpr float  AGC_MAX_GAIN   = 300.0f; // Hard ceiling on gain
static constexpr float  AGC_ATTACK_TC  = 0.005f; // Attack time constant (sec)
static constexpr float  AGC_RELEASE_TC = 0.400f; // Release time constant (sec)

// DC blocker pole (0.995 ~ 7 Hz cutoff @ 48k)
static constexpr double DC_POLE        = 0.9950;

// WAV recording
static constexpr int    WAV_MAX_SEC    = 60;


// ═══════════════════════════════════════════════════════════════════
//  TIMING
// ═══════════════════════════════════════════════════════════════════

static LARGE_INTEGER g_qpc_freq;
static void timing_init() { QueryPerformanceFrequency(&g_qpc_freq); }
static inline double qpc_now() {
    LARGE_INTEGER t; QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)g_qpc_freq.QuadPart;
}


// ═══════════════════════════════════════════════════════════════════
//  MOUSE EVENT  +  SPSC RING BUFFER  (single producer, single consumer)
// ═══════════════════════════════════════════════════════════════════

struct MouseEvent { double t, dx, dy; };

struct EventQueue {
    MouseEvent            buf[RING_CAP];
    std::atomic<uint32_t> head{0}; // written by producer (RawInput thread)
    std::atomic<uint32_t> tail{0}; // written by consumer (audio thread)

    bool push(const MouseEvent& e) {
        uint32_t h    = head.load(std::memory_order_relaxed);
        uint32_t next = (h + 1) & (RING_CAP - 1);
        if (next == tail.load(std::memory_order_acquire)) return false; // full
        buf[h] = e;
        head.store(next, std::memory_order_release);
        return true;
    }
    bool pop(MouseEvent& e) {
        uint32_t t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire)) return false; // empty
        e = buf[t];
        tail.store((t + 1) & (RING_CAP - 1), std::memory_order_release);
        return true;
    }
};

static EventQueue g_queue;


// ═══════════════════════════════════════════════════════════════════
//  RUN MODE
// ═══════════════════════════════════════════════════════════════════

enum class Mode { AUDIO, CSV };
static Mode  g_mode      = Mode::AUDIO;
static float g_user_gain = 1.0f;

// CSV capture buffer
static constexpr int    CSV_MAX = 20000;
static MouseEvent       g_csv_buf[CSV_MAX];
static std::atomic<int> g_csv_n{0};


// ═══════════════════════════════════════════════════════════════════
//  POLL RATE ESTIMATOR  (updated by RawInput thread)
// ═══════════════════════════════════════════════════════════════════

static std::atomic<double> g_poll_hz{1000.0};

static void update_poll(double t) {
    static double intervals[256] = {};
    static int    idx  = 0;
    static double prev = 0.0;
    if (prev > 0.0) {
        double dt = t - prev;
        if (dt > 0.0 && dt < 0.05) {
            intervals[idx++ & 255] = dt;
            if (idx >= 64) {
                double s[256]; int n = std::min(idx, 256);
                memcpy(s, intervals, n * sizeof(double));
                std::sort(s, s + n);
                g_poll_hz.store(1.0 / s[n / 2], std::memory_order_relaxed);
            }
        }
    }
    prev = t;
}


// ═══════════════════════════════════════════════════════════════════
//  BAND-LIMITED RECONSTRUCTION KERNEL
//  h(dt) = sinc(2*B*dt) * hann(dt / W)
//
//  Each mouse event at time t_k with displacement a_k contributes:
//    a_k * h(t_sample - t_k)
//  to the reconstructed signal at time t_sample.
// ═══════════════════════════════════════════════════════════════════

static inline double sinc_norm(double x) {
    if (std::abs(x) < 1e-12) return 1.0;
    double px = M_PI * x;
    return std::sin(px) / px;
}

static inline double kernel(double dt) {
    if (std::abs(dt) >= KERNEL_HALFWIN) return 0.0;
    double hann = 0.5 * (1.0 + std::cos(M_PI * dt / KERNEL_HALFWIN));
    return sinc_norm(2.0 * BANDWIDTH * dt) * hann;
}


// ═══════════════════════════════════════════════════════════════════
//  DSP
// ═══════════════════════════════════════════════════════════════════

struct DCBlock {
    double xp = 0, yp = 0;
    double process(double x) {
        double y = x - xp + DC_POLE * yp;
        xp = x; yp = y; return y;
    }
};

struct AGC {
    float env  = 1e-3f;
    float gain = 1.0f;
    const float atk = 1.0f - std::exp(-1.0f / (FS * AGC_ATTACK_TC));
    const float rel = 1.0f - std::exp(-1.0f / (FS * AGC_RELEASE_TC));

    // Feed peak of both channels; returns current gain
    float process(float peak) {
        float a = std::abs(peak);
        env  += (a > env ? atk : rel) * (a - env);
        env   = std::max(env, 1e-6f);
        gain  = std::min(AGC_TARGET / env, AGC_MAX_GAIN);
        return gain;
    }
};

static inline float soft_clip(float x) { return std::tanh(x); }


// ═══════════════════════════════════════════════════════════════════
//  AUDIO STATE  (shared with PortAudio callback)
// ═══════════════════════════════════════════════════════════════════

struct AudioState {
    static constexpr int MAX_EV = 256; // event window size

    MouseEvent ev[MAX_EV];
    int        ev_count = 0;
    int        ev_start = 0;

    double t_qpc_start = -1.0; // QPC time at first callback
    double t_pa_start  = -1.0; // Pa stream time at first callback

    DCBlock dcL, dcR;          // separate DC blockers per channel
    AGC     agc;               // shared (level sense from both channels)

    std::vector<float> wav;    // interleaved stereo float32
    bool wav_full = false;

    std::atomic<long long> n_events{0};
    std::atomic<long long> n_samples{0};
    std::atomic<int>       window_hits{0};
};

static AudioState g_audio;


// ═══════════════════════════════════════════════════════════════════
//  PORTAUDIO CALLBACK
// ═══════════════════════════════════════════════════════════════════

static int audio_callback(
    const void*, void* output, unsigned long frames,
    const PaStreamCallbackTimeInfo* ti,
    PaStreamCallbackFlags, void* ud)
{
    AudioState& s   = *static_cast<AudioState*>(ud);
    float*      out = static_cast<float*>(output);

    // Establish time baseline on first call
    if (s.t_qpc_start < 0.0) {
        s.t_qpc_start = qpc_now();
        s.t_pa_start  = ti->outputBufferDacTime;
    }

    // Drain SPSC queue into local event window
    {
        MouseEvent e;
        while (s.ev_count < AudioState::MAX_EV && g_queue.pop(e)) {
            s.ev[(s.ev_start + s.ev_count++) % AudioState::MAX_EV] = e;
            s.n_events.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // Convert PortAudio time to QPC-aligned absolute time
    double t_dac = s.t_qpc_start + (ti->outputBufferDacTime - s.t_pa_start);

    for (unsigned long i = 0; i < frames; ++i) {
        double ts = t_dac + (double)i / FS;

        // Evict events older than kernel window
        while (s.ev_count > 0 &&
               (ts - s.ev[s.ev_start].t) > KERNEL_HALFWIN + 1.0 / FS)
        {
            s.ev_start = (s.ev_start + 1) % AudioState::MAX_EV;
            s.ev_count--;
        }

        // Band-limited reconstruction
        // dx -> Left channel,  dy -> Right channel
        double xL = 0, xR = 0;
        int hits = 0;
        for (int j = 0; j < s.ev_count; ++j) {
            const MouseEvent& e = s.ev[(s.ev_start + j) % AudioState::MAX_EV];
            double dt = ts - e.t;
            if (std::abs(dt) <= KERNEL_HALFWIN) {
                double h = kernel(dt);
                xL += e.dx * h;
                xR += e.dy * h;
                hits++;
            }
        }
        if (i == 0) s.window_hits.store(hits, std::memory_order_relaxed);

        // DSP: DC block -> AGC -> soft clip
        float fL = (float)(s.dcL.process(xL) * g_user_gain);
        float fR = (float)(s.dcR.process(xR) * g_user_gain);
        float g  = s.agc.process(std::max(std::abs(fL), std::abs(fR)));

        out[2*i+0] = soft_clip(fL * g); // Left  = dx
        out[2*i+1] = soft_clip(fR * g); // Right = dy

        // WAV dump (interleaved stereo)
        if (!s.wav_full) {
            s.wav.push_back(out[2*i+0]);
            s.wav.push_back(out[2*i+1]);
            if ((int)s.wav.size() >= FS * WAV_MAX_SEC * 2)
                s.wav_full = true;
        }
    }

    s.n_samples.fetch_add(frames, std::memory_order_relaxed);
    return paContinue;
}


// ═══════════════════════════════════════════════════════════════════
//  WAV WRITER  (IEEE float32, N channels)
// ═══════════════════════════════════════════════════════════════════

static void write_wav(const char* path, const std::vector<float>& s, int fs, int ch) {
    uint32_t ds = (uint32_t)(s.size() * sizeof(float));
    std::ofstream f(path, std::ios::binary);
    auto w2 = [&](uint16_t v){ f.write((char*)&v, 2); };
    auto w4 = [&](uint32_t v){ f.write((char*)&v, 4); };
    f.write("RIFF", 4); w4(36 + ds);
    f.write("WAVE", 4);
    f.write("fmt ", 4);  w4(16);
    w2(3); w2((uint16_t)ch);
    w4((uint32_t)fs);
    w4((uint32_t)(fs * ch * sizeof(float)));
    w2((uint16_t)(ch * sizeof(float))); w2(32);
    f.write("data", 4);  w4(ds);
    f.write((char*)s.data(), ds);

    int frames = (int)s.size() / ch;
    printf("[WAV] %d frames / %.1f s  %s  -> %s\n",
           frames, (double)frames / fs,
           ch == 2 ? "stereo (L=dx, R=dy)" : "mono", path);
}


// ═══════════════════════════════════════════════════════════════════
//  CSV WRITER  (analysis / calibration mode)
// ═══════════════════════════════════════════════════════════════════

static void write_csv(const char* path) {
    int n = g_csv_n.load();
    {
        std::ofstream f(path);
        f << "qpc_sec,dx,dy,delta_ms\n";
        for (int i = 0; i < n; ++i) {
            double d = (i > 0) ? (g_csv_buf[i].t - g_csv_buf[i-1].t) * 1000.0 : 0.0;
            f << g_csv_buf[i].t << ","
              << (int)g_csv_buf[i].dx << ","
              << (int)g_csv_buf[i].dy << ","
              << d << "\n";
        }
    }
    // Compute stats
    std::vector<double> dts; dts.reserve(n - 1);
    for (int i = 1; i < n; ++i) dts.push_back(g_csv_buf[i].t - g_csv_buf[i-1].t);
    std::sort(dts.begin(), dts.end());
    double med = dts[dts.size() / 2];
    double p5  = dts[dts.size() * 5  / 100];
    double p95 = dts[dts.size() * 95 / 100];

    printf("[CSV] %d events -> %s\n", n, path);
    printf("[CSV] Inter-event interval:  median=%.3f ms  p5=%.3f ms  p95=%.3f ms\n",
           med * 1e3, p5 * 1e3, p95 * 1e3);
    printf("[CSV] Effective poll rate:   median=%.0f Hz\n", 1.0 / med);
    printf("[CSV] Max safe bandwidth:    %.0f Hz  (0.45 * poll)\n", 0.45 / med);
    printf("[CSV] Recommendation: set BANDWIDTH=%.0f in mouse_mic.cpp\n",
           std::min(400.0, 0.45 / med));
}


// ═══════════════════════════════════════════════════════════════════
//  RAWINPUT THREAD  (dedicated message loop — nothing else here)
// ═══════════════════════════════════════════════════════════════════

static LRESULT CALLBACK ri_wndproc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg != WM_INPUT) return DefWindowProcA(hwnd, msg, wp, lp);

    UINT sz = 0;
    GetRawInputData((HRAWINPUT)lp, RID_INPUT, nullptr, &sz, sizeof(RAWINPUTHEADER));

    BYTE stack[128];
    BYTE* buf = sz <= sizeof(stack) ? stack : new BYTE[sz];

    if (GetRawInputData((HRAWINPUT)lp, RID_INPUT, buf, &sz, sizeof(RAWINPUTHEADER)) == sz) {
        auto* raw = (RAWINPUT*)buf;
        if (raw->header.dwType == RIM_TYPEMOUSE) {
            double t  = qpc_now();
            double dx = raw->data.mouse.lLastX;
            double dy = raw->data.mouse.lLastY;
            update_poll(t);

            if (dx != 0.0 || dy != 0.0) {
                MouseEvent e{t, dx, dy};
                if (g_mode == Mode::CSV) {
                    int i = g_csv_n.load(std::memory_order_relaxed);
                    if (i < CSV_MAX) {
                        g_csv_buf[i] = e;
                        g_csv_n.store(i + 1, std::memory_order_relaxed);
                    }
                } else {
                    g_queue.push(e);
                }
            }
        }
    }
    if (buf != stack) delete[] buf;
    return 0;
}

static void rawinput_thread() {
    WNDCLASSA wc    = {};
    wc.lpfnWndProc   = ri_wndproc;
    wc.hInstance     = GetModuleHandleA(nullptr);
    wc.lpszClassName = "MouseMicRI";
    RegisterClassA(&wc);

    HWND hwnd = CreateWindowExA(0, "MouseMicRI", nullptr, 0, 0, 0, 0, 0,
                                HWND_MESSAGE, nullptr, GetModuleHandleA(nullptr), nullptr);

    RAWINPUTDEVICE rid;
    rid.usUsagePage = 0x01; // Generic desktop
    rid.usUsage     = 0x02; // Mouse
    rid.dwFlags     = RIDEV_INPUTSINK; // Receive even when not in focus
    rid.hwndTarget  = hwnd;
    RegisterRawInputDevices(&rid, 1, sizeof(rid));

    MSG msg;
    while (GetMessageA(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
}


// ═══════════════════════════════════════════════════════════════════
//  STATS THREAD
// ═══════════════════════════════════════════════════════════════════

static std::atomic<bool> g_running{true};

static void stats_thread() {
    while (g_running.load()) {
        Sleep(1000);
        printf("[STATS] poll=%.0f Hz | events=%lld | samples=%lld | "
               "hits/frame=%d | agc_gain=%.1f\n",
               g_poll_hz.load(),
               g_audio.n_events.load(),
               g_audio.n_samples.load(),
               g_audio.window_hits.load(),
               g_audio.agc.gain);
    }
}


// ═══════════════════════════════════════════════════════════════════
//  MAIN
// ═══════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    timing_init();

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--csv") == 0) g_mode = Mode::CSV;
        else g_user_gain = (float)atof(argv[i]);
    }

    // ── CSV MODE: capture events, print polling stats, exit ─────────
    if (g_mode == Mode::CSV) {
        printf("mouse_mic --csv\n");
        printf("Move mouse continuously. Capturing %d events...\n\n", CSV_MAX);
        std::thread(rawinput_thread).detach();
        while (g_csv_n.load() < CSV_MAX) Sleep(50);
        write_csv("events.csv");
        printf("\nOpen events.csv and inspect delta_ms column.\n");
        printf("Then update BANDWIDTH in mouse_mic.cpp accordingly.\n");
        return 0;
    }

    // ── AUDIO MODE ───────────────────────────────────────────────────
    printf("mouse_mic — AUDIO MODE\n");
    printf("  gain=%.2f  |  bandwidth=%.0f Hz  |  kernel_halfwin=±%.0f ms\n",
           g_user_gain, BANDWIDTH, KERNEL_HALFWIN * 1000.0);
    printf("  Stereo: Left=dx  Right=dy\n");
    printf("  Recording up to %d s -> output.wav\n\n", WAV_MAX_SEC);

    g_audio.wav.reserve(FS * WAV_MAX_SEC * 2);
    std::thread(rawinput_thread).detach();
    std::thread(stats_thread).detach();

    PaError err = Pa_Initialize();
    if (err != paNoError) {
        fprintf(stderr, "PortAudio init: %s\n", Pa_GetErrorText(err)); return 1;
    }

    PaStreamParameters op;
    op.device                    = Pa_GetDefaultOutputDevice();
    op.channelCount              = 2;
    op.sampleFormat              = paFloat32;
    op.suggestedLatency          = Pa_GetDeviceInfo(op.device)->defaultLowOutputLatency;
    op.hostApiSpecificStreamInfo = nullptr;

    PaStream* stream;
    err = Pa_OpenStream(&stream, nullptr, &op, FS, PA_FRAMES,
                        paClipOff, audio_callback, &g_audio);
    if (err != paNoError) {
        fprintf(stderr, "Pa_OpenStream: %s\n", Pa_GetErrorText(err));
        Pa_Terminate(); return 1;
    }

    Pa_StartStream(stream);
    printf("[AUDIO] Started: %s\n", Pa_GetDeviceInfo(op.device)->name);
    printf("Press Enter to stop...\n");
    getchar();

    g_running.store(false);
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    if (!g_audio.wav.empty())
        write_wav("output.wav", g_audio.wav, FS, 2);

    return 0;
}
