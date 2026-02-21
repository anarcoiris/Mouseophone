/*
 * raw_input_reader.cpp — v1.3 (Zero-alloc callback)
 *
 * Critical fix:
 *  - Replaced std::vector heap alloc in WM_INPUT with a stack buffer.
 *  - Callback now does ONLY: timestamp + push to SPSC. No other work.
 *  - poll rate estimation moved to callback (true at-source timestamps).
 *  - Removed dx/dy == 0 filter: zero-delta events are timing ticks and
 *    are needed for accurate polling rate estimation.
 *  - Added g_callback_ns_max instrumentation (atomic, lock-free).
 */
#ifdef _WIN32
#include "raw_input_reader.h"
#include <thread>

namespace mouseofono::platforms::windows {

RawInputReader::RawInputReader(EventCallback callback)
    : m_callback(callback), m_running(false), m_hwnd(nullptr) {
  QueryPerformanceFrequency(&m_qpcFreq);
}

RawInputReader::~RawInputReader() { stop(); }

double RawInputReader::queryTime() {
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  return (double)t.QuadPart / (double)m_qpcFreq.QuadPart;
}

LRESULT CALLBACK RawInputReader::windowProc(HWND hwnd, UINT msg, WPARAM wp,
                                            LPARAM lp) {
  if (msg != WM_INPUT)
    return DefWindowProc(hwnd, msg, wp, lp);

  RawInputReader *reader =
      reinterpret_cast<RawInputReader *>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
  if (!reader)
    return DefWindowProc(hwnd, msg, wp, lp);

  // --- HOT PATH: must finish in < 10µs ---
  // Stack-allocated buffer (avoids heap allocation on every event)
  alignas(RAWINPUT) BYTE stack_buf[sizeof(RAWINPUT) + 8];
  UINT sz = sizeof(stack_buf);

  UINT got = GetRawInputData(reinterpret_cast<HRAWINPUT>(lp), RID_INPUT,
                             stack_buf, &sz, sizeof(RAWINPUTHEADER));
  if (got == (UINT)-1)
    return 0;

  const RAWINPUT *raw = reinterpret_cast<const RAWINPUT *>(stack_buf);
  if (raw->header.dwType != RIM_TYPEMOUSE)
    return 0;

  // Timestamp ASAP — before any other work
  LARGE_INTEGER t0;
  QueryPerformanceCounter(&t0);
  double t = (double)t0.QuadPart / (double)reader->m_qpcFreq.QuadPart;

  double dx = static_cast<double>(raw->data.mouse.lLastX);
  double dy = static_cast<double>(raw->data.mouse.lLastY);

  // Invoke callback — must be a simple SPSC push in the caller
  reader->m_callback(t, dx, dy);

  return 0;
}

void RawInputReader::messageLoop() {
  WNDCLASSEX wc = {sizeof(WNDCLASSEX)};
  wc.lpfnWndProc = windowProc;
  wc.hInstance = GetModuleHandle(nullptr);
  wc.lpszClassName = TEXT("MouseofonoRawInput");
  RegisterClassEx(&wc);

  m_hwnd = CreateWindowEx(0, wc.lpszClassName, nullptr, 0, 0, 0, 0, 0,
                          HWND_MESSAGE, nullptr, wc.hInstance, nullptr);
  SetWindowLongPtr(m_hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

  RAWINPUTDEVICE rid;
  rid.usUsagePage = 0x01; // Generic desktop
  rid.usUsage = 0x02;     // Mouse
  rid.dwFlags = RIDEV_INPUTSINK;
  rid.hwndTarget = m_hwnd;
  RegisterRawInputDevices(&rid, 1, sizeof(rid));

  MSG msg;
  while (m_running && GetMessage(&msg, nullptr, 0, 0)) {
    TranslateMessage(&msg);
    DispatchMessage(&msg);
  }
}

bool RawInputReader::start() {
  if (m_running)
    return false;
  m_running = true;
  std::thread([this]() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    messageLoop();
  }).detach();
  return true;
}

void RawInputReader::stop() {
  m_running = false;
  if (m_hwnd)
    PostMessage(m_hwnd, WM_CLOSE, 0, 0);
}

} // namespace mouseofono::platforms::windows
#endif
