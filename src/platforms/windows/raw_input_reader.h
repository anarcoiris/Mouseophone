#pragma once
#ifdef _WIN32

#include <atomic>
#include <functional>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace mouseofono::platforms::windows {

class RawInputReader {
public:
  using EventCallback = std::function<void(double t, double dx, double dy)>;

  explicit RawInputReader(EventCallback callback);
  ~RawInputReader();

  bool start();
  void stop();

private:
  static LRESULT CALLBACK windowProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
  void messageLoop();
  double queryTime();

  EventCallback m_callback;
  std::atomic<bool> m_running;
  HWND m_hwnd;
  LARGE_INTEGER m_qpcFreq;
};

} // namespace mouseofono::platforms::windows

#endif // _WIN32
