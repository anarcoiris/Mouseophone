#ifndef MOUSEOFONO_PLATFORMS_WINDOWS_RAW_INPUT_READER_H
#define MOUSEOFONO_PLATFORMS_WINDOWS_RAW_INPUT_READER_H

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "../../core/ring_buffer.h"
#include <atomic>
#include <functional>
#include <windows.h>


namespace mouseofono::platforms::windows {

using EventCallback = std::function<void(double t, double dx, double dy)>;

class RawInputReader {
public:
  RawInputReader(EventCallback callback);
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
#endif // MOUSEOFONO_PLATFORMS_WINDOWS_RAW_INPUT_READER_H
