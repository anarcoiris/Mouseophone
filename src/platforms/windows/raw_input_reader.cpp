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
  if (msg == WM_INPUT) {
    RawInputReader *reader = reinterpret_cast<RawInputReader *>(
        GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (!reader)
      return DefWindowProc(hwnd, msg, wp, lp);

    UINT sz = 0;
    GetRawInputData((HRAWINPUT)lp, RID_INPUT, nullptr, &sz,
                    sizeof(RAWINPUTHEADER));

    if (sz > 0) {
      std::vector<BYTE> buf(sz);
      if (GetRawInputData((HRAWINPUT)lp, RID_INPUT, buf.data(), &sz,
                          sizeof(RAWINPUTHEADER)) == sz) {
        RAWINPUT *raw = reinterpret_cast<RAWINPUT *>(buf.data());
        if (raw->header.dwType == RIM_TYPEMOUSE) {
          double t = reader->queryTime();
          double dx = static_cast<double>(raw->data.mouse.lLastX);
          double dy = static_cast<double>(raw->data.mouse.lLastY);

          if (dx != 0.0 || dy != 0.0) {
            reader->m_callback(t, dx, dy);
          }
        }
      }
    }
    return 0;
  }
  return DefWindowProc(hwnd, msg, wp, lp);
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
  rid.usUsagePage = 0x01;
  rid.usUsage = 0x02;
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
  std::thread([this]() { messageLoop(); }).detach();
  return true;
}

void RawInputReader::stop() {
  m_running = false;
  if (m_hwnd) {
    PostMessage(m_hwnd, WM_CLOSE, 0, 0);
  }
}

} // namespace mouseofono::platforms::windows
#endif
