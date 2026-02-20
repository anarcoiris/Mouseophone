#ifndef MOUSEOFONO_PLATFORMS_LINUX_EVDEV_READER_H
#define MOUSEOFONO_PLATFORMS_LINUX_EVDEV_READER_H

#ifndef _WIN32

#include <atomic>
#include <fcntl.h>
#include <functional>
#include <linux/input.h>
#include <string>
#include <thread>
#include <unistd.h>


namespace mouseofono::platforms::linux {

using EventCallback = std::function<void(double t, double dx, double dy)>;

class LinuxEvdevReader {
public:
  LinuxEvdevReader(EventCallback callback);
  ~LinuxEvdevReader();

  bool start(const std::string &devicePath);
  void stop();

private:
  void readLoop(const std::string &devicePath);
  double getTime();

  EventCallback m_callback;
  std::atomic<bool> m_running;
  int m_fd;
};

} // namespace mouseofono::platforms::linux

#endif // !_WIN32
#endif // MOUSEOFONO_PLATFORMS_LINUX_EVDEV_READER_H
