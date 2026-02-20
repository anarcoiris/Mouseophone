#ifndef _WIN32
#include "evdev_reader.h"
#include <iostream>
#include <sys/time.h>


namespace mouseofono::platforms::linux {

LinuxEvdevReader::LinuxEvdevReader(EventCallback callback)
    : m_callback(callback), m_running(false), m_fd(-1) {}

LinuxEvdevReader::~LinuxEvdevReader() { stop(); }

double LinuxEvdevReader::getTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

void LinuxEvdevReader::readLoop(const std::string &devicePath) {
  m_fd = open(devicePath.c_str(), O_RDONLY);
  if (m_fd < 0) {
    std::cerr << "Failed to open evdev device: " << devicePath << std::endl;
    return;
  }

  struct input_event ev;
  while (m_running) {
    ssize_t n = read(m_fd, &ev, sizeof(ev));
    if (n == (ssize_t)sizeof(ev)) {
      if (ev.type == EV_REL) {
        double t = (double)ev.time.tv_sec + (double)ev.time.tv_usec / 1000000.0;
        double dx = 0, dy = 0;
        if (ev.code == REL_X)
          dx = ev.value;
        if (ev.code == REL_Y)
          dy = ev.value;

        if (dx != 0 || dy != 0) {
          m_callback(t, dx, dy);
        }
      }
    } else if (n < 0 && errno != EINTR) {
      break;
    }
  }
  close(m_fd);
  m_fd = -1;
}

bool LinuxEvdevReader::start(const std::string &devicePath) {
  if (m_running)
    return false;
  m_running = true;
  std::thread([this, devicePath]() { readLoop(devicePath); }).detach();
  return true;
}

void LinuxEvdevReader::stop() { m_running = false; }

} // namespace mouseofono::platforms::linux
#endif
