#include "signal_reconstructor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mouseofono::core {

SignalReconstructor::SignalReconstructor(double bandwidth, double halfWindow,
                                         int sampleRate)
    : m_bandwidth(bandwidth), m_halfWindow(halfWindow),
      m_sampleRate(sampleRate) {}

double SignalReconstructor::sinc_norm(double x) const {
  if (std::abs(x) < 1e-12)
    return 1.0;
  double px = M_PI * x;
  return std::sin(px) / px;
}

double SignalReconstructor::kernel(double dt) const {
  if (std::abs(dt) >= m_halfWindow)
    return 0.0;
  double hann = 0.5 * (1.0 + std::cos(M_PI * dt / m_halfWindow));
  return sinc_norm(2.0 * m_bandwidth * dt) * hann;
}

int SignalReconstructor::reconstruct(double ts, const MouseEvent *events,
                                     int count, double &outX,
                                     double &outY) const {
  outX = 0;
  outY = 0;
  int hits = 0;

  for (int i = 0; i < count; ++i) {
    double dt = ts - events[i].t;
    if (std::abs(dt) <= m_halfWindow) {
      double h = kernel(dt);
      outX += events[i].dx * h;
      outY += events[i].dy * h;
      hits++;
    }
  }
  return hits;
}

} // namespace mouseofono::core
