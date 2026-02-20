#ifndef MOUSEOFONO_CORE_SIGNAL_RECONSTRUCTOR_H
#define MOUSEOFONO_CORE_SIGNAL_RECONSTRUCTOR_H

#include <algorithm>
#include <cmath>
#include <vector>


namespace mouseofono::core {

struct MouseEvent {
  double t;  // Timestamp in seconds
  double dx; // Delta X
  double dy; // Delta Y
};

/**
 * @brief Band-limited signal reconstruction using a Sinc kernel with Hann
 * window.
 */
class SignalReconstructor {
public:
  SignalReconstructor(double bandwidth, double halfWindow, int sampleRate);

  /**
   * @brief Reconstructs the signal at a specific time using a window of events.
   * @param ts Target timestamp (seconds)
   * @param events Buffer of events surrounding the timestamp
   * @param count Number of events in buffer
   * @param outX Reconstructed X amplitude
   * @param outY Reconstructed Y amplitude
   * @return Number of events that contributed to this sample (hits)
   */
  int reconstruct(double ts, const MouseEvent *events, int count, double &outX,
                  double &outY) const;

  double getHalfWindow() const { return m_halfWindow; }

private:
  double sinc_norm(double x) const;
  double kernel(double dt) const;

  double m_bandwidth;
  double m_halfWindow;
  int m_sampleRate;
};

} // namespace mouseofono::core

#endif // MOUSEOFONO_CORE_SIGNAL_RECONSTRUCTOR_H
