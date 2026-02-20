#ifndef MOUSEOFONO_CORE_PROCESSOR_CHAIN_H
#define MOUSEOFONO_CORE_PROCESSOR_CHAIN_H

#include <algorithm>
#include <cmath>


namespace mouseofono::core {

/**
 * @brief DC Blocking filter to remove offset.
 */
class DCBlock {
public:
  DCBlock(double pole = 0.995) : m_pole(pole) {}

  double process(double x) {
    double y = x - xp + m_pole * yp;
    xp = x;
    yp = y;
    return y;
  }

private:
  double xp = 0, yp = 0;
  double m_pole;
};

/**
 * @brief Automatic Gain Control.
 */
class AGC {
public:
  AGC(int fs, float target = 0.40f, float maxGain = 300.0f,
      float attackSec = 0.005f, float releaseSec = 0.400f);

  float process(float peak);
  float getGain() const { return m_gain; }

private:
  float m_env = 1e-3f;
  float m_gain = 1.0f;
  float m_target;
  float m_maxGain;
  float m_atk;
  float m_rel;
};

/**
 * @brief Tanh soft-clipping.
 */
inline float soft_clip(float x) { return std::tanh(x); }

} // namespace mouseofono::core

#endif // MOUSEOFONO_CORE_PROCESSOR_CHAIN_H
