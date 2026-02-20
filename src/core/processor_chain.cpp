#include "processor_chain.h"

namespace mouseofono::core {

AGC::AGC(int fs, float target, float maxGain, float attackSec, float releaseSec)
    : m_target(target), m_maxGain(maxGain) {
  m_atk = 1.0f - std::exp(-1.0f / (fs * attackSec));
  m_rel = 1.0f - std::exp(-1.0f / (fs * releaseSec));
}

float AGC::process(float peak) {
  float a = std::abs(peak);
  m_env += (a > m_env ? m_atk : m_rel) * (a - m_env);
  m_env = std::max(m_env, 1e-6f);
  m_gain = std::min(m_target / m_env, m_maxGain);
  return m_gain;
}

} // namespace mouseofono::core
