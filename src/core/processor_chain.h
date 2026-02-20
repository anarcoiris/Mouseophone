#ifndef MOUSEOFONO_CORE_PROCESSOR_CHAIN_H
#define MOUSEOFONO_CORE_PROCESSOR_CHAIN_H

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>


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

// ═══════════════════════════════════════════════════════════════
/**
 * @brief Spectral Subtraction (Wiener-style) filter.
 *
 * Usage:
 *   1. Calibrate: feed ~10s of silence into feed_calibration()
 *   2. Call finalize_calibration() to lock in the noise PSD
 *   3. Call process_frame() on each audio block
 *
 * Works on fixed-size frames (frame_n samples). Uses a simple
 * overlap-add with 50% overlap and a Hann window. FFT is a
 * hand-rolled radix-2 Cooley-Tukey for zero-dependency builds.
 *
 * Spectral floor parameter (alpha): how many times the noise PSD
 * to subtract. Typical range 1.0–2.0. Higher = more aggressive.
 * Beta: spectral floor multiplier to avoid musical noise. 0.01–0.1
 */
class WienerFilter {
public:
  explicit WienerFilter(int fft_n = 512, float alpha = 1.5f,
                        float beta = 0.02f);

  // --- Calibration ---
  void feed_calibration(const float *samples, int n);
  void finalize_calibration();

  bool save_noise_profile(const std::string &path) const;
  bool load_noise_profile(const std::string &path);
  bool is_calibrated() const { return m_calibrated; }

  // --- Processing ---
  // In-place: filters 'samples' (length = one FFT frame or less)
  // For streaming use, call repeatedly; internally manages 50% overlap.
  void process_inplace(float *samples, int n);

  int fft_n() const { return m_fft_n; }

private:
  void fft(std::vector<std::complex<float>> &x, bool inverse);
  void apply_hann(std::vector<float> &buf);

  int m_fft_n;
  float m_alpha; // over-subtraction factor
  float m_beta;  // spectral floor (fraction of noise)

  // Noise PSD estimate (mag^2 per bin)
  std::vector<float> m_noise_psd;
  long long m_calib_frames = 0;
  bool m_calibrated = false;

  // Overlap-add state
  std::vector<float> m_in_buf;  // ring input
  std::vector<float> m_out_buf; // overlap-add accumulator
  int m_hop;
};

} // namespace mouseofono::core

#endif // MOUSEOFONO_CORE_PROCESSOR_CHAIN_H
