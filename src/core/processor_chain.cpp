#define _USE_MATH_DEFINES
#include "processor_chain.h"
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace mouseofono::core {

// ─── AGC ────────────────────────────────────────────────────────
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

// ─── WienerFilter ───────────────────────────────────────────────
WienerFilter::WienerFilter(int fft_n, float alpha, float beta)
    : m_fft_n(fft_n), m_alpha(alpha), m_beta(beta) {
  m_hop = fft_n / 2;
  m_noise_psd.assign(fft_n / 2 + 1, 0.0f);
  m_in_buf.assign(fft_n, 0.0f);
  m_out_buf.assign(fft_n + m_hop, 0.0f); // extra hop for overlap
}

// Radix-2 Cooley-Tukey FFT (in-place, complex)
void WienerFilter::fft(std::vector<std::complex<float>> &x, bool inverse) {
  int N = (int)x.size();
  // Bit-reversal permutation
  for (int i = 1, j = 0; i < N; ++i) {
    int bit = N >> 1;
    for (; j & bit; bit >>= 1)
      j ^= bit;
    j ^= bit;
    if (i < j)
      std::swap(x[i], x[j]);
  }
  // Iterative Cooley-Tukey
  for (int len = 2; len <= N; len <<= 1) {
    float ang = 2.0f * (float)M_PI / (float)len * (inverse ? -1.0f : 1.0f);
    std::complex<float> wlen(std::cos(ang), std::sin(ang));
    for (int i = 0; i < N; i += len) {
      std::complex<float> w(1.0f, 0.0f);
      for (int j = 0; j < len / 2; ++j) {
        std::complex<float> u = x[i + j], v = x[i + j + len / 2] * w;
        x[i + j] = u + v;
        x[i + j + len / 2] = u - v;
        w *= wlen;
      }
    }
  }
  if (inverse) {
    for (auto &c : x)
      c /= (float)N;
  }
}

void WienerFilter::apply_hann(std::vector<float> &buf) {
  int N = (int)buf.size();
  for (int i = 0; i < N; ++i) {
    double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (N - 1)));
    buf[i] *= (float)w;
  }
}

// Feed raw silence samples for calibration
void WienerFilter::feed_calibration(const float *samples, int n) {
  std::vector<std::complex<float>> frame(m_fft_n, {0, 0});
  int off = 0;
  while (off + m_fft_n <= n) {
    for (int i = 0; i < m_fft_n; ++i)
      frame[i] = {samples[off + i], 0.0f};

    // Apply Hann window
    for (int i = 0; i < m_fft_n; ++i) {
      float w =
          0.5f * (1.0f - std::cos(2.0f * (float)M_PI * i / (m_fft_n - 1)));
      frame[i] *= w;
    }

    fft(frame, false);

    // Accumulate |X|^2 for each positive-frequency bin
    for (int k = 0; k <= m_fft_n / 2; ++k) {
      float mag2 =
          frame[k].real() * frame[k].real() + frame[k].imag() * frame[k].imag();
      m_noise_psd[k] += mag2;
    }
    ++m_calib_frames;
    off += m_fft_n; // non-overlapping for calibration
  }
}

// Average PSD over all calibration frames
void WienerFilter::finalize_calibration() {
  if (m_calib_frames > 0) {
    for (auto &v : m_noise_psd)
      v /= (float)m_calib_frames;
  }
  m_calibrated = true;
}

bool WienerFilter::save_noise_profile(const std::string &path) const {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    return false;
  int n = (int)m_noise_psd.size();
  f.write(reinterpret_cast<const char *>(&n), sizeof(n));
  f.write(reinterpret_cast<const char *>(m_noise_psd.data()),
          n * sizeof(float));
  return true;
}

bool WienerFilter::load_noise_profile(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    return false;
  int n = 0;
  f.read(reinterpret_cast<char *>(&n), sizeof(n));
  if (n != (int)m_noise_psd.size())
    return false;
  f.read(reinterpret_cast<char *>(m_noise_psd.data()), n * sizeof(float));
  m_calibrated = true;
  return true;
}

// Streaming spectral subtraction (overlap-add, 50% hop)
void WienerFilter::process_inplace(float *samples, int n) {
  if (!m_calibrated)
    return; // no-op until calibrated

  for (int s = 0; s < n; ++s) {
    // Fill input ring buffer (shift + new sample)
    // Simple approach: accumulate whole block then process
    (void)samples; // handled below
    (void)s;
    break;
  }

  // Block-based overlap-add:
  int hop = m_hop;
  int off = 0;
  while (off + m_fft_n <= n) {
    // Build complex frame from input
    std::vector<std::complex<float>> frame(m_fft_n);
    std::vector<float> win(m_fft_n);
    for (int i = 0; i < m_fft_n; ++i) {
      win[i] = 0.5f * (1.0f - std::cos(2.0f * (float)M_PI * i / (m_fft_n - 1)));
      frame[i] = {samples[off + i] * win[i], 0.0f};
    }

    fft(frame, false);

    // Spectral subtraction: H(k) = max(|X(k)|² - α*N(k), β*N(k)) / |X(k)|²
    for (int k = 0; k <= m_fft_n / 2; ++k) {
      float mag2 =
          frame[k].real() * frame[k].real() + frame[k].imag() * frame[k].imag();
      float noise = m_alpha * m_noise_psd[k];
      float floor = m_beta * m_noise_psd[k];
      float signal = std::max(mag2 - noise, floor);
      float gain = (mag2 > 1e-12f) ? std::sqrt(signal / (mag2 + 1e-12f)) : 0.0f;
      frame[k] *= gain;
      // Mirror negative frequencies (real signal symmetry)
      if (k > 0 && k < m_fft_n / 2) {
        frame[m_fft_n - k] = std::conj(frame[k]);
      }
    }

    fft(frame, true); // IFFT

    // Overlap-add with synthesis Hann
    for (int i = 0; i < m_fft_n; ++i) {
      float w =
          0.5f * (1.0f - std::cos(2.0f * (float)M_PI * i / (m_fft_n - 1)));
      m_out_buf[i] += frame[i].real() * w;
    }

    // Output one hop of processed samples
    for (int i = 0; i < hop && (off - hop + i) >= 0 && (off - hop + i) < n;
         ++i) {
      samples[off - hop + i] = m_out_buf[i];
    }

    // Shift output buffer
    std::memmove(m_out_buf.data(), m_out_buf.data() + hop,
                 (m_out_buf.size() - hop) * sizeof(float));
    std::fill(m_out_buf.end() - hop, m_out_buf.end(), 0.0f);

    off += hop;
  }
}

} // namespace mouseofono::core
