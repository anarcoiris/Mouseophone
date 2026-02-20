#ifndef MOUSEOFONO_CORE_WAV_WRITER_H
#define MOUSEOFONO_CORE_WAV_WRITER_H

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>


namespace mouseofono::core {

/**
 * @brief Simple WAV writer supporting IEEE float32.
 */
class WavWriter {
public:
  WavWriter();
  ~WavWriter();

  bool open(const std::string &path, int sampleRate, int channels);
  bool write(const std::vector<float> &samples);
  void close();

private:
  void writeHeader();
  void finalize();

  std::ofstream m_file;
  std::string m_path;
  int m_sampleRate;
  int m_channels;
  uint32_t m_dataSize;
  bool m_isOpen;
};

} // namespace mouseofono::core

#endif // MOUSEOFONO_CORE_WAV_WRITER_H
