#include "wav_writer.h"
#include <iostream>

namespace mouseofono::core {

WavWriter::WavWriter()
    : m_sampleRate(0), m_channels(0), m_dataSize(0), m_isOpen(false) {}

WavWriter::~WavWriter() {
  if (m_isOpen) {
    close();
  }
}

bool WavWriter::open(const std::string &path, int sampleRate, int channels) {
  m_path = path;
  m_sampleRate = sampleRate;
  m_channels = channels;
  m_dataSize = 0;

  m_file.open(path, std::ios::binary);
  if (!m_file.is_open())
    return false;

  writeHeader();
  m_isOpen = true;
  return true;
}

void WavWriter::writeHeader() {
  auto w2 = [&](uint16_t v) { m_file.write((char *)&v, 2); };
  auto w4 = [&](uint32_t v) { m_file.write((char *)&v, 4); };

  m_file.write("RIFF", 4);
  w4(0); // placeholder for file size
  m_file.write("WAVE", 4);

  m_file.write("fmt ", 4);
  w4(16);
  w2(3); // IEEE float
  w2((uint16_t)m_channels);
  w4((uint32_t)m_sampleRate);
  w4((uint32_t)(m_sampleRate * m_channels * sizeof(float)));
  w2((uint16_t)(m_channels * sizeof(float)));
  w2(32); // bits per sample

  m_file.write("data", 4);
  w4(0); // placeholder for data size
}

bool WavWriter::write(const std::vector<float> &samples) {
  if (!m_isOpen)
    return false;
  m_file.write((char *)samples.data(), samples.size() * sizeof(float));
  m_dataSize += (uint32_t)(samples.size() * sizeof(float));
  return true;
}

void WavWriter::close() {
  if (!m_isOpen)
    return;
  finalize();
  m_file.close();
  m_isOpen = false;
}

void WavWriter::finalize() {
  m_file.seekp(4);
  uint32_t fileSize = 36 + m_dataSize;
  m_file.write((char *)&fileSize, 4);

  m_file.seekp(40);
  m_file.write((char *)&m_dataSize, 4);
}

} // namespace mouseofono::core
