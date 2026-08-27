// mclib
#include "mclib/telemetry/sd_sink.hpp"

#include <cstddef>
#include <cstdio>

namespace mclib {
namespace telemetry {
namespace {

/** @brief Whether a path already exists, using the only test stdio offers. */
bool fileExists(const char* path) {
  std::FILE* existing = std::fopen(path, "r");
  if (existing == nullptr) {
    return false;
  }
  std::fclose(existing);
  return true;
}

}  // namespace

SdCardSink::SdCardSink(const char* base_name, const SdSinkConfig& config)
    : m_base_name(base_name != nullptr ? base_name : "log"), m_config(config) {}

SdCardSink::~SdCardSink() { SdCardSink::close(); }

bool SdCardSink::openFile() {
  // Pick the first index whose file does not exist yet, so a re-run never
  // silently overwrites the previous match's log.
  for (int index = 0; index <= m_config.max_index; ++index) {
    const int length = std::snprintf(m_path, sizeof(m_path), "%s%s%03d.csv", kSdMountPoint,
                                     m_base_name, index);
    if (length <= 0 || static_cast<std::size_t>(length) >= sizeof(m_path)) {
      m_path[0] = '\0';
      return false;
    }
    if (fileExists(m_path)) {
      continue;
    }
    m_file = std::fopen(m_path, "w");
    if (m_file != nullptr) {
      return true;
    }
    // The very first failed open means there is no card (or it is read-only).
    // Trying 999 more names would just be 999 more failures.
    m_path[0] = '\0';
    return false;
  }
  m_path[0] = '\0';
  return false;
}

bool SdCardSink::isAvailable() {
  if (m_probed) {
    return m_available;
  }
  m_probed = true;
  m_available = openFile();
  return m_available;
}

bool SdCardSink::write(const char* data, std::size_t length) {
  if (!isAvailable() || m_file == nullptr || m_capped) {
    return false;
  }
  if (m_config.max_bytes != 0 && m_bytes_written + length > m_config.max_bytes) {
    // Say so in the file. A log that just stops looks the same as a complete
    // one, and the whole point of the cap is that it is a known truncation.
    static const char kCapNotice[] = "# telemetry: byte cap reached, log truncated\n";
    m_capped = true;
    std::fwrite(kCapNotice, 1, sizeof(kCapNotice) - 1, m_file);
    close();
    return false;
  }
  const std::size_t written = std::fwrite(data, 1, length, m_file);
  m_bytes_written += written;
  if (written != length) {
    m_write_failed = true;
    close();
    return false;
  }
  return true;
}

void SdCardSink::flush() {
  if (m_file != nullptr && m_config.flush_each_batch) {
    std::fflush(m_file);
  }
}

void SdCardSink::close() {
  if (m_file != nullptr) {
    std::fclose(m_file);
    m_file = nullptr;
  }
  m_available = false;
  m_probed = true;
}

}  // namespace telemetry
}  // namespace mclib
