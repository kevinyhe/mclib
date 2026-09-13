// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/telemetry/file_sink.hpp"

#include <cstddef>
#include <cstdio>

namespace mclib {
namespace telemetry {

bool FileSink::isAvailable() { return m_file != nullptr; }

bool FileSink::write(const char* data, std::size_t length) {
  if (m_file == nullptr) {
    return false;
  }
  const std::size_t written = std::fwrite(data, 1, length, m_file);
  m_bytes_written += written;
  if (written != length) {
    // A stream that takes fewer bytes than asked is closed, full, or broken.
    // Retrying every batch would only spend the flush task on a dead link.
    m_write_failed = true;
    m_file = nullptr;
    return false;
  }
  return true;
}

void FileSink::flush() {
  if (m_file != nullptr) {
    std::fflush(m_file);
  }
}

void FileSink::close() {
  // Not ours to fclose. Push what is buffered and forget the stream so the
  // logger's "further writes must fail" contract holds.
  flush();
  m_file = nullptr;
}

}  // namespace telemetry
}  // namespace mclib
