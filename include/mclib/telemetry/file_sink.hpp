// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @brief A telemetry Sink over a `std::FILE*` somebody else owns.
 *
 * @details The one that matters on the robot is stdout: on the V5, stdout is
 * the USB serial link, so a logger pointed at `stdoutSink()` streams its CSV
 * straight into the PROS terminal (`pros terminal`) with no SD card involved.
 * It also works over any other open stream, which is how the host test drives
 * it with `std::tmpfile()`.
 */

#include "mclib/telemetry/sink.hpp"

#include <cstddef>
#include <cstdio>

namespace mclib {
namespace telemetry {

/**
 * @brief Appends bytes to a stream this sink does not own.
 *
 * @details The caller keeps the `FILE*` open and closes it, if it ever does.
 * `close()` here only stops further writes; it never calls fclose, because
 * closing stdout would take the serial link away from everything else.
 */
class FileSink : public Sink {
 public:
  /**
   * @param file An open stream, or nullptr for a sink that is never
   *   available. Defaults to stdout - the USB serial link on the V5.
   */
  explicit FileSink(std::FILE* file = stdout) : m_file(file) {}

  bool isAvailable() override;
  bool write(const char* data, std::size_t length) override;
  void flush() override;
  void close() override;

  /** @brief Bytes handed to fwrite and accepted, so far. */
  std::size_t bytesWritten() const { return m_bytes_written; }

  /** @brief Whether a short write killed this sink. */
  bool writeFailed() const { return m_write_failed; }

 private:
  std::FILE* m_file;
  bool m_write_failed = false;
  std::size_t m_bytes_written = 0;
};

/**
 * @brief The one FileSink over stdout, built on first use.
 *
 * @details A function-local static so nothing has to own it and it is never
 * constructed before stdio is ready.
 */
inline FileSink& stdoutSink() {
  static FileSink sink(stdout);
  return sink;
}

}  // namespace telemetry
}  // namespace mclib
