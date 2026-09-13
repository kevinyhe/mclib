// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @brief A telemetry Sink that writes to the V5 SD card.
 *
 * @details PROS mounts the microSD card at "/usd/". If no card is inserted,
 * every open fails; this sink reports itself unavailable exactly once and is
 * a no-op from then on, so a missing card never costs a match.
 */

#include "mclib/telemetry/sink.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace mclib {
namespace telemetry {

/** @brief Where PROS mounts the microSD card. */
inline constexpr const char* kSdMountPoint = "/usd/";

/**
 * @brief Knobs for SdCardSink.
 */
struct SdSinkConfig {
  /**
   * @brief Stop writing once the file reaches this many bytes. 0 disables.
   *
   * @details A log that fills the card mid-match is worse than no log. 4 MiB
   * is roughly 40000 rows of a dozen columns.
   */
  std::size_t max_bytes = 4u * 1024u * 1024u;

  /**
   * @brief Highest index tried when picking a non-colliding filename.
   *
   * @details The sink opens "<base>000.csv", "<base>001.csv" and so on,
   * stopping at the first name that does not already exist, so today's run
   * never overwrites yesterday's.
   */
  int max_index = 999;

  /** @brief fflush() after every batch, so a brownout loses one batch only. */
  bool flush_each_batch = true;
};

/**
 * @brief Appends CSV to a uniquely named file on the SD card.
 *
 * @details The file is opened lazily on the first isAvailable() call, which
 * the logger makes once from the flush thread. Nothing here runs on the
 * control loop.
 */
class SdCardSink : public Sink {
 public:
  /**
   * @brief Build a sink over "/usd/<base_name>NNN.csv".
   *
   * @param base_name Filename stem, e.g. "auton". Kept by pointer, so pass a
   *   string literal or something that outlives the sink.
   * @param config Size cap and naming limits.
   */
  explicit SdCardSink(const char* base_name, const SdSinkConfig& config = {});

  ~SdCardSink() override;

  SdCardSink(const SdCardSink&) = delete;
  SdCardSink& operator=(const SdCardSink&) = delete;

  bool isAvailable() override;
  bool write(const char* data, std::size_t length) override;
  void flush() override;
  void close() override;

  /** @brief The path actually opened, or "" when nothing was opened. */
  const char* path() const { return m_path; }

  /** @brief Bytes written so far. */
  std::size_t bytesWritten() const { return m_bytes_written; }

  /** @brief Whether the byte cap stopped this sink. */
  bool capped() const { return m_capped; }

  /** @brief Whether a short write killed this sink - a full or failing card. */
  bool writeFailed() const { return m_write_failed; }

 private:
  bool openFile();

  const char* m_base_name;
  SdSinkConfig m_config;
  std::FILE* m_file = nullptr;
  bool m_probed = false;
  bool m_available = false;
  bool m_capped = false;
  bool m_write_failed = false;
  std::size_t m_bytes_written = 0;
  char m_path[64] = {};
};

}  // namespace telemetry
}  // namespace mclib
