// mclib
#pragma once

/**
 * @brief Byte sinks the telemetry logger writes CSV into.
 *
 * @details The logger never touches a file directly. It hands finished CSV
 * text to a Sink, so the same logger runs against the V5 SD card on the robot
 * and against an in-memory buffer in a host test.
 */

#include <cstddef>
#include <string>
#include <vector>

namespace mclib {
namespace telemetry {

/**
 * @brief Somewhere CSV bytes can go.
 *
 * @details Implementations must be safe to call with no storage behind them:
 * a sink that reports itself unavailable is never written to, and the logger
 * degrades to a no-op instead of failing.
 */
class Sink {
 public:
  virtual ~Sink() = default;

  /**
   * @brief Whether this sink can accept bytes.
   *
   * @details The logger calls this exactly once, on the first flush, and
   * caches the answer. Implementations may do real work here (opening a file,
   * probing for a card) because it happens once and off the control loop.
   *
   * @return true when writes should be attempted.
   */
  virtual bool isAvailable() = 0;

  /**
   * @brief Append bytes.
   *
   * @param data Bytes to append. Not null-terminated by contract.
   * @param length Number of bytes.
   * @return false when the sink has died and should never be written again.
   */
  virtual bool write(const char* data, std::size_t length) = 0;

  /** @brief Push buffered bytes towards storage. Cheap no-op by default. */
  virtual void flush() {}

  /** @brief Release the underlying resource. Further writes must fail. */
  virtual void close() {}
};

/**
 * @brief A sink that is never available. Every write is dropped.
 *
 * @details The stand-in for "no SD card" that a host test can use without
 * touching the filesystem.
 */
class NullSink : public Sink {
 public:
  bool isAvailable() override { return false; }
  bool write(const char*, std::size_t) override { return false; }
};

/**
 * @brief A sink that keeps everything written to it in a std::string.
 *
 * @details Host tests only - it grows without bound. Not used on the robot.
 */
class MemorySink : public Sink {
 public:
  MemorySink() = default;

  /** @param available What isAvailable() should report. */
  explicit MemorySink(bool available) : m_available(available) {}

  bool isAvailable() override {
    ++m_availability_probes;
    return m_available;
  }

  bool write(const char* data, std::size_t length) override {
    if (!m_available) {
      return false;
    }
    m_data.append(data, length);
    ++m_writes;
    return true;
  }

  void flush() override { ++m_flushes; }

  void close() override { m_closed = true; }

  /** @brief Everything written so far, in order. */
  const std::string& data() const { return m_data; }

  /** @brief data() split on newlines, with no trailing empty line. */
  std::vector<std::string> lines() const {
    std::vector<std::string> out;
    std::string current;
    for (const char c : m_data) {
      if (c == '\n') {
        out.push_back(current);
        current.clear();
      } else {
        current.push_back(c);
      }
    }
    if (!current.empty()) {
      out.push_back(current);
    }
    return out;
  }

  /** @brief How many times isAvailable() was asked. Should never exceed 1. */
  int availabilityProbes() const { return m_availability_probes; }

  /** @brief How many write() calls landed. */
  int writes() const { return m_writes; }

  /** @brief How many flush() calls landed. */
  int flushes() const { return m_flushes; }

  /** @brief Whether close() has been called. */
  bool closed() const { return m_closed; }

  /** @brief Throw away captured bytes, keeping the counters. */
  void clear() { m_data.clear(); }

 private:
  std::string m_data;
  bool m_available = true;
  bool m_closed = false;
  int m_availability_probes = 0;
  int m_writes = 0;
  int m_flushes = 0;
};

}  // namespace telemetry
}  // namespace mclib
