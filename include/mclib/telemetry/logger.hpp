// mclib
#pragma once

/**
 * @brief Fixed-rate CSV telemetry with typed, self-describing columns.
 *
 * @details The point of this file is to make "what did the robot actually do"
 * answerable. A caller registers named channels once, writes values into them
 * from the control loop, and calls sample() on every tick. Sampling is gated
 * by a period, costs a memcpy, and never touches a file. A separate consumer
 * - a low-priority pros::Task on the robot, or the test itself on the host -
 * calls flush(), which turns buffered rows into CSV and hands them to a Sink.
 *
 * Everything here is header-only and PROS-free so host tests can drive it
 * under a mclib::time::ScopedClock.
 */

#include "mclib/telemetry/sink.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace mclib {
namespace telemetry {

/** @brief Hard cap on registered channels. One row is this many doubles. */
inline constexpr std::size_t kMaxChannels = 24;

/** @brief Longest channel name kept, including the unit suffix and the null. */
inline constexpr std::size_t kMaxChannelNameLength = 32;

/** @brief The id returned when a channel could not be registered. */
inline constexpr std::size_t kInvalidChannel = static_cast<std::size_t>(-1);

/**
 * @brief One buffered sample: a timestamp plus one double per channel.
 *
 * @details Fixed size and trivially copyable on purpose - committing a sample
 * from the control loop is a struct copy into a ring slot, with no allocation
 * and no lock.
 */
struct Row {
  std::uint32_t timestamp_ms = 0;
  double values[kMaxChannels] = {};
};

/**
 * @brief Storage for the ring buffer, sized at compile time.
 *
 * @details Declare one next to the logger, usually static so it lands in .bss
 * rather than on a task stack. sizeof(Row) is about 200 bytes, so a 256-row
 * buffer is roughly 50 KB - 2.5 seconds of headroom at a 10 ms control loop.
 *
 * @tparam Capacity Number of rows. One slot is always kept empty to
 *   distinguish full from empty, so the usable depth is Capacity - 1.
 */
template <std::size_t Capacity>
struct RowBuffer {
  static_assert(Capacity >= 2, "a ring buffer needs at least two slots");
  Row rows[Capacity];
  static constexpr std::size_t capacity = Capacity;
};

/**
 * @brief A typed handle to one CSV column.
 *
 * @details The template parameter is the Quantity the column accepts, so
 * `logger.set(x_channel, some_voltage)` is a compile error when x_channel is
 * a length. The handle also carries the unit the column is recorded in, which
 * is what turns a raw double back into "inches".
 *
 * @tparam Q The accepted value type: a units::Quantity, or double.
 */
template <typename Q>
class Channel {
 public:
  Channel() = default;

  Channel(std::size_t id, double unit_in_base) : m_id(id), m_unit_in_base(unit_in_base) {}

  /** @brief Column index, or kInvalidChannel when registration failed. */
  std::size_t id() const { return m_id; }

  /** @brief Whether this handle refers to a real column. */
  bool valid() const { return m_id != kInvalidChannel; }

  /** @brief Convert a value into the number this column stores. */
  double toColumn(Q value) const;

 private:
  std::size_t m_id = kInvalidChannel;
  double m_unit_in_base = 1.0;
};

template <typename Q>
inline double Channel<Q>::toColumn(Q value) const {
  return value.raw() / m_unit_in_base;
}

/** @brief A plain-double column stores its value unchanged. */
template <>
inline double Channel<double>::toColumn(double value) const {
  return value / m_unit_in_base;
}

/**
 * @brief What to do about a row that arrives with the buffer full.
 */
enum class OverflowPolicy {
  /**
   * @brief Drop the new row and keep everything already buffered.
   *
   * @details The default, and the only policy a single-producer /
   * single-consumer ring can offer without the producer racing the consumer's
   * read cursor. It means the log is a contiguous prefix of the run with a
   * gap at the end, which is what you want for tuning: the beginning of a
   * motion is the interesting part, and a contiguous prefix beats a log with
   * a hole punched in the middle. Dropped rows are counted, and the count is
   * written into the CSV trailer.
   */
  kDropNewest,
};

/**
 * @brief Knobs for the logger.
 */
struct LoggerConfig {
  /** @brief Time between samples. sample() is a no-op until it elapses. */
  units::QTime period = 20.0 * units::millisecond;

  /**
   * @brief Stop committing rows after this many. 0 means no limit.
   *
   * @details A cap so a forgotten logger cannot fill the card. 30000 rows at
   * 20 ms is 10 minutes, which outlasts any match.
   */
  std::size_t max_rows = 30000;

  /** @brief Significant digits per value. 6 round-trips a tuning log fine. */
  int precision = 6;

  /** @brief What happens when the ring is full. */
  OverflowPolicy overflow = OverflowPolicy::kDropNewest;
};

/**
 * @brief Fixed-rate CSV logger over a lock-free single-producer ring.
 *
 * @details Threading contract: one producer thread calls set()/sample(), one
 * consumer thread calls flush(). Registration must finish before either
 * starts. Anything else needs external synchronisation.
 *
 * Cost on the producer side, which is the control loop: set() is one store,
 * sample() is a clock read plus - only on a sampling tick - a Row copy and
 * one release store. No allocation, no lock, no file I/O. The worst case a
 * control tick pays is a single sizeof(Row) copy, about 200 bytes.
 *
 * Typical use:
 * @code
 * static mclib::telemetry::RowBuffer<256> buffer;
 * static mclib::telemetry::MemorySink sink;
 * static mclib::telemetry::Logger logger(sink, buffer, {.period = 20_ms});
 * auto x = logger.addLength("x");
 * auto volts = logger.addVoltage("left_cmd");
 * // control loop:
 * logger.set(x, pose.x);
 * logger.set(volts, command);
 * logger.sample();
 * @endcode
 */
class Logger {
 public:
  /**
   * @brief Build a logger over caller-owned ring storage.
   *
   * @param sink Where CSV goes. Must outlive the logger.
   * @param buffer Ring storage. Must outlive the logger.
   * @param config Period, caps and overflow policy.
   */
  template <std::size_t Capacity>
  Logger(Sink& sink, RowBuffer<Capacity>& buffer, const LoggerConfig& config = {})
      : m_sink(&sink),
        m_rows(buffer.rows),
        m_capacity(Capacity),
        m_config(config),
        m_period_ms(periodMs(config.period)) {}

  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;
  Logger(Logger&&) = delete;
  Logger& operator=(Logger&&) = delete;

  // -------------------------------------------------------------------------
  // Registration. Call before the first sample(), from one thread.
  // -------------------------------------------------------------------------

  /**
   * @brief Register a column recorded in an arbitrary unit.
   *
   * @param name Column name. The unit suffix is appended to it.
   * @param unit The unit values are divided by, e.g. units::inch.
   * @param suffix Unit label written into the header, e.g. "in".
   * @return A typed handle, invalid when the channel table is full or
   *   sampling has already started.
   */
  template <typename Q>
  Channel<Q> addChannel(const char* name, Q unit, const char* suffix) {
    return Channel<Q>(registerColumn(name, suffix), unit.raw());
  }

  /** @brief Register a unitless column, e.g. a PID error term. */
  Channel<double> addNumber(const char* name, const char* suffix = "") {
    return Channel<double>(registerColumn(name, suffix), 1.0);
  }

  /** @brief Register a length column recorded in inches, suffix "in". */
  Channel<units::QLength> addLength(const char* name) {
    return addChannel(name, units::inch, "in");
  }

  /** @brief Register an angle column recorded in degrees, suffix "deg". */
  Channel<units::QAngle> addAngle(const char* name) {
    return addChannel(name, units::degree, "deg");
  }

  /** @brief Register a voltage column recorded in volts, suffix "V". */
  Channel<units::QVoltage> addVoltage(const char* name) {
    return addChannel(name, units::volt, "V");
  }

  /** @brief Register a current column recorded in amps, suffix "A". */
  Channel<units::QCurrent> addCurrent(const char* name) {
    return addChannel(name, units::ampere, "A");
  }

  /** @brief Register a duration column recorded in milliseconds. */
  Channel<units::QTime> addTime(const char* name) {
    return addChannel(name, units::millisecond, "ms");
  }

  /** @brief Register a velocity column recorded in inches per second. */
  Channel<units::QVelocity> addVelocity(const char* name) {
    return addChannel(name, units::inps, "inps");
  }

  /** @brief Register an angular velocity column recorded in degrees/second. */
  Channel<units::QAngularVelocity> addAngularVelocity(const char* name) {
    return addChannel(name, units::degps, "degps");
  }

  /** @brief How many columns are registered, not counting the timestamp. */
  std::size_t channelCount() const { return m_channel_count; }

  /** @brief The header line this logger will emit, without the newline. */
  const char* headerName(std::size_t index) const {
    return index < m_channel_count ? m_names[index] : "";
  }

  // -------------------------------------------------------------------------
  // Producer side. Control-loop thread only.
  // -------------------------------------------------------------------------

  /**
   * @brief Stage a value for the next sample.
   *
   * @details Overwrites whatever was staged before, so a channel written on
   * every tick and sampled every fifth tick records the most recent value.
   * A channel keeps its last value until it is written again.
   *
   * @param channel Handle from an add*() call. An invalid handle is ignored.
   * @param value The value, in whatever unit the handle's type demands.
   */
  template <typename Q>
  void set(const Channel<Q>& channel, Q value) {
    if (channel.id() < m_channel_count) {
      m_pending.values[channel.id()] = channel.toColumn(value);
    }
  }

  /**
   * @brief Whether the sampling period has elapsed.
   *
   * @return true when the next sample() call will commit a row.
   */
  bool due() const {
    if (m_stopped || m_row_count >= rowLimit()) {
      return false;
    }
    if (!m_started) {
      return true;
    }
    return static_cast<std::int32_t>(time::millis() - m_next_due_ms) >= 0;
  }

  /**
   * @brief Commit the staged values as a row, if the period has elapsed.
   *
   * @details Call this every control tick. It is cheap on the ticks that do
   * not sample: one clock read and a comparison. This is the only call that
   * moves the periodic schedule forward.
   *
   * @return true when a row was committed.
   */
  bool sample() {
    if (!due()) {
      return false;
    }
    const std::uint32_t now = time::millis();
    startClock(now);
    // Advance by whole periods so sampling does not drift, and clamp forward
    // when the caller fell behind so a late tick does not fire a burst. Done
    // even when the ring is full, so a dropped sample does not turn into a
    // burst once space frees up.
    m_next_due_ms += m_period_ms;
    if (static_cast<std::int32_t>(m_next_due_ms - now) <= 0) {
      m_next_due_ms = now + m_period_ms;
    }
    return commitRow(now);
  }

  /**
   * @brief Commit the staged values as a row right now, period ignored.
   *
   * @details For event rows - a motion starting, a settle firing - that should
   * land whether or not the period has elapsed. Still respects max_rows, the
   * stop flag and the overflow policy.
   *
   * Scheduling is independent of sample(): an event row never moves the
   * periodic deadline, so committing events faster than the period cannot
   * starve sampling. The first row of the run starts the periodic clock at
   * the current time, which leaves the next sample() due immediately - the
   * same thing that happens when sample() itself commits the first row.
   *
   * @return true when a row was committed.
   */
  bool commit() {
    if (m_stopped || m_row_count >= rowLimit()) {
      return false;
    }
    const std::uint32_t now = time::millis();
    startClock(now);
    return commitRow(now);
  }

  /**
   * @brief Stop committing rows. flush() still drains what is buffered.
   */
  void stop() { m_stopped = true; }

  /** @brief Whether stop() has been called or the row cap was reached. */
  bool stopped() const { return m_stopped || m_row_count >= rowLimit(); }

  /** @brief Rows lost to a full buffer. */
  std::size_t droppedRows() const { return m_dropped_rows; }

  /**
   * @brief Rows thrown away on the sink side.
   *
   * @details Rows that were committed but never reached storage, because the
   * sink was unavailable or died mid-batch. Discarding them is what keeps a
   * dead sink from wedging the producer behind a permanently full ring.
   */
  std::size_t discardedRows() const { return m_discarded_rows; }

  /** @brief Rows committed to the buffer over the logger's life. */
  std::size_t rowCount() const { return m_row_count; }

  /** @brief Rows sitting in the buffer waiting to be flushed. */
  std::size_t pending() const {
    const std::size_t head = m_head.load(std::memory_order_acquire);
    const std::size_t tail = m_tail.load(std::memory_order_relaxed);
    return (head + m_capacity - tail) % m_capacity;
  }

  // -------------------------------------------------------------------------
  // Consumer side. Flush thread only.
  // -------------------------------------------------------------------------

  /**
   * @brief Turn buffered rows into CSV and hand them to the sink.
   *
   * @details Probes the sink for availability on the first call and caches the
   * answer, so a missing SD card costs one probe and nothing after that. When
   * the sink is unavailable or has died, buffered rows are discarded rather
   * than left to block the producer, and this returns 0.
   *
   * @return Number of rows written.
   */
  std::size_t flush() {
    const std::size_t head = m_head.load(std::memory_order_acquire);
    std::size_t tail = m_tail.load(std::memory_order_relaxed);

    if (head == tail) {
      return 0;
    }
    // Probed here rather than above, so a logger that never records a row also
    // never opens a file. An empty run should not burn an SD filename.
    if (!sinkUsable()) {
      discardTo(head, tail);
      return 0;
    }
    if (!m_header_written) {
      if (!writeHeader()) {
        discardTo(head, tail);
        return 0;
      }
      m_header_written = true;
    }

    std::size_t written = 0;
    char line[32 + kMaxChannels * 32];
    while (tail != head) {
      const Row& row = m_rows[tail];
      std::size_t offset = 0;
      bool fits = append(line, sizeof(line), offset, "%lu",
                         static_cast<unsigned long>(row.timestamp_ms));
      for (std::size_t i = 0; fits && i < m_channel_count; ++i) {
        fits = append(line, sizeof(line), offset, ",%.*g", m_config.precision, row.values[i]);
      }
      if (fits) {
        fits = appendChar(line, sizeof(line), offset, '\n');
      }
      if (!fits) {
        // A precision high enough to overrun the line buffer truncates the row
        // rather than running off the end of it. Still newline-terminated, so
        // the CSV stays parseable.
        offset = sizeof(line) - 1;
        line[offset - 1] = '\n';
      }
      if (!m_sink->write(line, offset)) {
        m_sink_state = SinkState::kDead;
        discardTo(head, tail);
        return written;
      }
      tail = (tail + 1) % m_capacity;
      ++written;
    }
    m_tail.store(tail, std::memory_order_release);
    m_sink->flush();
    return written;
  }

  /**
   * @brief Drain the buffer, write a trailer, and close the sink.
   *
   * @details The trailer is a comment line recording how many rows were
   * dropped, so a truncated log says so instead of looking complete. Safe to
   * call twice.
   */
  void close() {
    if (m_closed) {
      return;
    }
    m_stopped = true;
    flush();
    // Attempted whenever a header was written, including after the sink died,
    // so a log truncated by a card failure carries the loss counts instead of
    // just stopping. Writing to a dead sink returns false and is harmless.
    if (m_header_written) {
      char trailer[128];
      const int length = std::snprintf(
          trailer, sizeof(trailer), "# rows=%lu dropped=%lu discarded=%lu\n",
          static_cast<unsigned long>(m_row_count),
          static_cast<unsigned long>(m_dropped_rows),
          static_cast<unsigned long>(m_discarded_rows));
      if (length > 0) {
        m_sink->write(trailer, static_cast<std::size_t>(length));
      }
    }
    m_sink->flush();
    m_sink->close();
    m_closed = true;
  }

  /** @brief Whether the sink accepted the availability probe. */
  bool sinkAlive() const { return m_sink_state == SinkState::kAlive; }

  /** @brief The configuration this logger was built with. */
  const LoggerConfig& config() const { return m_config; }

 private:
  enum class SinkState { kUnprobed, kAlive, kDead };

  /**
   * @brief Start the periodic clock on the first row of the run.
   *
   * @details Also closes registration. Until the first row lands there is no
   * meaningful zero for the period to count from, so the clock starts here
   * rather than at construction.
   */
  void startClock(std::uint32_t now) {
    if (!m_started) {
      m_started = true;
      m_next_due_ms = now;
    }
  }

  /**
   * @brief Copy the staged values into the ring. No scheduling, no gating.
   *
   * @return true when the row landed, false when the ring was full.
   */
  bool commitRow(std::uint32_t now) {
    const std::size_t head = m_head.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % m_capacity;
    if (next == m_tail.load(std::memory_order_acquire)) {
      // Full. kDropNewest: keep the buffered prefix, count the loss.
      ++m_dropped_rows;
      return false;
    }
    m_pending.timestamp_ms = now;
    m_rows[head] = m_pending;
    m_head.store(next, std::memory_order_release);
    ++m_row_count;
    return true;
  }

  /**
   * @brief snprintf that tracks an offset and refuses to run off the end.
   *
   * @return false when the text did not fit, leaving @p offset unchanged.
   */
  template <typename... Args>
  static bool append(char* buffer, std::size_t size, std::size_t& offset, const char* format,
                     Args... args) {
    if (offset + 1 >= size) {
      return false;
    }
    const int written = std::snprintf(buffer + offset, size - offset, format, args...);
    if (written < 0 || static_cast<std::size_t>(written) >= size - offset) {
      return false;
    }
    offset += static_cast<std::size_t>(written);
    return true;
  }

  /** @brief Append one character, refusing to run off the end. */
  static bool appendChar(char* buffer, std::size_t size, std::size_t& offset, char c) {
    if (offset + 1 >= size) {
      return false;
    }
    buffer[offset++] = c;
    buffer[offset] = '\0';
    return true;
  }

  /**
   * @brief Throw away buffered rows so the producer never wedges behind a
   * sink that cannot take them, counting the loss.
   */
  void discardTo(std::size_t head, std::size_t tail) {
    m_discarded_rows += (head + m_capacity - tail) % m_capacity;
    m_tail.store(head, std::memory_order_release);
  }

  static std::uint32_t periodMs(units::QTime period) {
    const double ms = period.ms();
    if (!(ms >= 1.0)) {
      return 1;  // Also catches NaN. A zero period would sample every tick.
    }
    return static_cast<std::uint32_t>(ms);
  }

  std::size_t rowLimit() const {
    return m_config.max_rows == 0 ? static_cast<std::size_t>(-1) : m_config.max_rows;
  }

  std::size_t registerColumn(const char* name, const char* suffix) {
    if (m_started || m_channel_count >= kMaxChannels || name == nullptr) {
      return kInvalidChannel;
    }
    const std::size_t id = m_channel_count;
    char* dest = m_names[id];
    std::size_t out = 0;
    for (const char* p = name; *p != '\0' && out + 1 < kMaxChannelNameLength; ++p) {
      dest[out++] = (*p == ',' || *p == '\n' || *p == '\r') ? '_' : *p;
    }
    if (suffix != nullptr && suffix[0] != '\0' && out + 1 < kMaxChannelNameLength) {
      dest[out++] = '_';
      for (const char* p = suffix; *p != '\0' && out + 1 < kMaxChannelNameLength; ++p) {
        dest[out++] = (*p == ',' || *p == '\n' || *p == '\r') ? '_' : *p;
      }
    }
    dest[out] = '\0';
    ++m_channel_count;
    return id;
  }

  bool sinkUsable() {
    if (m_sink_state == SinkState::kUnprobed) {
      m_sink_state = m_sink->isAvailable() ? SinkState::kAlive : SinkState::kDead;
    }
    return m_sink_state == SinkState::kAlive;
  }

  bool writeHeader() {
    char line[16 + kMaxChannels * (kMaxChannelNameLength + 1)];
    std::size_t offset = 0;
    bool fits = append(line, sizeof(line), offset, "%s", "t_ms");
    for (std::size_t i = 0; fits && i < m_channel_count; ++i) {
      fits = append(line, sizeof(line), offset, ",%s", m_names[i]);
    }
    if (fits) {
      fits = appendChar(line, sizeof(line), offset, '\n');
    }
    if (!fits) {
      offset = sizeof(line) - 1;
      line[offset - 1] = '\n';
    }
    if (!m_sink->write(line, offset)) {
      m_sink_state = SinkState::kDead;
      return false;
    }
    return true;
  }

  Sink* m_sink;
  Row* m_rows;
  std::size_t m_capacity;
  LoggerConfig m_config;
  std::uint32_t m_period_ms;

  char m_names[kMaxChannels][kMaxChannelNameLength] = {};
  std::size_t m_channel_count = 0;

  Row m_pending;
  bool m_started = false;
  bool m_stopped = false;
  bool m_closed = false;
  bool m_header_written = false;
  std::uint32_t m_next_due_ms = 0;
  std::size_t m_row_count = 0;
  std::size_t m_dropped_rows = 0;
  std::size_t m_discarded_rows = 0;

  SinkState m_sink_state = SinkState::kUnprobed;

  std::atomic<std::size_t> m_head{0};
  std::atomic<std::size_t> m_tail{0};
};

}  // namespace telemetry
}  // namespace mclib
