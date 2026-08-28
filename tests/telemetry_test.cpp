// mclib
/**
 * @brief Host tests for the telemetry logger, driven by a fake clock and an
 * in-memory sink. No SD card, no PROS, no filesystem.
 */

#include "mclib/telemetry/logger.hpp"
#include "mclib/telemetry/sink.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// The host definition of mclib::time::systemMillis() lives in
// tests/support/host_time.cpp, which HOST_TEST_SRC links into every test
// binary. Defining it here too is a duplicate symbol at link time.

namespace {

using mclib::telemetry::Logger;
using mclib::telemetry::LoggerConfig;
using mclib::telemetry::MemorySink;
using mclib::telemetry::NullSink;
using mclib::telemetry::RowBuffer;

constexpr std::size_t kNameBufferSize = 40;

std::uint32_t g_fake_ms = 0;
std::uint32_t fakeClock() { return g_fake_ms; }

/** @brief Split one CSV line on commas. */
std::vector<std::string> fields(const std::string& line) {
  std::vector<std::string> out;
  std::string current;
  for (const char c : line) {
    if (c == ',') {
      out.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  out.push_back(current);
  return out;
}

/** @brief The header names every channel with its unit suffix. */
void testHeaderNamesUnits() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<32> buffer;
  Logger logger(sink, buffer, LoggerConfig{.period = 50.0 * mclib::units::millisecond});

  auto x = logger.addLength("x");
  auto heading = logger.addAngle("heading");
  auto cmd = logger.addVoltage("left_cmd");
  auto err = logger.addNumber("error");

  CHECK(x.valid());
  CHECK_EQ(static_cast<double>(logger.channelCount()), 4.0);

  logger.set(x, 24.0 * mclib::units::inch);
  logger.set(heading, 90.0 * mclib::units::degree);
  logger.set(cmd, 12.0 * mclib::units::volt);
  logger.set(err, 0.5);
  CHECK(logger.sample());
  CHECK_EQ(static_cast<double>(logger.flush()), 1.0);

  const std::vector<std::string> lines = sink.lines();
  CHECK_EQ(static_cast<double>(lines.size()), 2.0);
  if (lines.size() >= 1) {
    CHECK(lines[0] == "t_ms,x_in,heading_deg,left_cmd_V,error");
    if (lines[0] != "t_ms,x_in,heading_deg,left_cmd_V,error") {
      std::printf("       header was: %s\n", lines[0].c_str());
    }
  }
  // Values round-trip: 24_in comes back as 24.0 in the column called x_in.
  if (lines.size() >= 2) {
    const std::vector<std::string> row = fields(lines[1]);
    CHECK_EQ(static_cast<double>(row.size()), 5.0);
    if (row.size() == 5) {
      CHECK_EQ(std::atof(row[0].c_str()), 0.0);
      CHECK_NEAR(std::atof(row[1].c_str()), 24.0, 1e-9);
      CHECK_NEAR(std::atof(row[2].c_str()), 90.0, 1e-9);
      CHECK_NEAR(std::atof(row[3].c_str()), 12.0, 1e-9);
      CHECK_NEAR(std::atof(row[4].c_str()), 0.5, 1e-12);
    }
  }

  // Availability is probed once, not per write.
  logger.set(x, 12.0 * mclib::units::inch);
  g_fake_ms = 50;
  CHECK(logger.sample());
  CHECK_EQ(static_cast<double>(logger.flush()), 1.0);
  CHECK_EQ(static_cast<double>(sink.availabilityProbes()), 1.0);
}

/** @brief Fixed-rate sampling under a fake clock: exact count and stamps. */
void testFixedRateSampling() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<64> buffer;
  Logger logger(sink, buffer, LoggerConfig{.period = 50.0 * mclib::units::millisecond});
  auto x = logger.addLength("x");

  // A 10 ms control loop for 500 ms: 51 ticks at t = 0..500.
  int sampled = 0;
  for (int tick = 0; tick <= 50; ++tick) {
    g_fake_ms = static_cast<std::uint32_t>(tick * 10);
    logger.set(x, static_cast<double>(tick) * mclib::units::inch);
    if (logger.sample()) {
      ++sampled;
    }
  }
  // 50 ms period over 0..500 ms inclusive: 0, 50, ... 500 = 11 rows.
  CHECK_EQ(static_cast<double>(sampled), 11.0);
  CHECK_EQ(static_cast<double>(logger.rowCount()), 11.0);
  CHECK_EQ(static_cast<double>(logger.pending()), 11.0);
  CHECK_EQ(static_cast<double>(logger.droppedRows()), 0.0);

  CHECK_EQ(static_cast<double>(logger.flush()), 11.0);
  const std::vector<std::string> lines = sink.lines();
  CHECK_EQ(static_cast<double>(lines.size()), 12.0);  // header + 11 rows
  for (std::size_t i = 1; i < lines.size(); ++i) {
    const std::vector<std::string> row = fields(lines[i]);
    CHECK_EQ(std::atof(row[0].c_str()), static_cast<double>((i - 1) * 50));
    CHECK_NEAR(std::atof(row[1].c_str()), static_cast<double>((i - 1) * 5), 1e-9);
  }

  std::printf("  sample of actual CSV output:\n");
  for (std::size_t i = 0; i < lines.size() && i < 5; ++i) {
    std::printf("    %s\n", lines[i].c_str());
  }
}

/** @brief A full buffer drops the newest row and keeps the buffered prefix. */
void testOverflowDropsNewest() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<4> buffer;  // one slot reserved, so 3 rows fit
  Logger logger(sink, buffer, LoggerConfig{.period = 10.0 * mclib::units::millisecond});
  auto n = logger.addNumber("n");

  for (int i = 0; i < 10; ++i) {
    g_fake_ms = static_cast<std::uint32_t>(i * 10);
    logger.set(n, static_cast<double>(i));
    logger.sample();
  }
  CHECK_EQ(static_cast<double>(logger.rowCount()), 3.0);
  CHECK_EQ(static_cast<double>(logger.droppedRows()), 7.0);
  CHECK_EQ(static_cast<double>(logger.pending()), 3.0);

  CHECK_EQ(static_cast<double>(logger.flush()), 3.0);
  const std::vector<std::string> lines = sink.lines();
  CHECK_EQ(static_cast<double>(lines.size()), 4.0);
  // The prefix survived: rows 0, 1, 2 - not the newest three.
  for (std::size_t i = 1; i < lines.size(); ++i) {
    const std::vector<std::string> row = fields(lines[i]);
    CHECK_NEAR(std::atof(row[1].c_str()), static_cast<double>(i - 1), 1e-12);
  }

  // Draining frees slots and sampling resumes.
  g_fake_ms = 200;
  logger.set(n, 99.0);
  CHECK(logger.sample());

  logger.close();
  const std::vector<std::string> closed = sink.lines();
  CHECK(closed.back() == "# rows=4 dropped=7 discarded=0");
  if (closed.back() != "# rows=4 dropped=7 discarded=0") {
    std::printf("       trailer was: %s\n", closed.back().c_str());
  }
  std::printf("  overflow log trailer: %s\n", closed.back().c_str());
}

/** @brief A run that records nothing never even probes the sink. */
void testEmptyRunNeverProbes() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<8> buffer;
  Logger logger(sink, buffer);
  logger.addLength("x");

  for (int i = 0; i < 5; ++i) {
    CHECK_EQ(static_cast<double>(logger.flush()), 0.0);
  }
  // Nothing was sampled, so no file should have been opened on the robot.
  CHECK_EQ(static_cast<double>(sink.availabilityProbes()), 0.0);
  CHECK(sink.data().empty());
}

/**
 * @brief A precision wide enough to overrun the line buffer truncates the row
 * instead of walking off the end of it.
 *
 * @details The line buffer holds any row at the widest precision that means
 * anything for a double (17 significant digits, 24 characters per field). This
 * asks for 200, which is nonsense but is a public knob, and checks the guard.
 */
void testWideRowIsTruncatedNotOverrun() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<8> buffer;
  Logger logger(sink, buffer,
                LoggerConfig{.period = 10.0 * mclib::units::millisecond, .precision = 200});
  for (std::size_t i = 0; i < mclib::telemetry::kMaxChannels; ++i) {
    char name[kNameBufferSize];
    std::snprintf(name, sizeof(name), "channel_with_a_long_name_%02u", static_cast<unsigned>(i));
    auto channel = logger.addNumber(name);
    logger.set(channel, -1.0 / 3.0e-300);
  }
  g_fake_ms = 4000000000u;  // a long match plus a wide timestamp
  CHECK(logger.sample());
  CHECK_EQ(static_cast<double>(logger.flush()), 1.0);

  const std::vector<std::string> lines = sink.lines();
  CHECK_EQ(static_cast<double>(lines.size()), 2.0);
  // Every emitted line stays inside the line buffer and is newline-terminated,
  // so the CSV is still parseable even when a row had to be cut short.
  for (const std::string& line : lines) {
    CHECK(line.size() < 32 + mclib::telemetry::kMaxChannels * 32);
  }
  CHECK(sink.data().back() == '\n');
  logger.close();
}

/** @brief The row cap stops the logger before it can fill a card. */
void testRowCap() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<64> buffer;
  Logger logger(sink, buffer,
                LoggerConfig{.period = 10.0 * mclib::units::millisecond, .max_rows = 5});
  auto n = logger.addNumber("n");

  for (int i = 0; i < 20; ++i) {
    g_fake_ms = static_cast<std::uint32_t>(i * 10);
    logger.set(n, static_cast<double>(i));
    logger.sample();
  }
  CHECK_EQ(static_cast<double>(logger.rowCount()), 5.0);
  CHECK_EQ(static_cast<double>(logger.droppedRows()), 0.0);
  CHECK(logger.stopped());
  CHECK_EQ(static_cast<double>(logger.flush()), 5.0);
}

/** @brief A missing sink is a silent no-op that never blocks the producer. */
void testMissingSinkIsNoOp() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  NullSink sink;
  RowBuffer<4> buffer;
  Logger logger(sink, buffer, LoggerConfig{.period = 10.0 * mclib::units::millisecond});
  auto x = logger.addLength("x");

  for (int i = 0; i < 100; ++i) {
    g_fake_ms = static_cast<std::uint32_t>(i * 10);
    logger.set(x, static_cast<double>(i) * mclib::units::inch);
    logger.sample();
    if (i % 3 == 0) {
      CHECK_EQ(static_cast<double>(logger.flush()), 0.0);
    }
  }
  CHECK(!logger.sinkAlive());
  // Flushing a dead sink drains the ring, so the producer never wedges.
  logger.flush();
  CHECK_EQ(static_cast<double>(logger.pending()), 0.0);
  logger.close();
}

/** @brief A sink that dies mid-run stops the logger instead of crashing. */
void testSinkDiesMidRun() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  class DyingSink : public mclib::telemetry::Sink {
   public:
    bool isAvailable() override { return true; }
    bool write(const char* data, std::size_t length) override {
      if (m_writes++ >= 2) {
        return false;
      }
      m_data.append(data, length);
      return true;
    }
    const std::string& data() const { return m_data; }

   private:
    int m_writes = 0;
    std::string m_data;
  };

  DyingSink sink;
  RowBuffer<64> buffer;
  Logger logger(sink, buffer, LoggerConfig{.period = 10.0 * mclib::units::millisecond});
  auto n = logger.addNumber("n");

  for (int i = 0; i < 10; ++i) {
    g_fake_ms = static_cast<std::uint32_t>(i * 10);
    logger.set(n, static_cast<double>(i));
    logger.sample();
  }
  logger.flush();
  CHECK(!logger.sinkAlive());
  CHECK_EQ(static_cast<double>(logger.pending()), 0.0);
  // Rows the sink swallowed are counted, so the log does not look complete.
  CHECK(logger.discardedRows() > 0);
  logger.close();  // must not crash on the dead sink
}

/** @brief Registration is refused once sampling has started, and past the cap. */
void testRegistrationLimits() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<8> buffer;
  Logger logger(sink, buffer);

  for (std::size_t i = 0; i < mclib::telemetry::kMaxChannels; ++i) {
    char name[8];
    std::snprintf(name, sizeof(name), "c%u", static_cast<unsigned>(i));
    CHECK(logger.addNumber(name).valid());
  }
  CHECK(!logger.addNumber("overflow").valid());

  CHECK(logger.sample());
  auto late = logger.addNumber("late");
  CHECK(!late.valid());
  // Setting through an invalid handle is ignored, not a buffer overrun.
  logger.set(late, 1.0);
}

/**
 * @brief Event rows do not reschedule the periodic sample.
 *
 * @details commit() used to advance the periodic due time, so a 50 ms logger
 * fed an event row every 10 ms saw its deadline move 50 ms per 10 ms of wall
 * time and never sampled again after the first row.
 */
void testCommitDoesNotStarveSampling() {
  g_fake_ms = 0;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<256> buffer;
  Logger logger(sink, buffer, LoggerConfig{.period = 50.0 * mclib::units::millisecond});
  auto x = logger.addLength("x");
  auto event = logger.addNumber("event");

  // A 10 ms control loop for one second, with an event row on every tick.
  int sampled = 0;
  int committed = 0;
  for (int tick = 0; tick <= 100; ++tick) {
    g_fake_ms = static_cast<std::uint32_t>(tick * 10);
    logger.set(x, static_cast<double>(tick) * mclib::units::inch);
    logger.set(event, 1.0);
    if (logger.commit()) {
      ++committed;
    }
    if (logger.sample()) {
      ++sampled;
    }
  }

  // 50 ms period over 0..1000 ms inclusive: 0, 50, ... 1000 = 21 rows.
  CHECK_EQ(static_cast<double>(sampled), 21.0);
  CHECK_EQ(static_cast<double>(committed), 101.0);
  CHECK_EQ(static_cast<double>(logger.rowCount()), 122.0);
  CHECK_EQ(static_cast<double>(logger.droppedRows()), 0.0);
}

/** @brief An event row leaves the periodic deadline exactly where it was. */
void testCommitDoesNotMoveTheDeadline() {
  g_fake_ms = 100;
  mclib::time::ScopedClock clock(fakeClock);

  MemorySink sink;
  RowBuffer<32> buffer;
  Logger logger(sink, buffer, LoggerConfig{.period = 50.0 * mclib::units::millisecond});
  auto x = logger.addNumber("x");
  logger.set(x, 1.0);

  // An event row before the first sample leaves sampling due immediately,
  // exactly as if the event row had never happened.
  CHECK(logger.commit());
  CHECK(logger.due());
  CHECK(logger.sample());
  // Now the deadline is 150 ms, and more event rows must not push it out.
  CHECK(!logger.due());
  g_fake_ms = 120;
  CHECK(logger.commit());
  g_fake_ms = 140;
  CHECK(logger.commit());
  CHECK(!logger.sample());
  g_fake_ms = 150;
  CHECK(logger.due());
  CHECK(logger.sample());
  CHECK(!logger.due());
  CHECK_EQ(static_cast<double>(logger.rowCount()), 5.0);
}

}  // namespace

int main() {
  testHeaderNamesUnits();
  testFixedRateSampling();
  testOverflowDropsNewest();
  testEmptyRunNeverProbes();
  testWideRowIsTruncatedNotOverrun();
  testRowCap();
  testMissingSinkIsNoOp();
  testSinkDiesMidRun();
  testRegistrationLimits();
  testCommitDoesNotStarveSampling();
  testCommitDoesNotMoveTheDeadline();
  return mclib::test::summary("telemetry");
}
