// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// ConveyorMechanism: the plain states, jam detection, the unjam reversal, the
// retry budget and its refill, a jam that clears mid-reversal, and disable.
//
// Runs on a mclib::time::ScopedClock at 10 ms per tick. The motors are the
// host stand-in in tests/support/host_devices.cpp: the test sets the current
// and speed both ports report and reads back the voltage they were sent.
//
// Timing with the thresholds below (dwell 250 ms, unjam 250 ms, clear 1000 ms):
// the first stalled tick starts the dwell timer, the unjam starts 25 ticks
// later, reverses for 25 ticks, and the 26th tick after the unjam started is
// back on the normal state.

#include "mclib/mechanism/conveyor_mechanism.hpp"
#include "mclib/time.hpp"
#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>

using mclib::mechanism::ConveyorConfig;
using mclib::mechanism::ConveyorMechanism;
using mclib::mechanism::ConveyorState;
namespace host = mclib::test::host_devices;

namespace {

std::uint32_t g_fake_ms = 0;

constexpr std::uint32_t kTickMs = 10;
constexpr std::int8_t kPort = 3;

// Readings on either side of the thresholds in config().
constexpr std::int32_t kStallMa = 2500;  // above 2.0 A
constexpr std::int32_t kIdleMa = 500;    // below 2.0 A
constexpr double kStallRpm = 2.0;        // below 10 RPM
constexpr double kRunRpm = 400.0;        // above 10 RPM

ConveyorConfig config() {
  ConveyorConfig c;
  c.motor_ports = {kPort, static_cast<std::int8_t>(kPort + 1)};
  c.forward_voltage = 12.0;
  c.reverse_voltage = -12.0;
  c.index_voltage = 8.0;
  c.jam_current_amps = 2.0;
  c.jam_velocity_rpm = 10.0;
  c.jam_dwell_ms = 250.0;
  c.unjam_ms = 250.0;
  c.unjam_voltage = -8.0;
  c.jam_clear_ms = 1000.0;
  c.max_unjam_retries = 3;
  c.max_voltage = 12.0;
  return c;
}

/// One conveyor plus the motor group it drives.
struct Rig {
  ConveyorMechanism conveyor;
  int ticks = 0;

  explicit Rig(const ConveyorConfig& c = config(),
               ConveyorMechanism::SensorGate gate = nullptr)
      : conveyor((host::reset(), c), std::move(gate)) {
    running();
  }

  /// The first motor. The conveyor sends both motors the same command.
  host::Motor& motors() { return host::motors[kPort]; }
  double volts() { return motors().volts(); }

  /// Set the current and speed both motors report.
  void reads(std::int32_t current_ma, double rpm) {
    for (const int port : {static_cast<int>(kPort), kPort + 1}) {
      host::motors[port].current_ma = current_ma;
      host::motors[port].rpm = rpm;
    }
  }
  void stalled() { reads(kStallMa, kStallRpm); }
  void running() { reads(kIdleMa, kRunRpm); }

  void tick() {
    ++ticks;
    g_fake_ms += kTickMs;
    conveyor.runPeriodic();
  }

  /// Ticks until the motors are sent unjam_voltage. Returns ticks taken, or -1.
  int ticksUntilUnjam(int max_ticks) {
    for (int i = 1; i <= max_ticks; ++i) {
      tick();
      if (volts() == -8.0) {
        return i;
      }
    }
    return -1;
  }

  /// Ticks while the motors read unjam_voltage. Returns how many ticks that
  /// was, counting the tick already seen by ticksUntilUnjam().
  int unjamLength(int max_ticks) {
    int length = 1;
    for (int i = 0; i < max_ticks; ++i) {
      tick();
      if (volts() != -8.0) {
        return length;
      }
      ++length;
    }
    return length;
  }

  /// Runs a permanent stall until the jam latches. Returns how many unjam
  /// reversals ran first, or -1 if it never latched within max_ticks.
  int attemptsBeforeLatch(int max_ticks) {
    stalled();
    int attempts = 0;
    bool reversing = false;
    for (int i = 0; i < max_ticks; ++i) {
      tick();
      const bool now_reversing = volts() == -8.0;
      if (now_reversing && !reversing) {
        ++attempts;
      }
      reversing = now_reversing;
      if (conveyor.isJammed()) {
        return attempts;
      }
    }
    return -1;
  }
};

// ---------------------------------------------------------------------------
// 1. Each state sends its voltage. Nothing is written until periodic() runs.
// ---------------------------------------------------------------------------
void statesSendTheirVoltage() {
  std::printf("-- states send their voltage\n");
  {
    Rig rig;
    CHECK(rig.conveyor.getConveyorState() == ConveyorState::Stopped);
    CHECK(host::motors.size() == 2 && host::motors.count(kPort) == 1 &&
          host::motors.count(kPort + 1) == 1);

    rig.conveyor.setConveyorState(ConveyorState::Forward);
    CHECK_EQ(rig.motors().voltage_writes, 0.0);
    rig.tick();
    CHECK_EQ(rig.volts(), 12.0);

    rig.conveyor.setConveyorState(ConveyorState::Reverse);
    rig.tick();
    CHECK_EQ(rig.volts(), -12.0);

    rig.conveyor.stop();
    CHECK(rig.conveyor.getConveyorState() == ConveyorState::Stopped);
    rig.tick();
    CHECK_EQ(rig.volts(), 0.0);

    // Without a sensor gate IndexToSensor holds at zero volts.
    rig.conveyor.setConveyorState(ConveyorState::IndexToSensor);
    rig.tick();
    CHECK_EQ(rig.volts(), 0.0);
    CHECK(!rig.conveyor.hasObject());
  }

  // max_voltage caps every configured voltage.
  ConveyorConfig capped = config();
  capped.max_voltage = 10.0;
  Rig capped_rig(capped);
  capped_rig.conveyor.setConveyorState(ConveyorState::Forward);
  capped_rig.tick();
  CHECK_EQ(capped_rig.volts(), 10.0);
  capped_rig.conveyor.setConveyorState(ConveyorState::Reverse);
  capped_rig.tick();
  CHECK_EQ(capped_rig.volts(), -10.0);
}

// ---------------------------------------------------------------------------
// 2. IndexToSensor runs until the gate reads true and re-arms when it clears.
// ---------------------------------------------------------------------------
void indexStopsAtTheSensor() {
  std::printf("-- IndexToSensor stops at the sensor\n");
  bool object = false;
  Rig rig(config(), [&object]() { return object; });
  rig.conveyor.setConveyorState(ConveyorState::IndexToSensor);
  rig.tick();
  CHECK_EQ(rig.volts(), 8.0);
  CHECK(!rig.conveyor.hasObject());

  object = true;
  rig.tick();
  CHECK_EQ(rig.volts(), 0.0);
  CHECK(rig.conveyor.hasObject());

  // Holding at zero volts is not a jam, whatever the motors read.
  rig.stalled();
  for (int i = 0; i < 100; ++i) {
    rig.tick();
  }
  CHECK_EQ(rig.volts(), 0.0);
  CHECK(!rig.conveyor.isJammed());

  rig.running();
  object = false;
  rig.tick();
  CHECK_EQ(rig.volts(), 8.0);
}

// ---------------------------------------------------------------------------
// 3. A jam needs both conditions, held for the whole jam_dwell_ms.
// ---------------------------------------------------------------------------
void jamNeedsBothConditionsForTheDwell() {
  std::printf("-- jam detection needs current and speed for jam_dwell_ms\n");
  {
    // High current while still moving: loaded, not jammed.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.reads(kStallMa, kRunRpm);
    CHECK_EQ(rig.ticksUntilUnjam(300), -1.0);
    CHECK_EQ(rig.volts(), 12.0);
  }
  {
    // Slow with low current: not pushing against anything.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.reads(kIdleMa, kStallRpm);
    CHECK_EQ(rig.ticksUntilUnjam(300), -1.0);
    CHECK_EQ(rig.volts(), 12.0);
  }
  {
    // Both: the first stalled tick starts the timer and the unjam starts
    // 250 ms later, on tick 26.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.stalled();
    const int ticks = rig.ticksUntilUnjam(100);
    std::printf("   stalled from tick 1, unjam on tick %d\n", ticks);
    CHECK_EQ(ticks, 26.0);
    CHECK(!rig.conveyor.isJammed());
  }
  {
    // A stall that lets go one tick short of the dwell restarts the timer.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.stalled();
    for (int i = 0; i < 25; ++i) {
      rig.tick();
    }
    CHECK_EQ(rig.volts(), 12.0);
    rig.running();
    rig.tick();
    CHECK_EQ(rig.volts(), 12.0);
    rig.stalled();
    CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);
  }
  {
    // Reverse is watched for jams too.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Reverse);
    rig.stalled();
    for (int i = 0; i < 25; ++i) {
      rig.tick();
    }
    CHECK_EQ(rig.volts(), -12.0);
    rig.tick();
    // The unjam runs opposite to the jam: a jam in Reverse pushes forward.
    CHECK_EQ(rig.volts(), 8.0);
  }
  {
    // Only the size of unjam_voltage counts. A positive setting still backs
    // a Forward jam off toward the intake.
    ConveyorConfig c = config();
    c.unjam_voltage = 6.0;
    Rig rig(c);
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.stalled();
    for (int i = 0; i < 26; ++i) {
      rig.tick();
    }
    CHECK_EQ(rig.volts(), -6.0);
    rig.conveyor.setConveyorState(ConveyorState::Reverse);
    rig.stalled();
    for (int i = 0; i < 26; ++i) {
      rig.tick();
    }
    CHECK_EQ(rig.volts(), 6.0);
  }
}

// ---------------------------------------------------------------------------
// 4. The unjam reverses for unjam_ms, then resumes the state it was in.
// ---------------------------------------------------------------------------
void unjamReversesForUnjamMs() {
  std::printf("-- unjam reverses for unjam_ms\n");
  {
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.stalled();
    CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);
    // The jam cleared at the same moment the reversal started.
    rig.running();
    const int length = rig.unjamLength(100);
    std::printf("   reversed for %d ticks\n", length);
    CHECK_EQ(length, 25.0);
    CHECK_EQ(rig.volts(), 12.0);
    CHECK(rig.conveyor.getConveyorState() == ConveyorState::Forward);
    CHECK(!rig.conveyor.isJammed());

    // With the jam gone it keeps running forward.
    for (int i = 0; i < 200; ++i) {
      rig.tick();
      if (rig.volts() != 12.0) {
        break;
      }
  }
  CHECK_EQ(rig.volts(), 12.0);
  }

  // A different unjam_ms changes the reversal length.
  ConveyorConfig longer = config();
  longer.unjam_ms = 400.0;
  Rig long_rig(longer);
  long_rig.conveyor.setConveyorState(ConveyorState::Forward);
  long_rig.stalled();
  CHECK_EQ(long_rig.ticksUntilUnjam(100), 26.0);
  long_rig.running();
  CHECK_EQ(long_rig.unjamLength(100), 40.0);
}

// ---------------------------------------------------------------------------
// 5. A jam that clears partway through the reversal. The reversal still runs
//    its full length, and the readings taken during it are ignored.
// ---------------------------------------------------------------------------
void jamClearsDuringReversal() {
  std::printf("-- jam clears during the reversal\n");
  Rig rig;
  rig.conveyor.setConveyorState(ConveyorState::Forward);
  rig.stalled();
  CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);

  // Still stalled for the first 10 reversal ticks, free after that.
  int reversing = 1;
  for (int i = 0; i < 9; ++i) {
    rig.tick();
    if (rig.volts() == -8.0) {
      ++reversing;
    }
  }
  CHECK_EQ(reversing, 10.0);
  rig.running();
  CHECK_EQ(rig.unjamLength(100) + 9, 25.0);
  CHECK_EQ(rig.volts(), 12.0);
  CHECK(!rig.conveyor.isJammed());

  // The stall readings from the first 10 reversal ticks did not carry over:
  // a new stall needs the full dwell again.
  rig.stalled();
  CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);
}

// ---------------------------------------------------------------------------
// 6. A permanent jam gets max_unjam_retries reversals, then latches jammed.
// ---------------------------------------------------------------------------
void retryBudgetThenLatch() {
  std::printf("-- retry budget, then the jam latches\n");
  Rig rig;
  rig.conveyor.setConveyorState(ConveyorState::Forward);
  const int attempts = rig.attemptsBeforeLatch(1000);
  std::printf("   %d reversals, latched on tick %d\n", attempts, rig.ticks);
  CHECK_EQ(attempts, 3.0);
  // Three rounds of 25 dwell ticks + 25 reversal ticks, then one more dwell.
  CHECK_EQ(rig.ticks, 176.0);
  CHECK(rig.conveyor.isJammed());
  CHECK_EQ(rig.volts(), 0.0);

  // Latched: held at zero volts, no more reversals, even with the jam gone.
  rig.running();
  bool moved = false;
  for (int i = 0; i < 300; ++i) {
    rig.tick();
    moved = moved || rig.volts() != 0.0;
  }
  CHECK(!moved);
  CHECK(rig.conveyor.isJammed());

  // Asking for the state it is already in is not a state change.
  rig.conveyor.setConveyorState(ConveyorState::Forward);
  rig.tick();
  CHECK(rig.conveyor.isJammed());
  CHECK_EQ(rig.volts(), 0.0);

  // clearJam() releases the latch and gives back the whole budget.
  rig.conveyor.clearJam();
  CHECK(!rig.conveyor.isJammed());
  rig.tick();
  CHECK_EQ(rig.volts(), 12.0);
  CHECK_EQ(rig.attemptsBeforeLatch(1000), 3.0);

  // So does a real state change.
  rig.conveyor.setConveyorState(ConveyorState::Stopped);
  CHECK(!rig.conveyor.isJammed());
  rig.conveyor.setConveyorState(ConveyorState::Forward);
  CHECK_EQ(rig.attemptsBeforeLatch(1000), 3.0);

  // max_unjam_retries = 0 latches on the first jam without reversing.
  ConveyorConfig none = config();
  none.max_unjam_retries = 0;
  Rig none_rig(none);
  none_rig.conveyor.setConveyorState(ConveyorState::Forward);
  CHECK_EQ(none_rig.attemptsBeforeLatch(1000), 0.0);
  CHECK_EQ(none_rig.ticks, 26.0);
}

// ---------------------------------------------------------------------------
// 7. The budget refills after jam_clear_ms of clean running, not before.
// ---------------------------------------------------------------------------
/// One jam and reversal, then `clean_ticks` ticks of clean running counting
/// the tick the reversal ends on. Returns the reversals a permanent jam gets
/// after that.
int attemptsAfterCleanRun(int clean_ticks) {
  Rig rig;
  rig.conveyor.setConveyorState(ConveyorState::Forward);
  rig.stalled();
  CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);
  rig.running();
  CHECK_EQ(rig.unjamLength(100), 25.0);  // ends on the first clean tick
  for (int i = 1; i < clean_ticks; ++i) {
    rig.tick();
  }
  CHECK_EQ(rig.volts(), 12.0);
  return rig.attemptsBeforeLatch(1000);
}

void budgetRefillsAfterCleanRunning() {
  std::printf("-- retry budget refills after jam_clear_ms\n");
  // The clear timer starts on the first clean tick. 1000 ms later is the
  // 101st clean tick.
  const int short_run = attemptsAfterCleanRun(100);
  const int full_run = attemptsAfterCleanRun(101);
  std::printf("   990 ms clean: %d reversals left; 1000 ms clean: %d\n",
              short_run, full_run);
  CHECK_EQ(short_run, 2.0);
  CHECK_EQ(full_run, 3.0);

  // A short stall partway through resets the clean timer.
  {
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.stalled();
    CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);
    rig.running();
    CHECK_EQ(rig.unjamLength(100), 25.0);
    for (int i = 1; i < 60; ++i) {
      rig.tick();
  }
  rig.stalled();
  for (int i = 0; i < 5; ++i) {  // 50 ms, well under the dwell
    rig.tick();
  }
  rig.running();
  for (int i = 0; i < 60; ++i) {  // 600 ms: 1190 ms total but not in a row
    rig.tick();
  }
  CHECK_EQ(rig.attemptsBeforeLatch(1000), 2.0);
  }

  // Stopping is a state change, and a state change resets the whole budget.
  Rig stop_rig;
  stop_rig.conveyor.setConveyorState(ConveyorState::Forward);
  stop_rig.stalled();
  CHECK_EQ(stop_rig.ticksUntilUnjam(100), 26.0);
  stop_rig.conveyor.setConveyorState(ConveyorState::Stopped);
  stop_rig.conveyor.setConveyorState(ConveyorState::Forward);
  CHECK_EQ(stop_rig.attemptsBeforeLatch(1000), 3.0);
}

// ---------------------------------------------------------------------------
// 8. Disable: stops the motors now, cancels a reversal, releases the latch.
// ---------------------------------------------------------------------------
void disableStopsEverything() {
  std::printf("-- onDisabled()\n");
  {
    // Mid-reversal.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.stalled();
    CHECK_EQ(rig.ticksUntilUnjam(100), 26.0);
    rig.tick();
    CHECK_EQ(rig.volts(), -8.0);

    const int brakes = rig.motors().brakes;
    rig.conveyor.onDisabled();
    // The motors stop at once, without waiting for periodic().
    CHECK_EQ(rig.motors().brakes, brakes + 1.0);
    CHECK(rig.motors().braked);
    CHECK_EQ(rig.volts(), 0.0);
    CHECK(rig.conveyor.getConveyorState() == ConveyorState::Stopped);

    // The reversal does not come back on the next tick.
    for (int i = 0; i < 30; ++i) {
      rig.tick();
      CHECK_EQ(rig.volts(), 0.0);
    }
    CHECK(!rig.conveyor.isJammed());
  }
  {
    // Latched jam.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    CHECK_EQ(rig.attemptsBeforeLatch(1000), 3.0);
    rig.conveyor.onDisabled();
    CHECK(!rig.conveyor.isJammed());
    CHECK(rig.conveyor.getConveyorState() == ConveyorState::Stopped);

    // Re-enabled with the jam gone: runs, with the full budget.
    rig.running();
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.tick();
    CHECK_EQ(rig.volts(), 12.0);
    CHECK_EQ(rig.attemptsBeforeLatch(1000), 3.0);
  }
  {
    // A disabled subsystem is skipped by runPeriodic(): no writes at all.
    Rig rig;
    rig.conveyor.setConveyorState(ConveyorState::Forward);
    rig.conveyor.setEnabled(false);
    rig.tick();
    CHECK_EQ(rig.motors().voltage_writes, 0.0);
    rig.conveyor.setEnabled(true);
    rig.tick();
    CHECK_EQ(rig.volts(), 12.0);
  }
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  statesSendTheirVoltage();
  indexStopsAtTheSensor();
  jamNeedsBothConditionsForTheDwell();
  unjamReversesForUnjamMs();
  jamClearsDuringReversal();
  retryBudgetThenLatch();
  budgetRefillsAfterCleanRunning();
  disableStopsEverything();
  return mclib::test::summary("conveyor_mechanism");
}
