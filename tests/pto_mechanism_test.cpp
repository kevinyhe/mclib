// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// PTOMechanism: shifting, the settle window, the drive guard, the drive
// watchdog, and disable.
//
// Runs on a mclib::time::ScopedClock at 10 ms per tick. The motors are the
// host stand-in in tests/support/host_motor_group.cpp and the solenoid is the
// one in tests/support/host_pneumatic.cpp, so the test reads back the voltage
// the motors got, whether they were braked, and the solenoid pin level.

#include "mclib/command/command.h"
#include "mclib/mechanism/pto_mechanism.hpp"
#include "mclib/time.hpp"
#include "support/host_motor_group.hpp"
#include "support/host_pneumatic.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>

using mclib::mechanism::PTOConfig;
using mclib::mechanism::PTOMechanism;
using mclib::units::millisecond;
namespace host_motors = mclib::test::host_motor_group;
namespace host_pins = mclib::test::host_pneumatic;

namespace {

std::uint32_t g_fake_ms = 0;

constexpr std::uint32_t kTickMs = 10;
constexpr std::int8_t kPort = 5;
constexpr char kAdi = 'C';

PTOConfig config() {
  PTOConfig c;
  c.motor_ports = {kPort, static_cast<std::int8_t>(-(kPort + 1))};
  c.adi_port = kAdi;
  c.engaged_when_extended = true;
  c.initial_engaged = false;
  c.shift_settle_time = 250 * millisecond;
  c.drive_timeout = 100 * millisecond;
  return c;
}

void resetHost() {
  host_motors::reset();
  host_pins::reset();
}

/// One PTO plus the motors and solenoid it drives.
struct Rig {
  PTOMechanism pto;

  explicit Rig(const PTOConfig& c = config()) : pto((resetHost(), c)) {}

  host_motors::Group& motors() { return host_motors::group[kPort]; }
  bool pin() { return host_pins::pin[kAdi]; }

  void tick() {
    g_fake_ms += kTickMs;
    pto.runPeriodic();
  }
  void ticks(int n) {
    for (int i = 0; i < n; ++i) {
      tick();
    }
  }
  double remainingMs() { return pto.remainingSettleTime().convert(millisecond); }
};

// ---------------------------------------------------------------------------
// 1. Construction counts as a shift: nothing drives for shift_settle_time.
// ---------------------------------------------------------------------------
void bootIsAShift() {
  std::printf("-- construction starts the settle window\n");
  Rig rig;
  CHECK(!rig.pto.isEngaged());
  CHECK(!rig.pin());  // disengaged, and engaged_when_extended: retracted
  CHECK(!rig.pto.isShiftSettled());
  CHECK_NEAR(rig.remainingMs(), 250.0, 1e-9);
  CHECK(!rig.pto.driveDisengaged(6.0));
  CHECK(rig.pto.motorsFor(false) == nullptr);

  rig.ticks(24);  // 240 ms
  CHECK(!rig.pto.isShiftSettled());
  CHECK_NEAR(rig.remainingMs(), 10.0, 1e-9);
  CHECK(!rig.pto.driveDisengaged(6.0));
  CHECK(rig.motors().braked);
  CHECK_EQ(rig.motors().voltage_writes, 0.0);

  rig.tick();  // 250 ms
  CHECK(rig.pto.isShiftSettled());
  CHECK_NEAR(rig.remainingMs(), 0.0, 1e-9);
  CHECK(rig.pto.driveDisengaged(6.0));
  CHECK(rig.pto.motorsFor(false) == &rig.pto.motors());
  rig.tick();
  CHECK_EQ(rig.motors().volts, 6.0);
  CHECK(!rig.motors().braked);

  // Starting engaged puts the solenoid on the engaged side from the start.
  PTOConfig engaged = config();
  engaged.initial_engaged = true;
  Rig engaged_rig(engaged);
  CHECK(engaged_rig.pto.isEngaged());
  CHECK(engaged_rig.pin());
}

// ---------------------------------------------------------------------------
// 2. engage(), disengage(), toggle() move the state and the solenoid.
// ---------------------------------------------------------------------------
void shiftingMovesTheSolenoid() {
  std::printf("-- engage, disengage, toggle\n");
  {
    Rig rig;
    rig.pto.engage();
    CHECK(rig.pto.isEngaged());
    // Thrown at once, not on the next periodic().
    CHECK(rig.pin());

    rig.pto.disengage();
    CHECK(!rig.pto.isEngaged());
    CHECK(!rig.pin());

    rig.pto.toggle();
    CHECK(rig.pto.isEngaged());
    CHECK(rig.pin());
    rig.pto.toggle();
    CHECK(!rig.pto.isEngaged());
    CHECK(!rig.pin());

    rig.pto.setEngaged(true);
    CHECK(rig.pto.isEngaged());
    CHECK(rig.pin());
  }
  {
    // engaged_when_extended = false inverts the pin.
    PTOConfig inverted = config();
    inverted.engaged_when_extended = false;
    Rig rig(inverted);
    CHECK(rig.pin());  // disengaged = extended
    rig.pto.engage();
    CHECK(!rig.pin());
    rig.pto.disengage();
    CHECK(rig.pin());
  }
  {
    // periodic() puts the solenoid back if something else moved it.
    Rig rig;
    rig.pto.engage();
    host_pins::pin[kAdi] = false;
    rig.tick();
    CHECK(rig.pin());
  }
}

// ---------------------------------------------------------------------------
// 3. isShiftSettled() timing around a shift.
// ---------------------------------------------------------------------------
void settleTiming() {
  std::printf("-- isShiftSettled() timing\n");
  Rig rig;
  rig.ticks(30);
  CHECK(rig.pto.isShiftSettled());

  rig.pto.engage();
  CHECK(!rig.pto.isShiftSettled());
  CHECK_NEAR(rig.remainingMs(), 250.0, 1e-9);
  rig.ticks(24);
  CHECK(!rig.pto.isShiftSettled());
  rig.tick();
  CHECK(rig.pto.isShiftSettled());

  // Asking for the side it is already on is not a shift.
  rig.pto.engage();
  CHECK(rig.pto.isShiftSettled());
  rig.pto.setEngaged(true);
  CHECK(rig.pto.isShiftSettled());

  // A second shift inside the window restarts it.
  rig.pto.disengage();
  rig.ticks(20);  // 200 ms
  rig.pto.toggle();
  CHECK(rig.pto.isEngaged());
  CHECK_NEAR(rig.remainingMs(), 250.0, 1e-9);
  rig.ticks(24);
  CHECK(!rig.pto.isShiftSettled());
  rig.tick();
  CHECK(rig.pto.isShiftSettled());

  // Settles at exactly 250 ms whatever the clock read at the shift. Before
  // the fix the window was kept as QTime (seconds in a double) and for some
  // start times 250 ms read as 249.99999 ms, so it settled one tick late.
  int late = 0;
  for (int shift = 0; shift < 200; ++shift) {
    rig.pto.toggle();
    rig.ticks(24);
    const bool early = rig.pto.isShiftSettled();
    rig.tick();
    if (early || !rig.pto.isShiftSettled() || rig.remainingMs() != 0.0) {
      ++late;
    }
  }
  std::printf("   200 shifts, %d settled off the 250 ms mark\n", late);
  CHECK_EQ(late, 0.0);

  // A different shift_settle_time.
  PTOConfig quick = config();
  quick.shift_settle_time = 80 * millisecond;
  Rig quick_rig(quick);
  quick_rig.ticks(7);
  CHECK(!quick_rig.pto.isShiftSettled());
  quick_rig.tick();
  CHECK(quick_rig.pto.isShiftSettled());
}

// ---------------------------------------------------------------------------
// 4. The drive guard: only the side that owns a settled PTO may drive.
// ---------------------------------------------------------------------------
void driveGuard() {
  std::printf("-- driveEngaged / driveDisengaged guard\n");
  Rig rig;
  rig.ticks(25);
  CHECK(rig.pto.isShiftSettled());

  // Disengaged: the drivetrain side owns the motors.
  CHECK(!rig.pto.driveEngaged(9.0));
  CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
  CHECK(rig.pto.motorsFor(true) == nullptr);
  CHECK(rig.pto.driveDisengaged(6.0));
  CHECK_EQ(rig.pto.getCommandedVoltage(), 6.0);
  // A dropped write from the other side leaves the owner's voltage alone.
  CHECK(!rig.pto.driveEngaged(-9.0));
  CHECK(!rig.pto.drive(true, -9.0));
  CHECK_EQ(rig.pto.getCommandedVoltage(), 6.0);
  rig.tick();
  CHECK_EQ(rig.motors().volts, 6.0);

  // Writes are clamped to +/-12 V.
  CHECK(rig.pto.driveDisengaged(20.0));
  CHECK_EQ(rig.pto.getCommandedVoltage(), 12.0);
  CHECK(rig.pto.drive(false, -20.0));
  CHECK_EQ(rig.pto.getCommandedVoltage(), -12.0);
  CHECK(rig.pto.driveDisengaged(6.0));
  rig.tick();

  // Shift while the drivetrain is driving: torque drops before periodic().
  const int brakes = rig.motors().brakes;
  rig.pto.engage();
  CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
  CHECK_EQ(rig.motors().brakes, brakes + 1.0);
  CHECK(rig.motors().braked);

  // During the settle window neither side may drive, and periodic() brakes.
  const int writes = rig.motors().voltage_writes;
  for (int i = 0; i < 24; ++i) {
    rig.tick();
    CHECK(!rig.pto.driveEngaged(9.0));
    CHECK(!rig.pto.driveDisengaged(6.0));
  }
  CHECK(rig.pto.motorsFor(true) == nullptr);
  CHECK(rig.pto.motorsFor(false) == nullptr);
  CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
  CHECK(rig.motors().braked);
  CHECK_EQ(rig.motors().voltage_writes, static_cast<double>(writes));

  // Settled on the engaged side: now only the lift side drives.
  rig.tick();
  CHECK(rig.pto.isShiftSettled());
  CHECK(!rig.pto.driveDisengaged(6.0));
  CHECK(rig.pto.motorsFor(false) == nullptr);
  CHECK(rig.pto.driveEngaged(9.0));
  CHECK(rig.pto.motorsFor(true) == &rig.pto.motors());
  rig.tick();
  CHECK_EQ(rig.motors().volts, 9.0);

  // Shifting back: the lift's voltage does not carry over to the drivetrain.
  rig.pto.disengage();
  rig.ticks(25);
  CHECK(rig.pto.isShiftSettled());
  CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
  CHECK(rig.motors().braked);
  CHECK_EQ(rig.motors().volts, 0.0);

  // A zero write from the owner brakes rather than coasting.
  CHECK(rig.pto.driveDisengaged(0.0));
  rig.tick();
  CHECK(rig.motors().braked);
}

// ---------------------------------------------------------------------------
// 5. The watchdog decays a voltage the owner stopped refreshing.
// ---------------------------------------------------------------------------
void driveWatchdog() {
  std::printf("-- drive_timeout watchdog\n");
  {
    Rig rig;
    rig.ticks(25);
    CHECK(rig.pto.driveDisengaged(6.0));
    rig.ticks(9);  // 90 ms after the write
    CHECK_EQ(rig.motors().volts, 6.0);
    rig.tick();  // 100 ms
    CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
    CHECK(rig.motors().braked);

    // Writing every tick keeps it alive.
    for (int i = 0; i < 50; ++i) {
      CHECK(rig.pto.driveDisengaged(4.0));
      rig.tick();
    }
    CHECK_EQ(rig.motors().volts, 4.0);

    // Decays at exactly 100 ms whatever the clock read at the write.
    int off = 0;
    for (int write = 0; write < 200; ++write) {
      rig.pto.driveDisengaged(4.0);
      rig.ticks(9);
      const bool live_at_90 = rig.motors().volts == 4.0;
      rig.tick();
      if (!live_at_90 || !rig.motors().braked) {
        ++off;
      }
    }
    std::printf("   200 writes, %d decayed off the 100 ms mark\n", off);
    CHECK_EQ(off, 0.0);
  }
  {
    // drive_timeout = 0 turns the watchdog off.
    PTOConfig latching = config();
    latching.drive_timeout = 0 * millisecond;
    Rig rig(latching);
    rig.ticks(25);
    CHECK(rig.pto.driveDisengaged(6.0));
    rig.ticks(500);
    CHECK_EQ(rig.pto.getCommandedVoltage(), 6.0);
    CHECK_EQ(rig.motors().volts, 6.0);
  }
}

// ---------------------------------------------------------------------------
// 6. Disable and stop(): zero the motors at once, from either side.
// ---------------------------------------------------------------------------
void disableStops() {
  std::printf("-- onDisabled() and stop()\n");
  {
    Rig rig;
    rig.pto.engage();
    rig.ticks(25);
    CHECK(rig.pto.driveEngaged(9.0));
    rig.tick();
    CHECK_EQ(rig.motors().volts, 9.0);

    const int brakes = rig.motors().brakes;
    rig.pto.onDisabled();
    // Braked right away, without waiting for periodic().
    CHECK_EQ(rig.motors().brakes, brakes + 1.0);
    CHECK(rig.motors().braked);
    CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
    // Disable does not shift the PTO.
    CHECK(rig.pto.isEngaged());
    CHECK(rig.pin());
    CHECK(rig.pto.isShiftSettled());

    // And the old voltage does not come back.
    rig.ticks(10);
    CHECK(rig.motors().braked);
    CHECK_EQ(rig.motors().volts, 0.0);

    // The owner can drive again afterwards.
    CHECK(rig.pto.driveEngaged(5.0));
    rig.tick();
    CHECK_EQ(rig.motors().volts, 5.0);
  }
  {
    // stop() is allowed mid-shift and from the side that does not own the PTO.
    Rig rig;
    rig.ticks(25);
    CHECK(rig.pto.driveDisengaged(6.0));
    rig.tick();
    rig.pto.stop();
    CHECK_EQ(rig.pto.getCommandedVoltage(), 0.0);
    CHECK(rig.motors().braked);
    rig.pto.engage();
    rig.pto.stop();
    CHECK(rig.motors().braked);
  }
  {
    // A disabled subsystem is skipped by runPeriodic(): no writes at all.
    Rig rig;
    rig.ticks(25);
    rig.pto.setEnabled(false);
    CHECK(rig.pto.driveDisengaged(6.0));
    const int writes = rig.motors().voltage_writes;
    rig.tick();
    CHECK_EQ(rig.motors().voltage_writes, static_cast<double>(writes));
    rig.pto.setEnabled(true);
    CHECK(rig.pto.driveDisengaged(6.0));
    rig.tick();
    CHECK_EQ(rig.motors().volts, 6.0);
  }
}

// ---------------------------------------------------------------------------
// 7. makeShiftCommand() holds until the shift has settled.
// ---------------------------------------------------------------------------
void shiftCommandWaitsForSettle() {
  std::printf("-- makeShiftCommand()\n");
  Rig rig;
  rig.ticks(25);
  std::unique_ptr<Command> command = rig.pto.makeShiftCommand(true);
  command->initialize();
  CHECK(rig.pto.isEngaged());
  int ticks = 0;
  while (!command->isFinished() && ticks < 100) {
    command->execute();
    rig.tick();
    ++ticks;
  }
  command->end(false);
  CHECK_EQ(ticks, 25.0);
  CHECK(rig.pto.isShiftSettled());
  CHECK(rig.motors().braked);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  bootIsAShift();
  shiftingMovesTheSolenoid();
  settleTiming();
  driveGuard();
  driveWatchdog();
  disableStops();
  shiftCommandWaitsForSettle();
  return mclib::test::summary("pto_mechanism");
}
