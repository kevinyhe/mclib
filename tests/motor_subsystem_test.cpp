// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// MotorSubsystem: voltage and percent clamping, non-finite input, what reaches
// the motors each tick, the command factories, a new command mid-command, and
// disable.
//
// The motors are a real device::MotorGroup over the host pros::MotorGroup in
// tests/support/host_devices.cpp, which records every command per port.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/motor_subsystem.hpp"
#include "mclib/time.hpp"
#include "support/host_devices.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <type_traits>

using mclib::mechanism::MotorSubsystem;
namespace hd = mclib::test::host_devices;

namespace {

std::uint32_t g_fake_ms = 0;

static_assert(!std::is_copy_constructible_v<MotorSubsystem>);
static_assert(!std::is_move_constructible_v<MotorSubsystem>);

double mv(int port) { return static_cast<double>(hd::motors[port].millivolts); }
double writes(int port) { return static_cast<double>(hd::motors[port].voltage_writes); }
double brakes(int port) { return static_cast<double>(hd::motors[port].brakes); }

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

void state_changes() {
  std::printf("-- setVoltage, setPercent, stop, clamping\n");
  hd::reset();
  MotorSubsystem m({1, -2});
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  // Construction writes nothing.
  CHECK_EQ(writes(1), 0.0);

  m.setVoltage(6.5);
  CHECK_EQ(m.getCommandedVoltage(), 6.5);
  // setVoltage alone writes nothing; the next tick does, to every motor.
  CHECK_EQ(writes(1), 0.0);
  m.runPeriodic();
  CHECK_EQ(mv(1), 6500.0);
  CHECK_EQ(mv(2), 6500.0);

  m.setVoltage(20.0);
  CHECK_EQ(m.getCommandedVoltage(), 12.0);
  m.setVoltage(-20.0);
  CHECK_EQ(m.getCommandedVoltage(), -12.0);
  m.setPercent(0.5);
  CHECK_EQ(m.getCommandedVoltage(), 6.0);
  m.setPercent(-3.0);
  CHECK_EQ(m.getCommandedVoltage(), -12.0);
  m.stop();
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  m.runPeriodic();
  CHECK_EQ(mv(1), 0.0);

  // Every tick rewrites.
  const double before = writes(1);
  m.runPeriodic();
  m.runPeriodic();
  CHECK_EQ(writes(1), before + 2.0);

  // A skipped subsystem writes nothing.
  m.setEnabled(false);
  m.setVoltage(3.0);
  m.runPeriodic();
  CHECK_EQ(writes(1), before + 2.0);
  m.setEnabled(true);
  m.runPeriodic();
  CHECK_EQ(mv(1), 3000.0);

  // motors() is the live group: a direct command reaches both ports.
  m.motors().setVoltage(-1.5);
  CHECK_EQ(mv(1), -1500.0);
  CHECK_EQ(mv(2), -1500.0);
}

void non_finite_input() {
  std::printf("-- NaN and infinity\n");
  hd::reset();
  MotorSubsystem m({3});

  // The motor gets 0 V for a NaN, and the commanded voltage has to agree
  // with what the motor gets.
  m.setVoltage(5.0);
  m.setVoltage(kNaN);
  m.runPeriodic();
  CHECK_EQ(mv(3), 0.0);
  CHECK_EQ(m.getCommandedVoltage(), 0.0);

  m.setVoltage(5.0);
  m.setPercent(kNaN);
  CHECK_EQ(m.getCommandedVoltage(), 0.0);

  // Infinity also becomes 0 V, as it does in device::MotorGroup and in
  // PositionMechanism. It comes from a division by zero, not from a driver
  // asking for full power.
  m.setVoltage(5.0);
  m.setVoltage(kInf);
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  m.setVoltage(5.0);
  m.setVoltage(-kInf);
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  m.setVoltage(5.0);
  m.setPercent(kInf);
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  m.runPeriodic();
  CHECK_EQ(mv(3), 0.0);

  // Same through the command factory.
  mclib::test::setCompetitionStatus(0);
  std::unique_ptr<Command> nan_cmd = m.makeVoltageCommand(kNaN);
  m.setName("nan");
  m.registerSelf();
  CommandScheduler::schedule(nan_cmd.get());
  CommandScheduler::run();
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  CommandScheduler::cancel(nan_cmd.get());
  CommandScheduler::unregisterSubsystem(&m);
}

void commands_and_disable() {
  std::printf("-- commands, a new command mid-command, and disable\n");
  hd::reset();
  mclib::test::setCompetitionStatus(0);
  MotorSubsystem m({4, 5}, mclib::device::Gearset::Green);
  m.setName("flywheel");
  m.setDefaultCommand(m.makeStopCommand());
  m.registerSelf();

  auto run_ticks = [](int n) {
    for (int i = 0; i < n; ++i) {
      g_fake_ms += 10;
      CommandScheduler::run();
    }
  };

  run_ticks(2);
  CHECK_EQ(mv(4), 0.0);

  // makeVoltageCommand clamps, and holds its voltage while it runs.
  std::unique_ptr<Command> full = m.makeVoltageCommand(15.0);
  CommandScheduler::schedule(full.get());
  run_ticks(3);
  CHECK(CommandScheduler::scheduled(full.get()));
  CHECK_EQ(m.getCommandedVoltage(), 12.0);
  CHECK_EQ(mv(4), 12000.0);
  CHECK_EQ(mv(5), 12000.0);

  // Mid-command: a percent command interrupts the voltage command. From then
  // on only the new value reaches the motors.
  std::unique_ptr<Command> half = m.makePercentCommand(-0.5);
  CommandScheduler::schedule(half.get());
  CHECK(!CommandScheduler::scheduled(full.get()));
  run_ticks(3);
  CHECK_EQ(m.getCommandedVoltage(), -6.0);
  CHECK_EQ(mv(4), -6000.0);

  // Out of range percent clamps.
  std::unique_ptr<Command> over = m.makePercentCommand(2.0);
  CommandScheduler::schedule(over.get());
  run_ticks(2);
  CHECK_EQ(m.getCommandedVoltage(), 12.0);

  // Cancelled: the default stop command takes over.
  CommandScheduler::cancel(over.get());
  run_ticks(3);
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  CHECK_EQ(mv(4), 0.0);

  // Disable while running at full voltage.
  CommandScheduler::schedule(full.get());
  run_ticks(2);
  CHECK_EQ(mv(4), 12000.0);
  const double brakes_before = brakes(4);
  mclib::test::setCompetitionStatus(1);
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(full.get()));
  // onDisabled() zeroes the state and brakes every motor at once.
  CHECK_EQ(m.getCommandedVoltage(), 0.0);
  CHECK_EQ(brakes(4), brakes_before + 1.0);
  CHECK_EQ(brakes(5), brakes_before + 1.0);
  const double disabled_writes = writes(4);
  run_ticks(3);
  CHECK_EQ(writes(4), disabled_writes);

  // Re-enabled: the first tick writes 0 V, not the pre-disable 12 V.
  mclib::test::setCompetitionStatus(0);
  CommandScheduler::run();
  CHECK_EQ(writes(4), disabled_writes + 1.0);
  CHECK_EQ(mv(4), 0.0);

  // onDisabled() called directly is safe to repeat.
  m.onDisabled();
  m.onDisabled();
  CHECK_EQ(m.getCommandedVoltage(), 0.0);

  CommandScheduler::unregisterSubsystem(&m);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  state_changes();
  non_finite_input();
  commands_and_disable();
  return mclib::test::summary("motor_subsystem");
}
