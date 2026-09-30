// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// MotorStateMechanism: how a voltage map's result is spread over the motors
// (empty, one value, one per motor, too few, too many), state changes, the
// inherited state commands, a new state mid-command, and disable.
//
// The motors are real device::Motor objects over the host pros::Motor in
// tests/support/host_devices.cpp, which records every command per port.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/motor_state_mechanism.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "support/host_devices.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <type_traits>
#include <vector>

using mclib::device::Gearset;
using mclib::mechanism::MotorStateMechanism;
namespace hd = mclib::test::host_devices;

namespace {

std::uint32_t g_fake_ms = 0;

enum class Intake { Off, In, Out, Split, Short, Long };

using IntakeMech = MotorStateMechanism<Intake>;

static_assert(!std::is_copy_constructible_v<IntakeMech>);
static_assert(!std::is_move_constructible_v<IntakeMech>);

std::vector<double> voltages(const Intake& state) {
  switch (state) {
    case Intake::Off: return {};
    case Intake::In: return {12.0};
    case Intake::Out: return {-6.0};
    case Intake::Split: return {3.0, -4.0, 5.0};
    case Intake::Short: return {7.0, 8.0};
    case Intake::Long: return {1.0, 2.0, 3.0, 4.0};
  }
  return {};
}

double mv(int port) { return static_cast<double>(hd::motors[port].millivolts); }
double writes(int port) { return static_cast<double>(hd::motors[port].voltage_writes); }

void construction() {
  std::printf("-- construction writes nothing\n");
  hd::reset();
  IntakeMech m({1, -2, 3}, Gearset::Green, Intake::In, voltages);
  CHECK_EQ(static_cast<double>(m.motorCount()), 3.0);
  CHECK(writes(1) == 0.0 && writes(2) == 0.0);

  IntakeMech from_vector(std::vector<std::int8_t>{4, 5}, Gearset::Red, Intake::Off,
                         voltages);
  CHECK_EQ(static_cast<double>(from_vector.motorCount()), 2.0);

  IntakeMech none(std::vector<std::int8_t>{}, Gearset::Blue, Intake::Split, voltages);
  CHECK_EQ(static_cast<double>(none.motorCount()), 0.0);
  none.runPeriodic();  // nothing to drive, must not index past the end
  CHECK(true);
}

void voltage_map_shapes() {
  std::printf("-- how the map result is spread over the motors\n");
  hd::reset();
  IntakeMech m({1, -2, 3}, Gearset::Blue, Intake::In, voltages);

  // One value: every motor gets it. A reversed port gets the same command;
  // the reversal is the motor's job.
  m.runPeriodic();
  CHECK_EQ(mv(1), 12000.0);
  CHECK_EQ(mv(2), 12000.0);
  CHECK_EQ(mv(3), 12000.0);

  // Empty: every motor gets 0 V.
  m.setState(Intake::Off);
  m.runPeriodic();
  CHECK_EQ(mv(1), 0.0);
  CHECK_EQ(mv(2), 0.0);
  CHECK_EQ(mv(3), 0.0);

  // One per motor.
  m.setState(Intake::Split);
  m.runPeriodic();
  CHECK_EQ(mv(1), 3000.0);
  CHECK_EQ(mv(2), -4000.0);
  CHECK_EQ(mv(3), 5000.0);

  // Too few: the motors past the end get 0 V.
  m.setState(Intake::Short);
  m.runPeriodic();
  CHECK_EQ(mv(1), 7000.0);
  CHECK_EQ(mv(2), 8000.0);
  CHECK_EQ(mv(3), 0.0);

  // Too many: the extra values are ignored.
  m.setState(Intake::Long);
  m.runPeriodic();
  CHECK_EQ(mv(1), 1000.0);
  CHECK_EQ(mv(2), 2000.0);
  CHECK_EQ(mv(3), 3000.0);
  CHECK_EQ(writes(4), 0.0);

  // Out of range and non-finite values are cut down by device::Motor.
  IntakeMech wild({6}, Gearset::Blue, Intake::In,
                  [](const Intake& s) -> std::vector<double> {
                    if (s == Intake::In) return {20.0};
                    return {std::numeric_limits<double>::quiet_NaN()};
                  });
  wild.runPeriodic();
  CHECK_EQ(mv(6), 12000.0);
  wild.setState(Intake::Out);
  wild.runPeriodic();
  CHECK_EQ(mv(6), 0.0);

  // Every tick rewrites, even with no state change.
  const double before = writes(1);
  m.runPeriodic();
  m.runPeriodic();
  CHECK_EQ(writes(1), before + 2.0);

  // setState alone writes nothing; the next tick does.
  m.setState(Intake::Out);
  CHECK_EQ(mv(1), 1000.0);
  m.runPeriodic();
  CHECK_EQ(mv(1), -6000.0);

  // A skipped subsystem writes nothing.
  m.setEnabled(false);
  m.setState(Intake::In);
  const double skipped = writes(1);
  m.runPeriodic();
  CHECK_EQ(writes(1), skipped);
  m.setEnabled(true);
}

void motor_access() {
  std::printf("-- motor(index)\n");
  hd::reset();
  IntakeMech m({7, 8}, Gearset::Blue, Intake::Off, voltages);
  m.motor(1).setVoltage(4.5);
  CHECK_EQ(mv(8), 4500.0);
  CHECK_EQ(writes(7), 0.0);
  // motor(index) does not check the index. motorCount() is the bound.
}

void commands_and_disable() {
  std::printf("-- commands, a new state mid-command, and disable\n");
  hd::reset();
  mclib::test::setCompetitionStatus(0);
  IntakeMech m({1, 2}, Gearset::Blue, Intake::Off, voltages);
  m.setName("intake");
  m.setDefaultCommand(m.makeStateCommand(Intake::Off));
  m.registerSelf();

  auto run_ticks = [](int n) {
    for (int i = 0; i < n; ++i) {
      g_fake_ms += 10;
      CommandScheduler::run();
    }
  };

  run_ticks(2);
  CHECK_EQ(mv(1), 0.0);

  // Hold In for 100 ms, then the default command takes over again.
  g_fake_ms = 1000;
  std::unique_ptr<Command> pulse =
      m.makeStateForCommand(Intake::In, 100.0 * mclib::units::millisecond);
  CommandScheduler::schedule(pulse.get());
  CHECK(m.getState() == Intake::In);
  int finished_on = -1;
  for (int i = 1; i <= 30; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    if (!CommandScheduler::scheduled(pulse.get())) {
      finished_on = i;
      break;
    }
    CHECK_EQ(mv(1), 12000.0);
  }
  std::printf("   100 ms pulse at 10 ms/tick: finished on tick %d\n", finished_on);
  CHECK_EQ(static_cast<double>(finished_on), 10.0);
  run_ticks(2);
  CHECK(m.getState() == Intake::Off);
  CHECK_EQ(mv(1), 0.0);

  // Mid-command: Out interrupts a running In. The old command must not drag
  // the state back.
  std::unique_ptr<Command> in = m.makeStateCommand(Intake::In);
  std::unique_ptr<Command> out = m.makeStateCommand(Intake::Out);
  CommandScheduler::schedule(in.get());
  run_ticks(2);
  CHECK_EQ(mv(2), 12000.0);
  CommandScheduler::schedule(out.get());
  CHECK(!CommandScheduler::scheduled(in.get()));
  run_ticks(3);
  CHECK(m.getState() == Intake::Out);
  CHECK_EQ(mv(2), -6000.0);

  // makeStateUntilCommand finishes when its predicate says so.
  bool done = false;
  std::unique_ptr<Command> until =
      m.makeStateUntilCommand(Intake::Split, [&done]() { return done; });
  CommandScheduler::schedule(until.get());
  run_ticks(2);
  CHECK(CommandScheduler::scheduled(until.get()));
  CHECK_EQ(mv(2), -4000.0);
  done = true;
  run_ticks(1);
  CHECK(!CommandScheduler::scheduled(until.get()));

  // Disable while In is running.
  // A RunCommand sets its state in execute(), which runs after periodic(), so
  // the voltage lands one tick later.
  CommandScheduler::schedule(in.get());
  run_ticks(2);
  CHECK_EQ(mv(1), 12000.0);
  mclib::test::setCompetitionStatus(1);
  const double disabled_writes = writes(1);
  run_ticks(3);
  CHECK(!CommandScheduler::scheduled(in.get()));
  // Nothing is written while disabled.
  CHECK_EQ(writes(1), disabled_writes);

  // onDisabled() puts the mechanism in its off state (here the initial state,
  // Off), so after re-enable periodic() drives 0 V on the first tick instead
  // of the pre-disable voltage.
  CHECK(m.getState() == Intake::Off);
  mclib::test::setCompetitionStatus(0);
  int stale_ticks = 0;
  for (int i = 0; i < 5; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    if (mv(1) == 12000.0) ++stale_ticks;
  }
  std::printf("   ticks at the pre-disable voltage after re-enable: %d\n", stale_ticks);
  CHECK_EQ(stale_ticks, 0);
  CHECK(m.getState() == Intake::Off);
  CHECK_EQ(mv(1), 0.0);

  CommandScheduler::unregisterSubsystem(&m);
}

}  // namespace

void explicit_off_state() {
  std::printf("-- an off state that is not the initial state\n");
  hd::reset();
  mclib::test::setCompetitionStatus(0);
  // Starts running In, with no default command to turn it off.
  IntakeMech m({1, 2}, Gearset::Blue, Intake::In, Intake::Off, voltages);
  CHECK(m.offState() == Intake::Off);
  m.registerSelf();
  g_fake_ms += 10;
  CommandScheduler::run();
  CHECK_EQ(mv(1), 12000.0);

  mclib::test::setCompetitionStatus(1);
  g_fake_ms += 10;
  CommandScheduler::run();
  CHECK(m.getState() == Intake::Off);
  mclib::test::setCompetitionStatus(0);
  g_fake_ms += 10;
  CommandScheduler::run();
  CHECK_EQ(mv(1), 0.0);
  CHECK_EQ(mv(2), 0.0);
  CommandScheduler::unregisterSubsystem(&m);

  // The constructors without one use the initial state.
  IntakeMech plain({3}, Gearset::Blue, Intake::Split, voltages);
  CHECK(plain.offState() == Intake::Split);
  IntakeMech from_vector(std::vector<std::int8_t>{4}, Gearset::Blue, Intake::Out,
                         Intake::Off, voltages);
  CHECK(from_vector.offState() == Intake::Off);
}

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  construction();
  voltage_map_shapes();
  motor_access();
  commands_and_disable();
  explicit_off_state();
  return mclib::test::summary("motor_state_mechanism");
}
