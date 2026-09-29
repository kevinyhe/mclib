// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// MultiPositionMechanism: table lookups, out of range indices, states missing
// from the table, next/previous clamping, cycle wrapping, what the setpoint
// sink receives each tick, a new target arriving mid-command, and disable.
//
// The setpoint sink is a lambda that records every value it is handed.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/multi_position_mechanism.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <type_traits>
#include <vector>

using mclib::mechanism::MultiPositionMechanism;

namespace {

std::uint32_t g_fake_ms = 0;

enum class Lift { Down, Low, Mid, High, Unlisted };

using LiftMech = MultiPositionMechanism<Lift>;

static_assert(!std::is_copy_constructible_v<LiftMech>);
static_assert(!std::is_move_constructible_v<LiftMech>);

/// Mechanism plus the list of setpoints its sink received.
struct Rig {
  std::vector<double> pushed;
  LiftMech lift;

  explicit Rig(Lift initial, double default_setpoint = 0.0)
      : lift(initial,
             {{Lift::Down, 0.0}, {Lift::Low, 10.0}, {Lift::Mid, 25.0}, {Lift::High, 40.0}},
             [this](double setpoint) { pushed.push_back(setpoint); },
             default_setpoint) {}

  double last() const { return pushed.empty() ? -999.0 : pushed.back(); }
};

double sz(std::size_t n) { return static_cast<double>(n); }

void lookups() {
  std::printf("-- table lookups and out of range indices\n");
  Rig rig(Lift::Down, -5.0);
  LiftMech& m = rig.lift;

  CHECK_EQ(sz(m.positionCount()), 4.0);
  CHECK(!m.empty());
  CHECK_EQ(sz(m.indexOf(Lift::Mid)), 2.0);
  CHECK(m.indexOf(Lift::Unlisted) == LiftMech::kNoPosition);
  CHECK(m.positionAt(3) != nullptr && *m.positionAt(3) == Lift::High);
  CHECK(m.positionAt(4) == nullptr);
  CHECK(m.positionAt(LiftMech::kNoPosition) == nullptr);
  CHECK_EQ(m.setpointAt(1), 10.0);
  CHECK_EQ(m.setpointAt(4), -5.0);
  CHECK_EQ(m.setpointAt(LiftMech::kNoPosition), -5.0);
  CHECK_EQ(m.setpointFor(Lift::High), 40.0);
  CHECK_EQ(m.setpointFor(Lift::Unlisted), -5.0);
  CHECK_EQ(m.defaultSetpoint(), -5.0);
  CHECK_EQ(m.currentSetpoint(), 0.0);
  CHECK_EQ(sz(m.currentIndex()), 0.0);

  // Nothing is pushed until a tick.
  CHECK(rig.pushed.empty());
}

void stepping() {
  std::printf("-- next/previous clamp, cycle wraps\n");
  Rig rig(Lift::Down);
  LiftMech& m = rig.lift;

  m.previous();
  CHECK(m.getState() == Lift::Down);
  m.next();
  CHECK(m.getState() == Lift::Low);
  m.next();
  m.next();
  CHECK(m.getState() == Lift::High);
  m.next();
  CHECK(m.getState() == Lift::High);
  m.previous();
  CHECK(m.getState() == Lift::Mid);
  m.setPosition(Lift::High);
  m.cycle();
  CHECK(m.getState() == Lift::Down);
  m.cycle();
  CHECK(m.getState() == Lift::Low);
}

void unlisted_state() {
  std::printf("-- state not in the table\n");
  Rig rig(Lift::Unlisted, 3.0);
  LiftMech& m = rig.lift;

  CHECK(m.currentIndex() == LiftMech::kNoPosition);
  CHECK_EQ(m.currentSetpoint(), 3.0);
  m.runPeriodic();
  CHECK_EQ(sz(rig.pushed.size()), 1.0);
  CHECK_EQ(rig.last(), 3.0);

  // Every stepper resyncs to entry 0, not to a neighbour.
  m.previous();
  CHECK(m.getState() == Lift::Down);
  m.setPosition(Lift::Unlisted);
  m.cycle();
  CHECK(m.getState() == Lift::Down);
  m.setPosition(Lift::Unlisted);
  m.next();
  CHECK(m.getState() == Lift::Down);
}

void degenerate_tables() {
  std::printf("-- empty table, single entry, no sink\n");
  {
    std::vector<double> pushed;
    LiftMech m(Lift::Mid, LiftMech::Table{},
               [&pushed](double s) { pushed.push_back(s); }, 7.0);
    CHECK(m.empty());
    m.next();
    m.previous();
    m.cycle();
    CHECK(m.getState() == Lift::Mid);
    m.runPeriodic();
    CHECK_EQ(sz(pushed.size()), 1.0);
    CHECK_EQ(pushed.empty() ? 0.0 : pushed.back(), 7.0);
  }
  {
    LiftMech m(Lift::Low, {{Lift::Low, 12.0}}, nullptr);
    m.cycle();
    CHECK(m.getState() == Lift::Low);
    m.next();
    m.previous();
    CHECK(m.getState() == Lift::Low);
    // No sink: the tick must not throw std::bad_function_call.
    m.runPeriodic();
    CHECK(true);
  }
  {
    // A duplicate state: the first row wins, setpointAt still reads each row.
    std::vector<double> pushed;
    LiftMech m(Lift::Low, {{Lift::Low, 1.0}, {Lift::Low, 2.0}},
               [&pushed](double s) { pushed.push_back(s); });
    CHECK_EQ(m.setpointFor(Lift::Low), 1.0);
    CHECK_EQ(m.setpointAt(1), 2.0);
  }
}

void sink_every_tick() {
  std::printf("-- the sink gets the current setpoint every tick\n");
  Rig rig(Lift::Down);
  LiftMech& m = rig.lift;

  for (int i = 0; i < 3; ++i) m.runPeriodic();
  CHECK_EQ(sz(rig.pushed.size()), 3.0);
  CHECK_EQ(rig.last(), 0.0);

  m.setPosition(Lift::Mid);
  // setState does not push by itself; the next tick does.
  CHECK_EQ(sz(rig.pushed.size()), 3.0);
  m.runPeriodic();
  CHECK_EQ(rig.last(), 25.0);

  // A disabled subsystem is skipped by runPeriodic().
  m.setEnabled(false);
  m.setPosition(Lift::High);
  m.runPeriodic();
  CHECK_EQ(sz(rig.pushed.size()), 4.0);
  m.setEnabled(true);
  m.runPeriodic();
  CHECK_EQ(rig.last(), 40.0);
}

void commands_and_disable() {
  std::printf("-- commands, a new target mid-command, and disable\n");
  mclib::test::setCompetitionStatus(0);
  Rig rig(Lift::Down);
  LiftMech& m = rig.lift;
  m.setName("lift");
  m.setDefaultCommand(m.idleCommand());
  m.registerSelf();

  // makePositionCommand holds its state for as long as it runs.
  std::unique_ptr<Command> to_high = m.makePositionCommand(Lift::High);
  std::unique_ptr<Command> to_low = m.makePositionCommand(Lift::Low);
  CommandScheduler::schedule(to_high.get());
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(m.getState() == Lift::High);
  CHECK(CommandScheduler::scheduled(to_high.get()));
  CHECK_EQ(rig.last(), 40.0);

  // A second command on the same mechanism interrupts the first. From the
  // next tick on only the new state is pushed: the old command does not
  // drag the mechanism back.
  CommandScheduler::schedule(to_low.get());
  CHECK(!CommandScheduler::scheduled(to_high.get()));
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(m.getState() == Lift::Low);
  CHECK_EQ(rig.last(), 10.0);

  // One-shot steppers finish after one pass and keep the state.
  CommandScheduler::cancel(to_low.get());
  std::unique_ptr<Command> next = m.makeNextCommand();
  std::unique_ptr<Command> prev = m.makePreviousCommand();
  std::unique_ptr<Command> cycle = m.makeCycleCommand();
  CommandScheduler::schedule(next.get());
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(next.get()));
  CHECK(m.getState() == Lift::Mid);
  CommandScheduler::schedule(prev.get());
  CommandScheduler::run();
  CHECK(m.getState() == Lift::Low);
  m.setPosition(Lift::High);
  CommandScheduler::schedule(cycle.get());
  CommandScheduler::run();
  CHECK(m.getState() == Lift::Down);

  // Disable: the scheduler cancels the running command and calls
  // onDisabled(). MultiPositionMechanism keeps its state, and pushes nothing
  // while disabled.
  CommandScheduler::schedule(to_high.get());
  CommandScheduler::run();
  const std::size_t before = rig.pushed.size();
  mclib::test::setCompetitionStatus(1);
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(to_high.get()));
  CHECK(m.getState() == Lift::High);
  CHECK_EQ(sz(rig.pushed.size()), sz(before));

  // Re-enabled: the kept state is pushed again on the first tick.
  mclib::test::setCompetitionStatus(0);
  CommandScheduler::run();
  CHECK_EQ(rig.last(), 40.0);

  CommandScheduler::unregisterSubsystem(&m);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  lookups();
  stepping();
  unlisted_state();
  degenerate_tables();
  sink_every_tick();
  commands_and_disable();
  return mclib::test::summary("multi_position_mechanism");
}
