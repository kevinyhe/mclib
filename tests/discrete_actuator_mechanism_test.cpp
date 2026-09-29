// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// DiscreteActuatorMechanism: the decode table, rows dropped for the wrong
// width, undecodable states, polarity, stepping, the command factories, a new
// state arriving while a timed command runs, and disable.
//
// Raw actuators are lambdas that record every value. The pneumatic
// constructor runs against tests/support/host_pneumatic.cpp.

#include "mclib/command/commandScheduler.h"
#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/discrete_actuator_mechanism.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "support/host_pneumatic.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <type_traits>
#include <vector>

using mclib::mechanism::DiscreteActuatorMechanism;
using mclib::mechanism::DiscreteActuatorMechanismConfig;

namespace {

std::uint32_t g_fake_ms = 0;

enum class Tilt { Low, Mid, High, Bad, Unlisted };

using TiltMech = DiscreteActuatorMechanism<Tilt>;

static_assert(!std::is_copy_constructible_v<TiltMech>);
static_assert(!std::is_move_constructible_v<TiltMech>);

double sz(std::size_t n) { return static_cast<double>(n); }

/// N recording actuators.
struct Recorder {
  std::vector<std::vector<bool>> writes;

  explicit Recorder(std::size_t channels) : writes(channels) {}

  std::vector<TiltMech::Actuator> actuators() {
    std::vector<TiltMech::Actuator> out;
    for (std::size_t i = 0; i < writes.size(); ++i) {
      out.push_back([this, i](bool value) { writes[i].push_back(value); });
    }
    return out;
  }

  std::size_t count(std::size_t i) const { return writes[i].size(); }
  bool last(std::size_t i) const { return !writes[i].empty() && writes[i].back(); }
  std::size_t total() const {
    std::size_t n = 0;
    for (const auto& w : writes) n += w.size();
    return n;
  }
};

TiltMech::Table table() {
  return {{Tilt::Low, {false, false}},
          {Tilt::Mid, {true, false}},
          {Tilt::High, {true, true}},
          {Tilt::Bad, {true}}};  // wrong width: dropped
}

void construction() {
  std::printf("-- construction, dropped rows, apply_on_construct\n");
  {
    Recorder rec(2);
    TiltMech m(Tilt::Mid, table(), rec.actuators());
    CHECK_EQ(sz(m.stateCount()), 3.0);
    CHECK_EQ(sz(m.actuatorCount()), 2.0);
    CHECK_EQ(sz(m.droppedRowCount()), 1.0);
    CHECK(!m.isDecodable(Tilt::Bad));
    // Written once from the constructor.
    CHECK_EQ(sz(rec.count(0)), 1.0);
    CHECK_EQ(sz(rec.count(1)), 1.0);
    CHECK(rec.last(0) && !rec.last(1));
  }
  {
    Recorder rec(2);
    DiscreteActuatorMechanismConfig config;
    config.apply_on_construct = false;
    TiltMech m(Tilt::High, table(), rec.actuators(), config);
    CHECK_EQ(sz(rec.total()), 0.0);
    m.apply();
    CHECK(rec.last(0) && rec.last(1));
  }
  {
    // Undecodable initial state: nothing is written, at construction or on a
    // tick, until a decodable state is set.
    Recorder rec(2);
    TiltMech m(Tilt::Unlisted, table(), rec.actuators());
    CHECK_EQ(sz(rec.total()), 0.0);
    m.runPeriodic();
    m.apply();
    CHECK_EQ(sz(rec.total()), 0.0);
    CHECK(m.currentIndex() == TiltMech::kNoState);
    CHECK(m.currentCombination().empty());
    // The first step resyncs to row 0.
    m.previous();
    CHECK(m.getDiscreteState() == Tilt::Low);
    CHECK_EQ(sz(rec.count(0)), 1.0);
  }
  {
    // Empty table: every state is undecodable and nothing is ever written.
    Recorder rec(1);
    TiltMech m(Tilt::Low, TiltMech::Table{}, rec.actuators());
    CHECK(m.empty());
    CHECK(!m.setDiscreteState(Tilt::Low));
    m.next();
    m.cycle();
    m.runPeriodic();
    CHECK_EQ(sz(rec.total()), 0.0);
  }
}

void lookups_and_polarity() {
  std::printf("-- lookups, out of range indices, polarity, null actuators\n");
  Recorder rec(3);
  DiscreteActuatorMechanismConfig config;
  config.inverted = {false, true};  // column 2 is past the end: not inverted
  std::vector<TiltMech::Actuator> acts = rec.actuators();
  acts[1] = nullptr;  // null keeps its column and is never called
  TiltMech m(Tilt::Low,
             {{Tilt::Low, {false, false, false}}, {Tilt::High, {true, true, true}}},
             acts, config);

  CHECK(m.stateAt(1) != nullptr && *m.stateAt(1) == Tilt::High);
  CHECK(m.stateAt(2) == nullptr);
  CHECK(m.stateAt(TiltMech::kNoState) == nullptr);
  CHECK(m.indexOf(Tilt::Mid) == TiltMech::kNoState);
  CHECK(m.combinationFor(Tilt::Mid).empty());
  CHECK(m.combinationFor(Tilt::High) == std::vector<bool>({true, true, true}));
  CHECK(!m.isInverted(0));
  CHECK(m.isInverted(1));
  CHECK(!m.isInverted(2));
  CHECK(!m.isInverted(99));
  CHECK(m.rawValue(1, true) == false);
  CHECK(m.rawValue(99, true) == true);

  CHECK(m.setDiscreteState(Tilt::High));
  CHECK(rec.last(0));
  CHECK(rec.last(2));
  CHECK_EQ(sz(rec.count(1)), 0.0);
}

void state_changes() {
  std::printf("-- setDiscreteState and stepping\n");
  Recorder rec(2);
  TiltMech m(Tilt::Low, table(), rec.actuators());

  // Accepted states write immediately, not on the next tick.
  CHECK(m.setDiscreteState(Tilt::High));
  CHECK(rec.last(0) && rec.last(1));

  // Refused states change nothing and write nothing.
  const std::size_t writes = rec.total();
  CHECK(!m.setDiscreteState(Tilt::Bad));
  CHECK(!m.setDiscreteState(Tilt::Unlisted));
  CHECK(m.getDiscreteState() == Tilt::High);
  CHECK_EQ(sz(rec.total()), sz(writes));

  // periodic() rewrites the cached state every tick.
  m.runPeriodic();
  CHECK_EQ(sz(rec.total()), sz(writes + 2));

  m.next();
  CHECK(m.getDiscreteState() == Tilt::High);
  m.previous();
  CHECK(m.getDiscreteState() == Tilt::Mid);
  CHECK(rec.last(0) && !rec.last(1));
  m.previous();
  m.previous();
  CHECK(m.getDiscreteState() == Tilt::Low);
  m.setDiscreteState(Tilt::High);
  m.cycle();
  CHECK(m.getDiscreteState() == Tilt::Low);
  CHECK(!rec.last(0) && !rec.last(1));
}

void pneumatics() {
  std::printf("-- pneumatic bank\n");
  namespace hp = mclib::test::host_pneumatic;
  hp::reset();
  auto front = std::make_shared<mclib::device::Pneumatic>('A');
  // Wired backwards: the device inverts, the mechanism must not.
  auto back = std::make_shared<mclib::device::Pneumatic>('B', false, false);
  TiltMech m(Tilt::Low, table(),
             std::vector<std::shared_ptr<mclib::device::Pneumatic>>{front, back, nullptr});
  // Three columns, so every two-wide row is dropped.
  CHECK_EQ(sz(m.droppedRowCount()), 4.0);
  CHECK(m.empty());

  TiltMech tilt(Tilt::Low, table(),
                std::vector<std::shared_ptr<mclib::device::Pneumatic>>{front, back});
  CHECK(tilt.setDiscreteState(Tilt::Mid));
  CHECK(front->get_value());
  CHECK(!back->get_value());
  CHECK(hp::pin['A']);
  CHECK(hp::pin['B']);  // retracted, and wired backwards: pin high
  CHECK(tilt.setDiscreteState(Tilt::High));
  CHECK(back->get_value());
  CHECK(!hp::pin['B']);
}

void commands_and_disable() {
  std::printf("-- commands, a new state mid-command, and disable\n");
  mclib::test::setCompetitionStatus(0);
  Recorder rec(2);
  TiltMech m(Tilt::Low, table(), rec.actuators());
  m.setName("tilt");
  m.setDefaultCommand(m.idleCommand());
  m.registerSelf();

  std::unique_ptr<Command> to_high = m.makeDiscreteStateCommand(Tilt::High);
  CommandScheduler::schedule(to_high.get());
  CHECK(m.getDiscreteState() == Tilt::High);
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(to_high.get()));

  std::unique_ptr<Command> to_bad = m.makeDiscreteStateCommand(Tilt::Bad);
  CommandScheduler::schedule(to_bad.get());
  CommandScheduler::run();
  CHECK(m.getDiscreteState() == Tilt::High);

  std::unique_ptr<Command> next = m.makeNextCommand();
  std::unique_ptr<Command> prev = m.makePreviousCommand();
  std::unique_ptr<Command> cycle = m.makeCycleCommand();
  CommandScheduler::schedule(prev.get());
  CommandScheduler::run();
  CHECK(m.getDiscreteState() == Tilt::Mid);
  CommandScheduler::schedule(next.get());
  CommandScheduler::run();
  CHECK(m.getDiscreteState() == Tilt::High);
  CommandScheduler::schedule(cycle.get());
  CommandScheduler::run();
  CHECK(m.getDiscreteState() == Tilt::Low);

  // Timed command: holds Mid for 100 ms, then finishes and leaves it.
  g_fake_ms = 1000;
  std::unique_ptr<Command> mid_for =
      m.makeDiscreteStateForCommand(Tilt::Mid, 100.0 * mclib::units::millisecond);
  CommandScheduler::schedule(mid_for.get());
  CHECK(m.getDiscreteState() == Tilt::Mid);
  int finished_on = -1;
  for (int i = 1; i <= 30; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    if (!CommandScheduler::scheduled(mid_for.get())) {
      finished_on = i;
      break;
    }
  }
  std::printf("   100 ms hold at 10 ms/tick: finished on tick %d\n", finished_on);
  CHECK_EQ(static_cast<double>(finished_on), 10.0);
  CHECK(m.getDiscreteState() == Tilt::Mid);

  // An undecodable state gives a command that finishes at once and moves
  // nothing.
  std::unique_ptr<Command> bad_for =
      m.makeDiscreteStateForCommand(Tilt::Bad, 100.0 * mclib::units::millisecond);
  CommandScheduler::schedule(bad_for.get());
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(bad_for.get()));
  CHECK(m.getDiscreteState() == Tilt::Mid);

  // Mid-command: a one-shot to High interrupts the timed hold of Low. The
  // timed command must not drag the mechanism back to Low afterwards.
  std::unique_ptr<Command> low_for =
      m.makeDiscreteStateForCommand(Tilt::Low, 500.0 * mclib::units::millisecond);
  CommandScheduler::schedule(low_for.get());
  g_fake_ms += 10;
  CommandScheduler::run();
  CHECK(m.getDiscreteState() == Tilt::Low);
  CommandScheduler::schedule(to_high.get());
  CHECK(!CommandScheduler::scheduled(low_for.get()));
  for (int i = 0; i < 5; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
  }
  CHECK(m.getDiscreteState() == Tilt::High);
  CHECK(rec.last(0) && rec.last(1));

  // Disable. There is no onDisabled() override: the actuators keep the last
  // combination (solenoids hold), the state is kept, and nothing is written
  // while disabled.
  CommandScheduler::schedule(low_for.get());
  CHECK(m.getDiscreteState() == Tilt::Low);
  const std::size_t writes = rec.total();
  mclib::test::setCompetitionStatus(1);
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(low_for.get()));
  CHECK(m.getDiscreteState() == Tilt::Low);
  CHECK_EQ(sz(rec.total()), sz(writes));
  mclib::test::setCompetitionStatus(0);

  // A subsystem switched off with setEnabled(false): periodic() is skipped,
  // so the timed command moves nothing, but setDiscreteState() still writes.
  // Both are documented.
  m.setEnabled(false);
  const std::size_t before = rec.total();
  std::unique_ptr<Command> high_for =
      m.makeDiscreteStateForCommand(Tilt::High, 50.0 * mclib::units::millisecond);
  CommandScheduler::schedule(high_for.get());
  CommandScheduler::run();
  CHECK_EQ(sz(rec.total()), sz(before));
  CommandScheduler::cancel(high_for.get());
  CHECK(m.setDiscreteState(Tilt::Mid));
  CHECK_EQ(sz(rec.total()), sz(before + 2));
  m.setEnabled(true);

  CommandScheduler::unregisterSubsystem(&m);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  construction();
  lookups_and_polarity();
  state_changes();
  pneumatics();
  commands_and_disable();
  return mclib::test::summary("discrete_actuator_mechanism");
}
