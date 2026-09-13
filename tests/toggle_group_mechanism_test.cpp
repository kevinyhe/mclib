// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// ToggleGroupMechanism: per-channel state, per-channel inversion, out of range
// indices, and the command factories through a real CommandScheduler on a
// mclib::time::ScopedClock. Includes the README's warning about a default
// command that is not idleCommand(), reproduced tick for tick.
//
// Every actuator is a lambda that records what it was handed. The pneumatic
// constructor is only exercised with null entries: a real device::Pneumatic
// constructs a pros::adi::DigitalOut and cannot exist on the host.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/toggle_group_mechanism.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <type_traits>
#include <vector>

using mclib::mechanism::ToggleGroupMechanism;
using mclib::mechanism::ToggleGroupMechanismConfig;

namespace {

std::uint32_t g_fake_ms = 0;

enum class Side : std::size_t { Left = 0, Right = 1 };

/// N recording actuators.
struct Recorder {
  std::vector<std::vector<bool>> writes;

  explicit Recorder(std::size_t channels) : writes(channels) {}

  std::vector<ToggleGroupMechanism::Actuator> actuators() {
    std::vector<ToggleGroupMechanism::Actuator> out;
    for (std::size_t i = 0; i < writes.size(); ++i) {
      out.push_back([this, i](bool value) { writes[i].push_back(value); });
    }
    return out;
  }

  std::size_t count(std::size_t i) const { return writes[i].size(); }
  bool last(std::size_t i) const { return writes[i].empty() ? false : writes[i].back(); }
};

void retire(const std::unique_ptr<Command>& command) {
  CommandScheduler::endAndForget(command.get());
}

// The class promises copy and move are deleted.
static_assert(!std::is_copy_constructible_v<ToggleGroupMechanism>);
static_assert(!std::is_copy_assignable_v<ToggleGroupMechanism>);
static_assert(!std::is_move_constructible_v<ToggleGroupMechanism>);
static_assert(!std::is_move_assignable_v<ToggleGroupMechanism>);

// ---------------------------------------------------------------------------
// 1. Construction: channel count, initial states, apply_on_construct.
// ---------------------------------------------------------------------------
void construction() {
  std::printf("-- construction\n");
  {
    Recorder rec(3);
    ToggleGroupMechanism g(rec.actuators());
    CHECK_EQ(static_cast<double>(g.count()), 3.0);
    for (std::size_t i = 0; i < 3; ++i) {
      CHECK(!g.get(i));
      CHECK_EQ(static_cast<double>(rec.count(i)), 1.0);
      CHECK(rec.last(i) == false);
    }
    CHECK(!g.anySet());
    CHECK(!g.allSet());
    CHECK_EQ(static_cast<double>(g.setCount()), 0.0);
  }
  {
    Recorder rec(2);
    ToggleGroupMechanism g(rec.actuators(), {.initial_extended = true});
    CHECK(g.get(0) && g.get(1));
    CHECK(rec.last(0) == true && rec.last(1) == true);
    CHECK(g.allSet());
  }
  {
    // A short initial_states: the rest fall back to initial_extended.
    Recorder rec(3);
    ToggleGroupMechanism g(rec.actuators(), {.initial_extended = false, .initial_states = {true}});
    CHECK(g.get(0));
    CHECK(!g.get(1));
    CHECK(!g.get(2));
    CHECK_EQ(static_cast<double>(g.setCount()), 1.0);
  }
  {
    // A long initial_states: entries past the channel count are ignored.
    Recorder rec(2);
    ToggleGroupMechanism g(rec.actuators(), {.initial_states = {false, true, true, true}});
    CHECK_EQ(static_cast<double>(g.count()), 2.0);
    CHECK(!g.get(0));
    CHECK(g.get(1));
    CHECK(g.getState().size() == 2);
  }
  {
    Recorder rec(2);
    ToggleGroupMechanism g(rec.actuators(), {.initial_extended = true, .apply_on_construct = false});
    CHECK(g.get(0));
    CHECK_EQ(static_cast<double>(rec.count(0)), 0.0);
    CHECK_EQ(static_cast<double>(rec.count(1)), 0.0);
  }
  {
    // No channels at all.
    ToggleGroupMechanism g(std::vector<ToggleGroupMechanism::Actuator>{});
    CHECK_EQ(static_cast<double>(g.count()), 0.0);
    CHECK(g.allSet());
    CHECK(!g.anySet());
    g.set(0, true);
    g.setAll(true);
    g.toggleAll();
    g.runPeriodic();
    CHECK(!g.get(0));
  }
}

// ---------------------------------------------------------------------------
// 2. Per-channel inversion, the README's raw_sides example.
// ---------------------------------------------------------------------------
void inversion() {
  std::printf("-- per channel inversion\n");
  Recorder rec(2);
  ToggleGroupMechanism g(rec.actuators(),
                         {.initial_states = {false, true}, .inverted = {false, true}});
  CHECK(!g.isInverted(0));
  CHECK(g.isInverted(1));
  CHECK(!g.isInverted(2));  // past the end
  CHECK(!g.isInverted(Side::Left));
  CHECK(g.isInverted(Side::Right));

  CHECK(g.rawValue(0, true) == true);
  CHECK(g.rawValue(1, true) == false);
  CHECK(g.rawValue(1, false) == true);
  CHECK(g.rawValue(7, true) == true);  // out of range is not inverted

  // Channel 1 starts logically extended, so its actuator got false.
  CHECK(g.get(1));
  CHECK(rec.last(1) == false);
  CHECK(!g.get(0));
  CHECK(rec.last(0) == false);

  g.set(1, false);
  CHECK(!g.get(1));
  CHECK(rec.last(1) == true);

  // A short inverted vector: channel 2 of 3 is treated as not inverted.
  Recorder rec3(3);
  ToggleGroupMechanism g3(rec3.actuators(), {.inverted = {true}});
  g3.setAll(true);
  CHECK(rec3.last(0) == false);
  CHECK(rec3.last(1) == true);
  CHECK(rec3.last(2) == true);
}

// ---------------------------------------------------------------------------
// 3. The mutators and readers, index and enum forms.
// ---------------------------------------------------------------------------
void mutators() {
  std::printf("-- mutators\n");
  Recorder rec(2);
  ToggleGroupMechanism g(rec.actuators(), {.apply_on_construct = false});

  g.set(Side::Left, true);
  CHECK(g.get(Side::Left));
  CHECK(!g.get(Side::Right));
  CHECK(g.get(0));
  CHECK(g.anySet());
  CHECK(!g.allSet());
  CHECK_EQ(static_cast<double>(g.setCount()), 1.0);
  // set() writes every actuator, not only the one that changed.
  CHECK_EQ(static_cast<double>(rec.count(0)), 1.0);
  CHECK_EQ(static_cast<double>(rec.count(1)), 1.0);
  CHECK(rec.last(0) == true);
  CHECK(rec.last(1) == false);

  g.toggle(Side::Right);
  CHECK(g.get(1));
  CHECK(g.allSet());
  CHECK_EQ(static_cast<double>(g.setCount()), 2.0);

  g.setAll(false);
  CHECK(!g.anySet());
  CHECK_EQ(static_cast<double>(g.setCount()), 0.0);

  g.set(0, true);
  g.toggleAll();
  CHECK(!g.get(0));
  CHECK(g.get(1));

  // Same value again still writes: the resync path.
  const std::size_t before = rec.count(1);
  g.set(1, true);
  CHECK_EQ(static_cast<double>(rec.count(1)), static_cast<double>(before + 1));

  // apply() writes without changing anything; periodic() does the same each
  // tick.
  g.apply();
  CHECK_EQ(static_cast<double>(rec.count(1)), static_cast<double>(before + 2));
  g.runPeriodic();
  g.runPeriodic();
  CHECK_EQ(static_cast<double>(rec.count(1)), static_cast<double>(before + 4));
  CHECK(rec.last(1) == true);
  CHECK(rec.last(0) == false);

  // An empty actuator entry is skipped, and the channel still holds state.
  std::vector<ToggleGroupMechanism::Actuator> holey = rec.actuators();
  holey[1] = ToggleGroupMechanism::Actuator{};
  ToggleGroupMechanism h(std::move(holey));
  h.set(1, true);
  h.runPeriodic();
  CHECK(h.get(1));
  CHECK_EQ(static_cast<double>(h.count()), 2.0);

  // Null pneumatics are skipped the same way.
  ToggleGroupMechanism p(std::vector<std::shared_ptr<mclib::device::Pneumatic>>{nullptr, nullptr});
  p.setAll(true);
  p.runPeriodic();
  CHECK(p.allSet());
  CHECK_EQ(static_cast<double>(p.count()), 2.0);
}

// ---------------------------------------------------------------------------
// 4. Out of range indices are ignored and read back false.
// ---------------------------------------------------------------------------
void outOfRange() {
  std::printf("-- out of range indices\n");
  Recorder rec(2);
  ToggleGroupMechanism g(rec.actuators(), {.apply_on_construct = false});

  g.set(2, true);
  g.toggle(2);
  g.set(static_cast<std::size_t>(-1), true);
  CHECK(!g.get(2));
  CHECK(!g.get(static_cast<std::size_t>(-1)));
  CHECK(!g.anySet());
  CHECK_EQ(static_cast<double>(rec.count(0)), 0.0);
  CHECK_EQ(static_cast<double>(rec.count(1)), 0.0);
  CHECK(g.getState().size() == 2);

  enum class Bad : std::size_t { Nope = 9 };
  g.set(Bad::Nope, true);
  CHECK(!g.get(Bad::Nope));
  CHECK(!g.isInverted(Bad::Nope));
  CHECK(!g.anySet());
}

// ---------------------------------------------------------------------------
// 5. Commands through the scheduler.
// ---------------------------------------------------------------------------
void commands() {
  std::printf("-- commands through the scheduler\n");
  Recorder rec(2);
  ToggleGroupMechanism g(rec.actuators());
  g.setName("sides");
  g.setDefaultCommand(g.idleCommand());
  g.registerSelf();

  // One-shots: state lands at schedule() (InstantCommand runs from
  // initialize()), the command is gone after one run(), and the state stays.
  std::unique_ptr<Command> left_on = g.makeSetCommand(Side::Left, true);
  CommandScheduler::schedule(left_on.get());
  CHECK(g.get(0));
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(left_on.get()));
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(g.get(0));
  CHECK(!g.get(1));

  std::unique_ptr<Command> right_toggle = g.makeToggleCommand(Side::Right);
  CommandScheduler::schedule(right_toggle.get());
  CommandScheduler::run();
  CHECK(g.get(1));
  CHECK(!CommandScheduler::scheduled(right_toggle.get()));

  std::unique_ptr<Command> all_off = g.makeSetAllCommand(false);
  CommandScheduler::schedule(all_off.get());
  CommandScheduler::run();
  CHECK(!g.anySet());

  std::unique_ptr<Command> all_toggle = g.makeToggleAllCommand();
  CommandScheduler::schedule(all_toggle.get());
  CommandScheduler::run();
  CHECK(g.allSet());
  CHECK(!CommandScheduler::scheduled(all_toggle.get()));

  // Out of range through a command: also ignored.
  std::unique_ptr<Command> bad = g.makeSetCommand(5, false);
  CommandScheduler::schedule(bad.get());
  CommandScheduler::run();
  CHECK(g.allSet());

  // Timed on one channel: holds it, leaves the other channel alone, finishes
  // when the duration is up, and does not restore.
  g.setAll(false);
  g_fake_ms = 1000;
  std::unique_ptr<Command> left_pulse =
      g.makeSetForCommand(Side::Left, true, 100.0 * mclib::units::millisecond);
  CommandScheduler::schedule(left_pulse.get());
  CHECK(g.get(0));
  CHECK(!g.get(1));
  int finished_on = -1;
  for (int i = 1; i <= 30; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    if (!CommandScheduler::scheduled(left_pulse.get())) {
      finished_on = i;
      break;
    }
    CHECK(g.get(0));
    CHECK(!g.get(1));
  }
  std::printf("   100 ms pulse at 10 ms/tick: finished on tick %d\n", finished_on);
  CHECK_EQ(static_cast<double>(finished_on), 10.0);
  CHECK(g.get(0));
  CHECK(!g.get(1));
  CommandScheduler::run();
  CHECK(g.get(0));

  std::unique_ptr<Command> all_pulse = g.makeSetAllForCommand(true, 50.0 * mclib::units::millisecond);
  CommandScheduler::schedule(all_pulse.get());
  CHECK(g.allSet());
  for (int i = 0; i < 6; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
  }
  CHECK(!CommandScheduler::scheduled(all_pulse.get()));
  CHECK(g.allSet());

  // One requirement for the whole group: a second timed command interrupts the
  // first instead of running beside it.
  g.setAll(false);
  std::unique_ptr<Command> hold_left =
      g.makeSetForCommand(Side::Left, true, 1000.0 * mclib::units::millisecond);
  std::unique_ptr<Command> hold_right =
      g.makeSetForCommand(Side::Right, true, 1000.0 * mclib::units::millisecond);
  CommandScheduler::schedule(hold_left.get());
  CommandScheduler::run();
  CHECK(CommandScheduler::scheduled(hold_left.get()));
  CommandScheduler::schedule(hold_right.get());
  CHECK(!CommandScheduler::scheduled(hold_left.get()));
  CHECK(CommandScheduler::scheduled(hold_right.get()));
  std::optional<Command*> owner = CommandScheduler::getRequiring(&g);
  CHECK(owner.has_value() && *owner == hold_right.get());
  // The interrupted command left its channel where it was; the new one only
  // touched its own.
  CHECK(g.get(0));
  CHECK(g.get(1));

  retire(left_on);
  retire(right_toggle);
  retire(all_off);
  retire(all_toggle);
  retire(bad);
  retire(left_pulse);
  retire(all_pulse);
  retire(hold_left);
  retire(hold_right);
  CommandScheduler::unregisterSubsystem(&g);
}

// ---------------------------------------------------------------------------
// 6. The README's warning: a non-idle default undoes every set one tick later.
// ---------------------------------------------------------------------------
void defaultCommandMustBeIdle() {
  std::printf("-- default command must be idleCommand()\n");
  Recorder rec(2);
  ToggleGroupMechanism g(rec.actuators());
  g.setName("sides");
  // The tempting "retract everything" default.
  g.setDefaultCommand(g.run([&g]() { g.setAll(false); }));
  g.registerSelf();
  CommandScheduler::run();  // default scheduled

  std::unique_ptr<Command> left_on = g.makeSetCommand(0, true);
  CommandScheduler::schedule(left_on.get());  // interrupts the default, sets
  CHECK(g.get(0));
  CommandScheduler::run();  // one-shot finishes, default rescheduled
  CHECK(g.get(0));
  CommandScheduler::run();  // default's execute() retracts it again
  CHECK(!g.get(0));

  retire(left_on);
  CommandScheduler::unregisterSubsystem(&g);

  // Same sequence with idleCommand(): the set sticks.
  Recorder rec2(2);
  ToggleGroupMechanism idle(rec2.actuators());
  idle.setName("sides");
  idle.setDefaultCommand(idle.idleCommand());
  idle.registerSelf();
  CommandScheduler::run();

  std::unique_ptr<Command> on = idle.makeSetCommand(0, true);
  CommandScheduler::schedule(on.get());
  CommandScheduler::run();
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(idle.get(0));

  retire(on);
  CommandScheduler::unregisterSubsystem(&idle);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  construction();
  inversion();
  mutators();
  outOfRange();
  commands();
  defaultCommandMustBeIdle();
  return mclib::test::summary("toggle_group_mechanism");
}
