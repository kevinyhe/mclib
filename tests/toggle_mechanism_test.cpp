// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// ToggleMechanism: the logical/raw split, when the actuator is written, null
// actuators, and the command factories through a real CommandScheduler on a
// mclib::time::ScopedClock.
//
// The actuator is a lambda that records every raw value it is handed. The two
// device::Pneumatic constructors are only exercised with null pointers: a real
// Pneumatic constructs a pros::adi::DigitalOut and cannot exist on the host.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/toggle_mechanism.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

using mclib::mechanism::ToggleMechanism;
using mclib::mechanism::ToggleMechanismConfig;

namespace {

std::uint32_t g_fake_ms = 0;

/// Every raw value the actuator was handed, in order.
struct Recorder {
  std::vector<bool> writes;

  ToggleMechanism::Actuator actuator() {
    return [this](bool value) { writes.push_back(value); };
  }

  bool last() const { return writes.empty() ? false : writes.back(); }
  std::size_t count() const { return writes.size(); }
};

/// End and forget a command the scheduler may still reference, before the
/// unique_ptr that owns it goes out of scope.
void retire(const std::unique_ptr<Command>& command) {
  CommandScheduler::endAndForget(command.get());
}

// ---------------------------------------------------------------------------
// 1. Construction: initial state, apply_on_construct, inversion.
// ---------------------------------------------------------------------------
void construction() {
  std::printf("-- construction\n");
  {
    Recorder rec;
    ToggleMechanism m(rec.actuator());
    CHECK(!m.isExtended());
    CHECK(!m.isInverted());
    // apply_on_construct defaults to true: exactly one write of the initial
    // raw value, so the hardware matches before the scheduler runs.
    CHECK_EQ(static_cast<double>(rec.count()), 1.0);
    CHECK(rec.last() == false);
  }
  {
    Recorder rec;
    ToggleMechanism m(rec.actuator(), {.initial_extended = true});
    CHECK(m.isExtended());
    CHECK_EQ(static_cast<double>(rec.count()), 1.0);
    CHECK(rec.last() == true);
  }
  {
    Recorder rec;
    ToggleMechanism m(rec.actuator(), {.initial_extended = true, .apply_on_construct = false});
    CHECK(m.isExtended());
    CHECK_EQ(static_cast<double>(rec.count()), 0.0);
  }
  {
    // Inverted: the actuator gets the negation, isExtended() stays logical.
    Recorder rec;
    ToggleMechanism m(rec.actuator(), {.initial_extended = true, .inverted = true});
    CHECK(m.isExtended());
    CHECK(m.isInverted());
    CHECK(m.rawValue(true) == false);
    CHECK(m.rawValue(false) == true);
    CHECK_EQ(static_cast<double>(rec.count()), 1.0);
    CHECK(rec.last() == false);

    m.retract();
    CHECK(!m.isExtended());
    CHECK(rec.last() == true);
  }
  {
    // Not inverted: raw is the logical value.
    Recorder rec;
    ToggleMechanism m(rec.actuator());
    CHECK(m.rawValue(true) == true);
    CHECK(m.rawValue(false) == false);
  }
}

// ---------------------------------------------------------------------------
// 2. set()/extend()/retract()/toggle() write right away, even when unchanged.
//    setState() does not write until periodic().
// ---------------------------------------------------------------------------
void whenTheActuatorIsWritten() {
  std::printf("-- when the actuator is written\n");
  Recorder rec;
  ToggleMechanism m(rec.actuator(), {.apply_on_construct = false});
  CHECK_EQ(static_cast<double>(rec.count()), 0.0);

  m.extend();
  CHECK(m.isExtended());
  CHECK_EQ(static_cast<double>(rec.count()), 1.0);
  CHECK(rec.last() == true);

  // Same value again: still a write. That is the documented resync path for
  // hardware that drifted.
  m.set(true);
  CHECK_EQ(static_cast<double>(rec.count()), 2.0);
  CHECK(rec.last() == true);

  m.toggle();
  CHECK(!m.isExtended());
  CHECK_EQ(static_cast<double>(rec.count()), 3.0);
  CHECK(rec.last() == false);

  m.retract();
  CHECK_EQ(static_cast<double>(rec.count()), 4.0);

  m.apply();
  CHECK_EQ(static_cast<double>(rec.count()), 5.0);
  CHECK(rec.last() == false);

  // The inherited setState() only caches. The actuator sees it on the next
  // periodic() tick, and every tick after that.
  m.setState(true);
  CHECK(m.isExtended());
  CHECK_EQ(static_cast<double>(rec.count()), 5.0);
  m.runPeriodic();
  CHECK_EQ(static_cast<double>(rec.count()), 6.0);
  CHECK(rec.last() == true);
  m.runPeriodic();
  m.runPeriodic();
  CHECK_EQ(static_cast<double>(rec.count()), 8.0);

  // A disabled subsystem skips periodic() but the direct mutators still write.
  m.setEnabled(false);
  m.runPeriodic();
  CHECK_EQ(static_cast<double>(rec.count()), 8.0);
  m.toggle();
  CHECK_EQ(static_cast<double>(rec.count()), 9.0);
  CHECK(rec.last() == false);
}

// ---------------------------------------------------------------------------
// 3. Null actuators and null pneumatics are ignored, never dereferenced.
// ---------------------------------------------------------------------------
void nullActuators() {
  std::printf("-- null actuators\n");
  {
    ToggleMechanism m(ToggleMechanism::Actuator{});
    m.toggle();
    m.apply();
    m.runPeriodic();
    CHECK(m.isExtended());
  }
  {
    ToggleMechanism m(std::shared_ptr<mclib::device::Pneumatic>{}, {.initial_extended = true});
    m.toggle();
    m.runPeriodic();
    CHECK(!m.isExtended());
  }
  {
    std::vector<std::shared_ptr<mclib::device::Pneumatic>> pneumatics = {nullptr, nullptr};
    ToggleMechanism m(pneumatics);
    m.extend();
    m.runPeriodic();
    CHECK(m.isExtended());
  }
}

// ---------------------------------------------------------------------------
// 4. Command factories through the scheduler.
// ---------------------------------------------------------------------------
void commands() {
  std::printf("-- commands through the scheduler\n");
  Recorder rec;
  ToggleMechanism m(rec.actuator());
  m.setName("toggle");
  m.setDefaultCommand(m.idleCommand());
  m.registerSelf();

  // One-shots: the state lands as soon as the command is scheduled (an
  // InstantCommand runs its body from initialize()), and the command is gone
  // after one run().
  std::unique_ptr<Command> extend = m.makeExtendCommand();
  CommandScheduler::schedule(extend.get());
  CHECK(m.isExtended());
  CHECK(CommandScheduler::scheduled(extend.get()));
  CommandScheduler::run();
  CHECK(!CommandScheduler::scheduled(extend.get()));
  CHECK(m.isExtended());

  // With an idle default the state survives every following tick, and each
  // tick re-writes it.
  const std::size_t before = rec.count();
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(m.isExtended());
  CHECK_EQ(static_cast<double>(rec.count()), static_cast<double>(before + 2));
  CHECK(rec.last() == true);

  std::unique_ptr<Command> retract = m.makeRetractCommand();
  CommandScheduler::schedule(retract.get());
  CommandScheduler::run();
  CHECK(!m.isExtended());
  CHECK(!CommandScheduler::scheduled(retract.get()));

  std::unique_ptr<Command> toggle = m.makeToggleCommand();
  CommandScheduler::schedule(toggle.get());
  CommandScheduler::run();
  CHECK(m.isExtended());
  CHECK(!CommandScheduler::scheduled(toggle.get()));

  std::unique_ptr<Command> set_false = m.makeSetCommand(false);
  CommandScheduler::schedule(set_false.get());
  CommandScheduler::run();
  CHECK(!m.isExtended());

  // Timed: holds for the duration, finishes, and does NOT restore the old
  // state.
  g_fake_ms = 1000;
  std::unique_ptr<Command> pulse = m.makeExtendForCommand(100.0 * mclib::units::millisecond);
  CommandScheduler::schedule(pulse.get());
  CHECK(m.isExtended());
  int finished_on = -1;
  for (int i = 1; i <= 30; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
    if (!CommandScheduler::scheduled(pulse.get())) {
      finished_on = i;
      break;
    }
  }
  // The finish check runs after execute() on the tick where 100 ms has
  // elapsed: tick 10.
  std::printf("   100 ms pulse at 10 ms/tick: finished on tick %d\n", finished_on);
  CHECK_EQ(static_cast<double>(finished_on), 10.0);
  CHECK(m.isExtended());
  CommandScheduler::run();
  CHECK(m.isExtended());

  std::unique_ptr<Command> drop = m.makeRetractForCommand(50.0 * mclib::units::millisecond);
  CommandScheduler::schedule(drop.get());
  CHECK(!m.isExtended());
  for (int i = 0; i < 10; ++i) {
    g_fake_ms += 10;
    CommandScheduler::run();
  }
  CHECK(!CommandScheduler::scheduled(drop.get()));
  CHECK(!m.isExtended());

  retire(extend);
  retire(retract);
  retire(toggle);
  retire(set_false);
  retire(pulse);
  retire(drop);
  CommandScheduler::unregisterSubsystem(&m);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  construction();
  whenTheActuatorIsWritten();
  nullActuators();
  commands();
  return mclib::test::summary("toggle_mechanism");
}
