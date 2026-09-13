// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// makeWaitForTriggerCommand() must not destroy a latch it did not set.
//
// disarmUntilReset() is what a driver override calls: it latches the mechanism
// and drops the edge, so the still-true condition cannot reapply the action the
// driver just undid. Arming clears both. The command's initialize() used to arm
// unconditionally, which threw that away and let the condition re-fire with no
// false->true edge behind it.

#include "mclib/command/commandScheduler.h"
#include "mclib/mechanism/auto_trigger_mechanism.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>

using mclib::mechanism::AutoTriggerConfig;
using mclib::mechanism::AutoTriggerMechanism;

namespace {

std::uint32_t g_fake_ms = 0;

/// One mechanism plus the sensor reading and action counter it is wired to.
struct Rig {
  bool condition = false;
  int fires = 0;
  AutoTriggerMechanism mechanism;

  explicit Rig(AutoTriggerConfig config)
      : mechanism([this]() { return condition; },
                  [this]() { ++fires; },
                  config) {}
};

/// Advance the clock 10 ms and give the command one execute()/isFinished() pass.
bool tick(Command& command) {
  g_fake_ms += 10;
  command.execute();
  return command.isFinished();
}

// ---------------------------------------------------------------------------
void testLatchSurvivesWaitCommand() {
  std::printf("-- latch survives makeWaitForTriggerCommand\n");

  AutoTriggerConfig config;
  config.debounce_ms = 0.0;
  Rig rig(config);

  CHECK(rig.mechanism.isArmed());

  // The condition comes true and the action fires once, the normal path.
  rig.condition = true;
  g_fake_ms += 10;
  rig.mechanism.poll();
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);
  CHECK(rig.mechanism.isLatched());

  // The driver overrides the result by hand and says "do not reapply this".
  rig.mechanism.disarmUntilReset();
  CHECK(rig.mechanism.isLatched());

  // A routine now waits for the next trigger. The condition never went false,
  // so there is no fresh edge and nothing should fire.
  auto command = rig.mechanism.makeWaitForTriggerCommand(500.0);
  command->initialize();

  CHECK(rig.mechanism.isLatched());

  bool finished = false;
  for (int i = 0; i < 60 && !finished; ++i) {
    finished = tick(*command);
  }

  CHECK_EQ(static_cast<double>(rig.fires), 1.0);
  CHECK(rig.mechanism.isLatched());
  // It ends by timing out, not by firing.
  CHECK(finished);

  command->end(false);

  // It was armed on entry, so the command leaves it armed.
  CHECK(rig.mechanism.isArmed());
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);
}

// ---------------------------------------------------------------------------
void testFreshEdgeStillFires() {
  std::printf("-- a real edge still fires while the command waits\n");

  AutoTriggerConfig config;
  config.debounce_ms = 0.0;
  Rig rig(config);

  rig.condition = true;
  g_fake_ms += 10;
  rig.mechanism.poll();
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);

  rig.mechanism.disarmUntilReset();

  auto command = rig.mechanism.makeWaitForTriggerCommand(2000.0);
  command->initialize();

  // The condition goes away: the latch clears and the edge is armed again.
  rig.condition = false;
  CHECK(!tick(*command));
  CHECK(!rig.mechanism.isLatched());
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);

  // And comes back: a genuine false->true edge, which fires and finishes.
  rig.condition = true;
  CHECK(tick(*command));
  CHECK_EQ(static_cast<double>(rig.fires), 2.0);

  command->end(false);
  CHECK(rig.mechanism.isArmed());
}

// ---------------------------------------------------------------------------
void testDisarmedMechanismIsStillArmedAndRestored() {
  std::printf("-- a disarmed mechanism is armed for the wait and put back\n");

  AutoTriggerConfig config;
  config.debounce_ms = 0.0;
  config.enabled_on_construct = false;
  Rig rig(config);

  CHECK(!rig.mechanism.isArmed());

  auto command = rig.mechanism.makeWaitForTriggerCommand(2000.0);
  command->initialize();
  CHECK(rig.mechanism.isArmed());

  rig.condition = true;
  CHECK(tick(*command));
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);

  command->end(false);
  CHECK(!rig.mechanism.isArmed());
}

// ---------------------------------------------------------------------------
/// Put the mechanism in the state a driver override leaves: armed, latched, no
/// re-arm condition, and a trigger condition that is still true. Nothing can
/// fire from here until the condition goes away.
void latchWhileConditionHolds(Rig& rig) {
  rig.condition = true;
  g_fake_ms += 10;
  rig.mechanism.poll();
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);

  rig.mechanism.disarmUntilReset();
  CHECK(rig.mechanism.isLatched());
  CHECK(rig.mechanism.isArmed());
}

void testLatchedCommandEndsOnTheDefaultTimeout() {
  std::printf("-- a latched mechanism gives up on the default timeout\n");

  AutoTriggerConfig config;
  config.debounce_ms = 0.0;
  Rig rig(config);

  latchWhileConditionHolds(rig);

  // The no-argument overload. Its default used to be "wait forever", which in
  // this state meant a routine step hung for the rest of the match. It is now a
  // finite kDefaultWaitForTriggerTimeoutMs, so the step gives up and moves on.
  auto command = rig.mechanism.makeWaitForTriggerCommand();
  command->initialize();

  const std::uint32_t start_ms = g_fake_ms;

  bool finished = false;
  int ticks = 0;
  for (; ticks < 2000 && !finished; ++ticks) {
    finished = tick(*command);
  }

  CHECK(finished);
  // It ended by timing out, not by firing: the latch is untouched.
  CHECK(rig.mechanism.isLatched());
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);

  const double elapsed_ms = static_cast<double>(g_fake_ms - start_ms);
  CHECK(elapsed_ms >= mclib::mechanism::kDefaultWaitForTriggerTimeoutMs);
  CHECK(elapsed_ms < mclib::mechanism::kDefaultWaitForTriggerTimeoutMs + 100.0);

  command->end(false);
}

void testWaitForeverIsStillAvailable() {
  std::printf("-- passing 0 still waits forever, and rearm() releases it\n");

  AutoTriggerConfig config;
  config.debounce_ms = 0.0;
  Rig rig(config);

  latchWhileConditionHolds(rig);

  // Explicit 0 is the old behaviour, kept for callers that mean it.
  auto command = rig.mechanism.makeWaitForTriggerCommand(0.0);
  command->initialize();

  bool finished = false;
  for (int i = 0; i < 2000 && !finished; ++i) {
    finished = tick(*command);
  }

  CHECK(!finished);
  CHECK(rig.mechanism.isLatched());
  CHECK_EQ(static_cast<double>(rig.fires), 1.0);

  // rearm() is the documented way out, and it works.
  rig.mechanism.rearm();
  rig.condition = false;
  CHECK(!tick(*command));
  rig.condition = true;
  CHECK(tick(*command));
  CHECK_EQ(static_cast<double>(rig.fires), 2.0);

  command->end(false);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });

  testLatchSurvivesWaitCommand();
  testFreshEdgeStillFires();
  testDisarmedMechanismIsStillArmedAndRestored();
  testLatchedCommandEndsOnTheDefaultTimeout();
  testWaitForeverIsStillAvailable();

  return mclib::test::summary("auto_trigger");
}
