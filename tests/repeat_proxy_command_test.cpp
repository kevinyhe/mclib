// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// RepeatCommand and ProxyCommand through the real CommandScheduler.
//
//   RepeatCommand: restarts the inner command each time it finishes, never
//   finishes itself, ends the inner command interrupted when cancelled or
//   displaced, and passes the inner command's requirements through.
//
//   ProxyCommand: schedules its target instead of running it, holds no
//   requirements, finishes once the target is no longer scheduled, and
//   cancels the target when the proxy is cancelled.

#include "mclib/command/commandScheduler.h"
#include "mclib/command/proxyCommand.h"
#include "mclib/command/repeatCommand.h"
#include "mclib/command/sequence.h"
#include "mclib/command/subsystem.h"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <memory>
#include <vector>

namespace {

/// Records every callback. Finishes after `finish_after` executes since its
/// last initialize(); 0 means it never finishes.
class TracerCommand : public Command {
public:
  explicit TracerCommand(int finish_after = 0, Subsystem* requirement = nullptr)
      : m_finish_after(finish_after), m_requirement(requirement) {}

  int initializes = 0;
  int executes = 0;
  int executes_since_init = 0;
  int ends = 0;
  int interrupted_ends = 0;
  bool last_end_interrupted = false;

  void initialize() override {
    ++initializes;
    executes_since_init = 0;
  }
  void execute() override {
    ++executes;
    ++executes_since_init;
  }
  bool isFinished() override {
    return m_finish_after > 0 && executes_since_init >= m_finish_after;
  }
  void end(bool interrupted) override {
    ++ends;
    if (interrupted) {
      ++interrupted_ends;
    }
    last_end_interrupted = interrupted;
  }
  std::vector<Subsystem*> getRequirements() override {
    if (m_requirement == nullptr) {
      return {};
    }
    return {m_requirement};
  }

private:
  int m_finish_after;
  Subsystem* m_requirement;
};

/// Leave the singleton scheduler empty for the next case.
void forget(Command* command) { CommandScheduler::endAndForget(command); }

double d(int value) { return static_cast<double>(value); }

// ---------------------------------------------------------------------------
// RepeatCommand
// ---------------------------------------------------------------------------

/// The inner command finishes every 2 executes. Over 6 ticks it runs 3 full
/// cycles: 3 normal ends and a fresh initialize after each one.
void repeatRestartsInnerCommand() {
  std::printf("-- RepeatCommand restarts the inner command\n");
  TracerCommand inner(2);
  auto repeat = inner.repeatedly();

  repeat->schedule();
  CHECK(repeat->scheduled());
  CHECK_EQ(d(inner.initializes), 1.0);

  for (int tick = 0; tick < 6; ++tick) {
    CommandScheduler::run();
  }
  CHECK_EQ(d(inner.executes), 6.0);
  CHECK_EQ(d(inner.ends), 3.0);
  CHECK_EQ(d(inner.interrupted_ends), 0.0);
  // One initialize at schedule time, one more after each finish.
  CHECK_EQ(d(inner.initializes), 4.0);
  // RepeatCommand never finishes on its own.
  CHECK(repeat->scheduled());
  // The inner command is driven by the repeat, never scheduled itself.
  CHECK(!inner.scheduled());

  forget(repeat.get());
}

/// An inner command that finishes on every execute restarts every tick
/// without looping inside a single tick.
void repeatInstantInnerOncePerTick() {
  std::printf("-- RepeatCommand with an inner command that finishes every tick\n");
  TracerCommand inner(1);
  auto repeat = inner.repeatedly();
  repeat->schedule();
  for (int tick = 0; tick < 5; ++tick) {
    CommandScheduler::run();
  }
  CHECK_EQ(d(inner.executes), 5.0);
  CHECK_EQ(d(inner.ends), 5.0);
  CHECK_EQ(d(inner.initializes), 6.0);
  forget(repeat.get());
}

/// Cancelling the repeat ends the inner command once, interrupted, and the
/// inner command is not restarted afterwards.
void repeatCancelInterruptsInner() {
  std::printf("-- cancelling RepeatCommand interrupts the inner command\n");
  TracerCommand inner(3);
  auto repeat = inner.repeatedly();
  repeat->schedule();
  CommandScheduler::run();

  repeat->cancel();
  CHECK(!repeat->scheduled());
  CHECK_EQ(d(inner.ends), 1.0);
  CHECK_EQ(d(inner.interrupted_ends), 1.0);
  CHECK(inner.last_end_interrupted);

  const int executes = inner.executes;
  const int initializes = inner.initializes;
  CommandScheduler::run();
  CommandScheduler::run();
  CHECK_EQ(d(inner.executes), d(executes));
  CHECK_EQ(d(inner.initializes), d(initializes));

  // A second cancel is a no-op: no second end().
  repeat->cancel();
  CHECK_EQ(d(inner.ends), 1.0);
  forget(repeat.get());
}

/// The repeat claims the inner command's subsystem, so a later command that
/// needs the same subsystem displaces it and the inner command ends
/// interrupted.
void repeatPassesRequirements() {
  std::printf("-- RepeatCommand passes the inner command's requirements\n");
  Subsystem arm;
  TracerCommand inner(2, &arm);
  auto repeat = inner.repeatedly();

  const std::vector<Subsystem*> requirements = repeat->getRequirements();
  CHECK_EQ(d(static_cast<int>(requirements.size())), 1.0);
  CHECK(!requirements.empty() && requirements[0] == &arm);

  repeat->schedule();
  CHECK(CommandScheduler::getRequiring(&arm).value_or(nullptr) == repeat.get());

  TracerCommand other(0, &arm);
  other.schedule();
  CHECK(!repeat->scheduled());
  CHECK(other.scheduled());
  CHECK_EQ(d(inner.interrupted_ends), 1.0);
  CHECK(CommandScheduler::getRequiring(&arm).value_or(nullptr) == &other);

  forget(&other);
  forget(repeat.get());
  CommandScheduler::forgetSubsystem(&arm);
}

/// Disabling the robot ends a running repeat interrupted, like any command.
void repeatEndsOnDisable() {
  std::printf("-- disable ends RepeatCommand\n");
  TracerCommand inner(2);
  auto repeat = inner.repeatedly();
  repeat->schedule();
  CommandScheduler::run();

  mclib::test::setCompetitionStatus(1);  // disabled
  CommandScheduler::run();
  mclib::test::setCompetitionStatus(0);
  CHECK(!repeat->scheduled());
  CHECK_EQ(d(inner.interrupted_ends), 1.0);
  forget(repeat.get());
}

// ---------------------------------------------------------------------------
// ProxyCommand
// ---------------------------------------------------------------------------

/// Scheduling the proxy schedules the target as its own command. The proxy
/// finishes on the tick after the target finishes.
void proxySchedulesTarget() {
  std::printf("-- ProxyCommand schedules its target\n");
  TracerCommand target(3);
  auto proxy = target.asProxy();

  proxy->schedule();
  CHECK(proxy->scheduled());
  CHECK(target.scheduled());
  CHECK_EQ(d(target.initializes), 1.0);

  int ticks = 0;
  while (proxy->scheduled() && ticks < 20) {
    CommandScheduler::run();
    ++ticks;
  }
  CHECK(!proxy->scheduled());
  CHECK(!target.scheduled());
  CHECK_EQ(d(target.executes), 3.0);
  CHECK_EQ(d(target.ends), 1.0);
  CHECK_EQ(d(target.interrupted_ends), 0.0);
  // Target finishes on tick 3; the proxy sees it on tick 4.
  std::printf("   target done after 3 executes, proxy done after %d ticks\n", ticks);
  CHECK_EQ(d(ticks), 4.0);

  forget(proxy.get());
  forget(&target);
}

/// Cancelling the proxy cancels the target.
void proxyCancelCancelsTarget() {
  std::printf("-- cancelling ProxyCommand cancels the target\n");
  TracerCommand target(0);
  auto proxy = target.asProxy();
  proxy->schedule();
  CommandScheduler::run();
  CHECK(target.scheduled());

  proxy->cancel();
  CHECK(!proxy->scheduled());
  CHECK(!target.scheduled());
  CHECK_EQ(d(target.ends), 1.0);
  CHECK(target.last_end_interrupted);

  forget(proxy.get());
  forget(&target);
}

/// Cancelling the proxy from inside another command's execute() goes through
/// the deferred cancel path and still reaches the target.
void proxyCancelFromRunLoop() {
  std::printf("-- cancelling ProxyCommand from inside the run loop\n");
  TracerCommand target(0);
  auto proxy = target.asProxy();
  proxy->schedule();

  class Canceller : public Command {
  public:
    Command* victim = nullptr;
    void execute() override { victim->cancel(); }
    bool isFinished() override { return true; }
  } canceller;
  canceller.victim = proxy.get();
  canceller.schedule();

  CommandScheduler::run();
  CHECK(!proxy->scheduled());
  CHECK(!target.scheduled());
  CHECK_EQ(d(target.interrupted_ends), 1.0);

  forget(&canceller);
  forget(proxy.get());
  forget(&target);
}

/// The target finishing on its own does not count as the proxy being
/// interrupted, and a target cancelled from outside ends the proxy normally.
void proxyTargetCancelledElsewhere() {
  std::printf("-- target cancelled directly ends the proxy\n");
  TracerCommand target(0);
  auto proxy = target.asProxy();
  proxy->schedule();
  CommandScheduler::run();

  target.cancel();
  CHECK(!target.scheduled());
  CHECK(proxy->scheduled());
  CommandScheduler::run();
  CHECK(!proxy->scheduled());
  CHECK_EQ(d(target.ends), 1.0);

  forget(proxy.get());
  forget(&target);
}

/// The proxy holds no requirements, so the subsystem is held by the target
/// alone and is free once the target is done, while the proxy still runs in
/// a sequence.
void proxyHoldsNoRequirements() {
  std::printf("-- ProxyCommand holds no requirements\n");
  Subsystem intake;
  TracerCommand target(2, &intake);
  auto proxy = target.asProxy();
  CHECK(proxy->getRequirements().empty());

  proxy->schedule();
  CHECK(CommandScheduler::getRequiring(&intake).value_or(nullptr) == &target);

  CommandScheduler::run();
  CommandScheduler::run();
  CHECK(!target.scheduled());
  CHECK(!CommandScheduler::getRequiring(&intake).has_value());

  forget(proxy.get());
  forget(&target);
  CommandScheduler::forgetSubsystem(&intake);
}

/// Inside a Sequence the proxy blocks the next step until the target is done.
void proxyInsideSequence() {
  std::printf("-- ProxyCommand in a Sequence waits for the target\n");
  TracerCommand target(3);
  TracerCommand after(1);
  auto proxy = target.asProxy();
  Sequence sequence({proxy.get(), &after});

  sequence.schedule();
  CHECK(target.scheduled());
  int ticks = 0;
  while (sequence.scheduled() && ticks < 20) {
    CommandScheduler::run();
    ++ticks;
    if (target.scheduled()) {
      // The next step must not start while the target still runs.
      CHECK_EQ(d(after.initializes), 0.0);
    }
  }
  CHECK(!sequence.scheduled());
  CHECK_EQ(d(target.executes), 3.0);
  CHECK_EQ(d(after.executes), 1.0);

  forget(&sequence);
  forget(&target);
}

/// Cancelling a Sequence while its proxy step runs cancels the target too.
void proxyCancelledWithSequence() {
  std::printf("-- cancelling a Sequence cancels its proxy's target\n");
  TracerCommand target(0);
  TracerCommand after(1);
  auto proxy = target.asProxy();
  Sequence sequence({proxy.get(), &after});

  sequence.schedule();
  CommandScheduler::run();
  CHECK(target.scheduled());

  sequence.cancel();
  CHECK(!sequence.scheduled());
  CHECK(!target.scheduled());
  CHECK_EQ(d(target.interrupted_ends), 1.0);
  CHECK_EQ(d(after.initializes), 0.0);

  forget(&sequence);
  forget(&target);
}

/// A supplier that returns nullptr leaves the proxy with nothing to run. The
/// proxy finishes on the next tick, and cancelling it before then is safe.
/// isFinished() already allowed for a null target; initialize() and end()
/// used to call through it anyway, which UBSan reports.
void proxyNullSupplier() {
  std::printf("-- ProxyCommand whose supplier returns nullptr\n");
  ProxyCommand proxy([]() -> Command* { return nullptr; });
  proxy.schedule();
  CHECK(proxy.scheduled());
  CommandScheduler::run();
  CHECK(!proxy.scheduled());

  proxy.schedule();
  proxy.cancel();
  CHECK(!proxy.scheduled());
  forget(&proxy);
}

/// The supplier runs on every initialize, so a reused proxy can pick a
/// different target each time.
void proxySupplierRunsEachInitialize() {
  std::printf("-- ProxyCommand supplier runs on every initialize\n");
  TracerCommand first(1);
  TracerCommand second(1);
  int calls = 0;
  ProxyCommand proxy([&]() -> Command* {
    ++calls;
    return calls == 1 ? static_cast<Command*>(&first) : &second;
  });

  proxy.schedule();
  CHECK(first.scheduled());
  for (int i = 0; i < 3; ++i) CommandScheduler::run();
  CHECK(!proxy.scheduled());

  proxy.schedule();
  CHECK(second.scheduled());
  for (int i = 0; i < 3; ++i) CommandScheduler::run();
  CHECK(!proxy.scheduled());
  CHECK_EQ(d(calls), 2.0);
  CHECK_EQ(d(first.executes), 1.0);
  CHECK_EQ(d(second.executes), 1.0);

  forget(&proxy);
  forget(&first);
  forget(&second);
}

}  // namespace

int main() {
  mclib::test::setCompetitionStatus(0);
  repeatRestartsInnerCommand();
  repeatInstantInnerOncePerTick();
  repeatCancelInterruptsInner();
  repeatPassesRequirements();
  repeatEndsOnDisable();
  proxySchedulesTarget();
  proxyCancelCancelsTarget();
  proxyCancelFromRunLoop();
  proxyTargetCancelledElsewhere();
  proxyHoldsNoRequirements();
  proxyInsideSequence();
  proxyCancelledWithSequence();
  proxyNullSupplier();
  proxySupplierRunsEachInitialize();
  return mclib::test::summary("repeat_proxy_command");
}
