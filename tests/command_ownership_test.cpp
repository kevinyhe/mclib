// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Command decorator ownership.
//
// andThen(), with(), race(), withTimeout(), until(), repeatedly() and
// asProxy() used to return a raw `new` pointer that nothing ever deleted.
// withTimeout() and until() were worse: the WaitCommand and WaitUntilCommand
// they build internally had no owner at all, so they leaked even if the caller
// remembered to delete the group.
//
// They now return std::unique_ptr<Command>, and the group adopts the helper it
// creates. This test pins the return types down and checks that a decorated
// command still runs and still ends, so a future change back to raw pointers
// fails to compile here rather than quietly leaking on the brain.
//
// The leak itself is caught by the AddressSanitizer run in CI, not here - a
// host binary cannot see its own heap without it.

#include "mclib/command/commandScheduler.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/parallelCommandGroup.h"
#include "mclib/command/parallelRaceGroup.h"
#include "mclib/command/proxyCommand.h"
#include "mclib/command/repeatCommand.h"
#include "mclib/command/sequence.h"
#include "mclib/command/trigger.h"
#include "mclib/command/waitCommand.h"
#include "mclib/command/waitUntilCommand.h"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <type_traits>

namespace {

std::uint32_t g_fake_ms = 0;

/// Counts its own destruction, so "the caller owns this" is observable.
int g_destroyed = 0;

class CountingCommand : public Command {
public:
  int executions = 0;

  void execute() override { ++executions; }
  bool isFinished() override { return false; }
  ~CountingCommand() override { ++g_destroyed; }
};

/// End and forget a command the scheduler may still reference, before the
/// unique_ptr that owns it goes out of scope.
void retire(const std::unique_ptr<Command>& command) {
  CommandScheduler::endAndForget(command.get());
}

// ---------------------------------------------------------------------------
// 1. Every decorator hands back ownership.
// ---------------------------------------------------------------------------
void returnTypes() {
  std::printf("-- every decorator returns std::unique_ptr\n");

  static InstantCommand a([] {}, {});
  static InstantCommand b([] {}, {});
  using Owned = std::unique_ptr<Command>;

  static_assert(std::is_same_v<decltype(a.andThen(&b)), Owned>);
  static_assert(std::is_same_v<decltype(a.with(&b)), Owned>);
  static_assert(std::is_same_v<decltype(a.race(&b)), Owned>);
  static_assert(std::is_same_v<decltype(a.withTimeout(1.0 * second)), Owned>);
  static_assert(std::is_same_v<decltype(a.until([] { return true; })), Owned>);
  static_assert(std::is_same_v<decltype(a.repeatedly()), Owned>);
  static_assert(std::is_same_v<decltype(a.asProxy()), Owned>);

  static Trigger t1([] { return true; });
  static Trigger t2([] { return false; });
  static_assert(std::is_same_v<decltype(t1.andOther(&t2)), std::unique_ptr<Trigger>>);
  static_assert(std::is_same_v<decltype(t1.orOther(&t2)), std::unique_ptr<Trigger>>);
  static_assert(std::is_same_v<decltype(t1.negate()), std::unique_ptr<Trigger>>);

  // Discarding the result drops the command on the floor, which is exactly
  // the old leak. [[nodiscard]] makes that a warning at every call site.
  CHECK(a.andThen(&b) != nullptr);
  CHECK(a.withTimeout(1.0 * second) != nullptr);
  CHECK(t1.negate() != nullptr);
}

// ---------------------------------------------------------------------------
// 2. The caller's unique_ptr is the only owner: dropping it destroys the
//    command, and nothing else holds a claim on it.
// ---------------------------------------------------------------------------
void ownershipIsExclusive() {
  std::printf("-- dropping the unique_ptr destroys the command\n");

  g_destroyed = 0;
  {
    auto counting = std::make_unique<CountingCommand>();
    auto timed = counting->withTimeout(100.0 * millisecond);
    CHECK_EQ(static_cast<double>(g_destroyed), 0.0);
    // The group borrows `counting`; it does not adopt it.
    timed.reset();
    CHECK_EQ(static_cast<double>(g_destroyed), 0.0);
  }
  // Now the caller's own unique_ptr went away.
  CHECK_EQ(static_cast<double>(g_destroyed), 1.0);
}

// ---------------------------------------------------------------------------
// 3. A decorated command still runs, and withTimeout() still times out. The
//    helper WaitCommand is owned by the group, so it has to survive as long
//    as the group does.
// ---------------------------------------------------------------------------
void decoratedCommandsStillRun() {
  std::printf("-- withTimeout() through a real scheduler\n");

  g_fake_ms = 0;
  auto counting = std::make_unique<CountingCommand>();
  auto timed = counting->withTimeout(100.0 * millisecond);

  timed->schedule();
  CHECK(timed->scheduled());

  int ticks = 0;
  for (; ticks < 50 && timed->scheduled(); ++ticks) {
    CommandScheduler::run();
    g_fake_ms += 10;
  }

  CHECK(!timed->scheduled());
  // 100 ms of timeout at 10 ms a tick. The helper is still alive and still
  // reading the clock at the end, which is the part that used to leak.
  std::printf("   100 ms timeout at 10 ms/tick: ended on tick %d\n", ticks);
  CHECK(ticks >= 10 && ticks <= 12);
  CHECK(counting->executions > 0);

  retire(timed);
}

// ---------------------------------------------------------------------------
// 4. Sequence and the parallel groups borrow, they do not adopt.
// ---------------------------------------------------------------------------
void groupsBorrowTheirMembers() {
  std::printf("-- groups borrow their members\n");

  g_destroyed = 0;
  auto first = std::make_unique<CountingCommand>();
  auto second = std::make_unique<CountingCommand>();
  {
    auto seq = first->andThen(second.get());
    auto par = first->with(second.get());
    auto rce = first->race(second.get());
    CHECK(seq != nullptr);
    CHECK(par != nullptr);
    CHECK(rce != nullptr);
  }
  // All three groups are gone; neither member went with them.
  CHECK_EQ(static_cast<double>(g_destroyed), 0.0);
  first.reset();
  second.reset();
  CHECK_EQ(static_cast<double>(g_destroyed), 2.0);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  returnTypes();
  ownershipIsExclusive();
  decoratedCommandsStillRun();
  groupsBorrowTheirMembers();
  return mclib::test::summary("command_ownership");
}
