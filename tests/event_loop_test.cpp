// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// EventLoop: bindings run in bind order on every poll(), clear() removes
// them, and a binding may call bind() or clear() on its own loop.
//
// poll() used to walk the binding vector with a range-for. A bind() from
// inside a binding could grow the vector and free the storage the loop was
// walking, and a clear() destroyed the binding that was still running. Both
// are reachable through the scheduler: a Trigger fires, schedules a command,
// and that command's initialize() creates a new Trigger on the same loop.
// The sanitizer run reports the old code as a heap-use-after-free.

#include "mclib/command/commandScheduler.h"
#include "mclib/command/eventLoop.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/trigger.h"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <functional>
#include <memory>
#include <vector>

namespace {

double d(int value) { return static_cast<double>(value); }

bool sameOrder(const std::vector<int>& actual, const std::vector<int>& expected) {
  return actual == expected;
}

/// Bindings run in the order they were bound, once per poll.
void runsInBindOrder() {
  std::printf("-- bindings run in bind order\n");
  std::vector<int> calls;
  EventLoop loop;
  loop.bind([&] { calls.push_back(1); });
  loop.bind([&] { calls.push_back(2); });
  loop.bind([&] { calls.push_back(3); });

  loop.poll();
  CHECK(sameOrder(calls, {1, 2, 3}));
  loop.poll();
  CHECK(sameOrder(calls, {1, 2, 3, 1, 2, 3}));
}

/// The constructors keep the given order too.
void constructorsKeepOrder() {
  std::printf("-- constructor bindings keep their order\n");
  std::vector<int> calls;
  EventLoop from_list{[&] { calls.push_back(1); }, [&] { calls.push_back(2); }};
  from_list.poll();
  CHECK(sameOrder(calls, {1, 2}));

  calls.clear();
  std::vector<std::function<void()>> bindings{[&] { calls.push_back(3); },
                                               [&] { calls.push_back(4); }};
  EventLoop from_vector(bindings);
  from_vector.poll();
  CHECK(sameOrder(calls, {3, 4}));
}

/// An empty loop polls without doing anything.
void emptyLoop() {
  std::printf("-- empty loop\n");
  EventLoop loop;
  loop.poll();
  loop.clear();
  loop.poll();
  CHECK(true);
}

/// clear() removes every binding; later binds still work.
void clearRemovesBindings() {
  std::printf("-- clear() removes bindings\n");
  int count = 0;
  EventLoop loop;
  loop.bind([&] { ++count; });
  loop.bind([&] { ++count; });
  loop.poll();
  CHECK_EQ(d(count), 2.0);

  loop.clear();
  loop.poll();
  CHECK_EQ(d(count), 2.0);

  loop.bind([&] { count += 10; });
  loop.poll();
  CHECK_EQ(d(count), 12.0);
}

/// A binding that binds more bindings. Enough of them to force the vector to
/// reallocate while poll() walks it. The new bindings start on the next
/// poll, not the current one, so one poll never runs a binding twice or
/// runs one that did not exist when it started.
void bindFromInsideBinding() {
  std::printf("-- bind() from inside a binding\n");
  std::vector<int> calls;
  EventLoop loop;
  bool added = false;
  loop.bind([&] {
    calls.push_back(1);
    if (!added) {
      added = true;
      for (int i = 0; i < 64; ++i) {
        loop.bind([&calls, i] { calls.push_back(100 + i); });
      }
    }
  });
  loop.bind([&] { calls.push_back(2); });

  loop.poll();
  CHECK(sameOrder(calls, {1, 2}));

  calls.clear();
  loop.poll();
  CHECK_EQ(d(static_cast<int>(calls.size())), 66.0);
  CHECK(calls.size() >= 3 && calls[0] == 1 && calls[1] == 2 && calls[2] == 100);
  CHECK(!calls.empty() && calls.back() == 163);
}

/// A binding that clears its own loop. The rest of this poll is skipped and
/// the loop is empty afterwards; the running binding is not destroyed under
/// itself (the sanitizer run checks that part).
void clearFromInsideBinding() {
  std::printf("-- clear() from inside a binding\n");
  std::vector<int> calls;
  EventLoop loop;
  auto big = std::make_shared<std::vector<int>>(1000, 7);
  loop.bind([&calls, &loop, big] {
    calls.push_back(1);
    loop.clear();
    // Touch the captures after clear(). If clear() destroyed this binding,
    // `big` is freed here.
    calls.push_back((*big)[999]);
  });
  loop.bind([&] { calls.push_back(2); });

  loop.poll();
  CHECK(sameOrder(calls, {1, 7}));

  calls.clear();
  loop.poll();
  CHECK(calls.empty());
}

/// clear() then bind() in the same binding: the old bindings go, the new one
/// stays and runs from the next poll.
void clearThenBindFromInsideBinding() {
  std::printf("-- clear() then bind() from inside a binding\n");
  std::vector<int> calls;
  EventLoop loop;
  loop.bind([&] {
    calls.push_back(1);
    loop.clear();
    loop.bind([&] { calls.push_back(3); });
  });
  loop.bind([&] { calls.push_back(2); });

  loop.poll();
  CHECK(sameOrder(calls, {1}));
  calls.clear();
  loop.poll();
  CHECK(sameOrder(calls, {3}));
  calls.clear();
  loop.poll();
  CHECK(sameOrder(calls, {3}));
}

/// bind() then clear() in the same binding drops the binding just added.
void bindThenClearFromInsideBinding() {
  std::printf("-- bind() then clear() from inside a binding\n");
  int count = 0;
  EventLoop loop;
  loop.bind([&] {
    loop.bind([&] { ++count; });
    loop.clear();
  });
  loop.poll();
  loop.poll();
  CHECK_EQ(d(count), 0.0);
}

/// The same thing through the scheduler: a Trigger fires, its command's
/// initialize() creates another Trigger on the scheduler's loop.
void triggerCreatedByTriggeredCommand() {
  std::printf("-- a triggered command creates a new Trigger\n");
  mclib::test::setCompetitionStatus(0);
  bool pressed = false;
  int inner_fires = 0;
  std::vector<std::unique_ptr<Trigger>> triggers;
  std::vector<std::unique_ptr<Command>> commands;

  auto inner_command = std::make_unique<InstantCommand>([&] { ++inner_fires; },
                                                        std::initializer_list<Subsystem*>{});
  Command* inner_ptr = inner_command.get();
  commands.push_back(std::move(inner_command));

  auto outer_command = std::make_unique<InstantCommand>(
      [&] {
        // Enough triggers to reallocate the scheduler's binding vector.
        for (int i = 0; i < 32; ++i) {
          triggers.push_back(std::make_unique<Trigger>([&] { return pressed; }));
          triggers.back()->onTrue(inner_ptr);
        }
      },
      std::initializer_list<Subsystem*>{});
  Trigger outer([&] { return pressed; });
  outer.onTrue(outer_command.get());

  pressed = true;
  CommandScheduler::run();
  CHECK_EQ(d(static_cast<int>(triggers.size())), 32.0);

  // The new triggers saw `pressed` already true when bound, so they wait for
  // the next rising edge.
  pressed = false;
  CommandScheduler::run();
  pressed = true;
  CommandScheduler::run();
  CHECK(inner_fires >= 1);

  CommandScheduler::getEventLoop()->clear();
  CommandScheduler::endAndForget(outer_command.get());
  CommandScheduler::endAndForget(inner_ptr);
}

}  // namespace

int main() {
  runsInBindOrder();
  constructorsKeepOrder();
  emptyLoop();
  clearRemovesBindings();
  bindFromInsideBinding();
  clearFromInsideBinding();
  clearThenBindFromInsideBinding();
  bindThenClearFromInsideBinding();
  triggerCreatedByTriggeredCommand();
  return mclib::test::summary("event_loop");
}
