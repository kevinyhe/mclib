// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Timed commands finish on the tick their duration ends, whatever the clock
// read when they started.
//
// They used to subtract two time::now() values: seconds in a double. For
// some start times the difference came out a hair short - 100 ms as
// 99.99999 ms for 59% of start times - and a check made once per 10 ms tick
// then finished a tick late. time::hasElapsed() compares whole milliseconds.

#include "mclib/auton/wait_until_command.hpp"
#include "mclib/command/commandScheduler.h"
#include "mclib/command/waitCommand.h"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>
#include <limits>

using namespace mclib::units::literals;

namespace {

std::uint32_t now_ms = 0;
std::uint32_t fakeClock() { return now_ms; }

/// Tick the scheduler every 10 ms until @p command ends. Returns how long it
/// ran, in ms.
std::uint32_t runTimed(Command& command, std::uint32_t start_ms) {
  now_ms = start_ms;
  command.schedule();
  CommandScheduler::run();
  while (command.scheduled() && now_ms - start_ms < 5000) {
    now_ms += 10;
    CommandScheduler::run();
  }
  return now_ms - start_ms;
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock(fakeClock);

  std::printf("-- hasElapsed() over every start time\n");
  {
    const int durations[] = {50, 100, 250, 500, 1000};
    for (int d : durations) {
      const QTime duration = d * mclib::units::millisecond;
      int late = 0, early = 0;
      for (std::uint32_t start = 0; start < 200000; ++start) {
        now_ms = start + d;
        late += !mclib::time::hasElapsed(start, duration);
        now_ms = start + d - 1;
        early += mclib::time::hasElapsed(start, duration);
      }
      std::printf("   %4d ms: %d late, %d early of 200000 start times\n", d, late, early);
      CHECK_EQ(late, 0);
      CHECK_EQ(early, 0);
    }
    // Across the 32-bit wrap.
    const std::uint32_t start = std::numeric_limits<std::uint32_t>::max() - 30;
    now_ms = start + 99;
    CHECK(!mclib::time::hasElapsed(start, 100_ms));
    now_ms = start + 100;
    CHECK(mclib::time::hasElapsed(start, 100_ms));
    // Zero and negative durations have passed at once.
    CHECK(mclib::time::hasElapsed(now_ms, 0_ms));
    CHECK(mclib::time::hasElapsed(now_ms, -5_ms));
  }

  // 5 ms and 39 ms are start times where 100 ms and 250 ms used to read short.
  std::printf("-- StateMechanism::makeStateForCommand()\n");
  {
    mclib::mechanism::MappedMechanism<int> mech(0, [](const int&) {});
    for (std::uint32_t start : {5u, 39u, 1000u}) {
      auto command = mech.makeStateForCommand(1, 100_ms);
      const std::uint32_t ran = runTimed(*command, start);
      std::printf("   start %u ms: ran %u ms\n", start, ran);
      CHECK_EQ(ran, 100u);
      CommandScheduler::forgetCommand(command.get());
    }
    CHECK_EQ(mech.getState(), 1);
    auto longer = mech.makeStateForCommand(2, 250_ms);
    CHECK_EQ(runTimed(*longer, 39), 250u);
    CommandScheduler::forgetCommand(longer.get());
  }

  std::printf("-- WaitCommand\n");
  {
    for (std::uint32_t start : {5u, 39u, 1000u}) {
      WaitCommand wait(100_ms);
      const std::uint32_t ran = runTimed(wait, start);
      std::printf("   start %u ms: ran %u ms\n", start, ran);
      // It used to need strictly more than the duration, so it always ran a
      // tick over when the arithmetic was exact.
      CHECK_EQ(ran, 100u);
      CommandScheduler::forgetCommand(&wait);
    }
  }

  std::printf("-- WaitUntilTimeoutCommand\n");
  {
    for (std::uint32_t start : {5u, 39u}) {
      mclib::auton::WaitUntilTimeoutCommand wait([] { return false; }, 100_ms);
      CHECK_EQ(runTimed(wait, start), 100u);
      CHECK(wait.timedOut());
      CommandScheduler::forgetCommand(&wait);
    }
  }

  return mclib::test::summary("timed_command");
}
