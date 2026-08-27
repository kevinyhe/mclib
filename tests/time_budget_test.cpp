// mclib
/**
 * @file time_budget_test.cpp
 * @brief The autonomous time budget, on a fake clock, with exact numbers.
 *
 * `auton/autonomous_routine.cpp` reaches the command scheduler and PROS and
 * cannot be linked on a host, so what is tested here is the part of the budget
 * that was lifted out of it: `auton/time_budget.hpp` (elapsed, remaining,
 * expiry, the grace window, the step-timeout clamp, the step to jump to after
 * the deadline) and `auton/wait_until_command.hpp` (a predicate wait that ends
 * on its timeout instead of hanging). Both read `mclib::time::now()`, so every
 * assertion below is a millisecond count, not an approximation.
 *
 * What is NOT covered here, because it needs the scheduler: Routine::execute()
 * calling into the policy, and a chassis command's end(true) braking the drive.
 */
#include "mclib/auton/time_budget.hpp"

#include "mclib/auton/wait_until_command.hpp"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <vector>

using mclib::auton::clampStepTimeout;
using mclib::auton::DeadlinePolicy;
using mclib::auton::kMinStepTimeout;
using mclib::auton::nextStepAfterDeadline;
using mclib::auton::stepTimeoutFor;
using mclib::auton::TimeBudget;
using mclib::auton::TimeBudgetConfig;
using mclib::auton::WaitUntilTimeoutCommand;

namespace {

/// @brief The fake clock, in milliseconds. Advanced by hand between checks.
std::uint32_t g_ms = 0;

std::uint32_t fakeClock() { return g_ms; }

TimeBudgetConfig config(double total_ms,
                        double grace_ms = 1500.0,
                        DeadlinePolicy policy = DeadlinePolicy::FinishMustRun) {
  TimeBudgetConfig out{};
  out.total = total_ms * millisecond;
  out.grace = grace_ms * millisecond;
  out.policy = policy;
  return out;
}

/// @brief A routine that fits: 15 s of budget, 9.5 s of work.
void testUnderBudget() {
  g_ms = 0;
  TimeBudget budget;
  budget.configure(config(15000.0, 0.0));

  CHECK(budget.active());
  CHECK(!budget.started());
  // Before the run starts the whole allowance is still there.
  CHECK_NEAR(budget.remaining().ms(), 15000.0, 1e-9);
  CHECK_NEAR(budget.elapsed().ms(), 0.0, 1e-9);

  g_ms = 100;
  budget.start();
  CHECK(budget.started());

  g_ms = 100 + 9500;
  CHECK_NEAR(budget.elapsed().ms(), 9500.0, 1e-9);
  CHECK_NEAR(budget.remaining().ms(), 5500.0, 1e-9);
  CHECK(!budget.expired());
  CHECK(!budget.inGrace());
}

/// @brief Expiry is at exactly the total, and the count never goes negative.
void testExpiryTick() {
  g_ms = 1000;
  TimeBudget budget;
  budget.configure(config(15000.0, 0.0));
  budget.start();

  g_ms = 1000 + 14999;
  CHECK_NEAR(budget.remaining().ms(), 1.0, 1e-9);
  CHECK(!budget.expired());

  g_ms = 1000 + 15000;
  CHECK_NEAR(budget.remaining().ms(), 0.0, 1e-9);
  CHECK(budget.expired());

  g_ms = 1000 + 20000;
  CHECK_NEAR(budget.elapsed().ms(), 20000.0, 1e-9);
  CHECK_NEAR(budget.remaining().ms(), 0.0, 1e-9);
}

/// @brief The grace is the last 1.5 s of the 15 s, not 1.5 s bolted onto it.
void testGraceWindow() {
  g_ms = 0;
  TimeBudget budget;
  budget.configure(config(15000.0, 1500.0));
  budget.start();

  // The driving stops at 13.5 s so the must-run steps land inside the period.
  CHECK_NEAR(budget.graceReserve().ms(), 1500.0, 1e-9);
  CHECK_NEAR(budget.driveAllowance().ms(), 13500.0, 1e-9);

  g_ms = 13499;
  CHECK_NEAR(budget.remaining().ms(), 1.0, 1e-9);
  CHECK(!budget.expired());

  g_ms = 13500;
  CHECK(budget.expired());
  budget.enterGrace();

  CHECK(budget.inGrace());
  // expired() is about the driving allowance; from here on it is graceExpired.
  CHECK(!budget.expired());
  CHECK_NEAR(budget.remaining().ms(), 1500.0, 1e-9);
  CHECK(!budget.graceExpired());

  g_ms = 13500 + 900;
  CHECK_NEAR(budget.remaining().ms(), 600.0, 1e-9);
  CHECK(!budget.graceExpired());

  // Everything is over at the 15 s the routine was given, not at 16.5 s.
  g_ms = 15000;
  CHECK_NEAR(budget.remaining().ms(), 0.0, 1e-9);
  CHECK(budget.graceExpired());
  CHECK_NEAR(budget.elapsed().ms(), 15000.0, 1e-9);
}

/// @brief Abandon runs nothing after expiry, so it holds no time back.
void testAbandonKeepsWholeTotal() {
  g_ms = 0;
  TimeBudget budget;
  budget.configure(config(15000.0, 1500.0, DeadlinePolicy::Abandon));
  budget.start();

  CHECK_NEAR(budget.graceReserve().ms(), 0.0, 1e-9);
  CHECK_NEAR(budget.driveAllowance().ms(), 15000.0, 1e-9);

  g_ms = 14999;
  CHECK(!budget.expired());
  g_ms = 15000;
  CHECK(budget.expired());
}

/// @brief A grace longer than the total does not make the allowance negative.
void testGraceClampedToTotal() {
  TimeBudget budget;
  budget.configure(config(1000.0, 4000.0));

  CHECK_NEAR(budget.graceReserve().ms(), 1000.0, 1e-9);
  CHECK_NEAR(budget.driveAllowance().ms(), 0.0, 1e-9);
}

/// @brief Changing the policy mid-run must not hand the routine a fresh 15 s.
void testReconfigureKeepsTheRun() {
  g_ms = 0;
  TimeBudget budget;
  budget.configure(config(15000.0, 0.0));
  budget.start();

  g_ms = 12000;
  CHECK_NEAR(budget.elapsed().ms(), 12000.0, 1e-9);

  TimeBudgetConfig changed = budget.config();
  changed.policy = DeadlinePolicy::Abandon;
  budget.configure(changed);

  CHECK(budget.started());
  CHECK_NEAR(budget.elapsed().ms(), 12000.0, 1e-9);
  CHECK_NEAR(budget.remaining().ms(), 3000.0, 1e-9);
}

/// @brief A step timeout longer than what is left gets cut to what is left.
void testStepClamp() {
  const double floor_ms = kMinStepTimeout.ms();

  // Fits: kept exactly.
  CHECK_NEAR(clampStepTimeout(1500.0 * millisecond, 5000.0 * millisecond).ms(),
             1500.0, 1e-9);
  // Does not fit: cut to the 800 ms that remain, not the 3000 ms asked for.
  CHECK_NEAR(clampStepTimeout(3000.0 * millisecond, 800.0 * millisecond).ms(),
             800.0, 1e-9);
  // Exactly fits.
  CHECK_NEAR(clampStepTimeout(800.0 * millisecond, 800.0 * millisecond).ms(),
             800.0, 1e-9);
  // No timeout of its own: it gets the remaining budget as one.
  CHECK_NEAR(clampStepTimeout(0.0 * millisecond, 2400.0 * millisecond).ms(),
             2400.0, 1e-9);
  // Nothing left. The floor matters: a timeout of exactly 0 means "no timeout"
  // to the motion API, which would be an unbounded drive at the worst moment.
  CHECK_NEAR(clampStepTimeout(3000.0 * millisecond, 0.0 * millisecond).ms(),
             floor_ms, 1e-9);
  CHECK_NEAR(clampStepTimeout(3000.0 * millisecond, 5.0 * millisecond).ms(),
             floor_ms, 1e-9);
  CHECK_NEAR(floor_ms, 20.0, 1e-9);
  // A step that asks for less than the floor still gets the floor.
  CHECK_NEAR(clampStepTimeout(1.0 * millisecond, 5000.0 * millisecond).ms(),
             floor_ms, 1e-9);

  // The clamp reads the live budget, so it shortens as the routine runs.
  g_ms = 0;
  TimeBudget budget;
  budget.configure(config(15000.0, 0.0));
  budget.start();

  g_ms = 13200;
  CHECK_NEAR(budget.remaining().ms(), 1800.0, 1e-9);
  CHECK_NEAR(clampStepTimeout(3000.0 * millisecond, budget.remaining()).ms(),
             1800.0, 1e-9);
}

/**
 * @brief A routine with no budget hands every step its own timeout, untouched.
 *
 * This is the regression for a build where the requested timeout never reached
 * the command: every motion came out with a timeout of zero. The loops in
 * motion.cpp are `while (elapsed <= limit)`, so they ran one 10 ms tick and
 * the robot did not move, while ChassisController reads zero as "no timeout"
 * and hung instead. Nothing about it is a compile error, and a routine that
 * never calls withTimeBudget() - which is every routine written so far - is
 * exactly the case that broke.
 */
void testTimeoutWithoutBudget() {
  g_ms = 0;
  TimeBudget none;

  CHECK(!none.active());
  CHECK_NEAR(stepTimeoutFor(2000.0 * millisecond, none).ms(), 2000.0, 1e-9);
  CHECK_NEAR(stepTimeoutFor(800.0 * millisecond, none).ms(), 800.0, 1e-9);
  // A step written with no timeout keeps having none, rather than picking up
  // the floor. Without a budget there is nothing to clamp against.
  CHECK_NEAR(stepTimeoutFor(0.0 * millisecond, none).ms(), 0.0, 1e-9);

  // A configured but never-started budget still clamps, against the full
  // allowance: a step built before the routine runs cannot exceed it.
  TimeBudget budget;
  budget.configure(config(15000.0, 1500.0));
  CHECK_NEAR(stepTimeoutFor(2000.0 * millisecond, budget).ms(), 2000.0, 1e-9);
  CHECK_NEAR(stepTimeoutFor(20000.0 * millisecond, budget).ms(), 13500.0, 1e-9);

  // Once it is running, the same request shortens as the time does.
  budget.start();
  g_ms = 13000;
  CHECK_NEAR(stepTimeoutFor(2000.0 * millisecond, budget).ms(), 500.0, 1e-9);
}

/// @brief Which step runs next once the budget is gone.
void testDeadlineSkip() {
  //            0      1      2      3      4
  // steps:  drive  drive  score  drive  release
  const std::vector<bool> must_run{false, false, true, false, true};

  // Expired while running step 1: skip to the scoring step.
  CHECK_EQ(static_cast<double>(
               nextStepAfterDeadline(must_run, 1, DeadlinePolicy::FinishMustRun)),
           2.0);
  // Scoring done, on step 3: skip that drive, run the release.
  CHECK_EQ(static_cast<double>(
               nextStepAfterDeadline(must_run, 3, DeadlinePolicy::FinishMustRun)),
           4.0);
  // A must-run step that was already running is not skipped over.
  CHECK_EQ(static_cast<double>(
               nextStepAfterDeadline(must_run, 2, DeadlinePolicy::FinishMustRun)),
           2.0);
  // Nothing must-run is left: the routine is finished.
  CHECK_EQ(static_cast<double>(
               nextStepAfterDeadline(must_run, 5, DeadlinePolicy::FinishMustRun)),
           5.0);
  // Abandon ignores the flags entirely.
  CHECK_EQ(static_cast<double>(
               nextStepAfterDeadline(must_run, 1, DeadlinePolicy::Abandon)),
           5.0);

  // A routine with nothing marked behaves the same under either policy.
  const std::vector<bool> none{false, false, false};
  CHECK_EQ(static_cast<double>(
               nextStepAfterDeadline(none, 0, DeadlinePolicy::FinishMustRun)),
           3.0);
}

/// @brief A predicate that never comes true ends on the timeout, not never.
void testWaitUntilTimeout() {
  g_ms = 5000;
  WaitUntilTimeoutCommand command([]() { return false; },
                                  250.0 * millisecond);
  command.initialize();

  CHECK(!command.isFinished());
  g_ms = 5000 + 249;
  CHECK(!command.isFinished());
  CHECK(!command.timedOut());

  g_ms = 5000 + 250;
  CHECK(command.isFinished());
  CHECK(command.timedOut());
}

/// @brief The predicate still wins when it comes true first.
void testWaitUntilCondition() {
  g_ms = 0;
  bool ready = false;
  WaitUntilTimeoutCommand command([&ready]() { return ready; },
                                  5000.0 * millisecond);
  command.initialize();

  g_ms = 300;
  CHECK(!command.isFinished());

  ready = true;
  CHECK(command.isFinished());
  CHECK(!command.timedOut());
  CHECK_NEAR(command.timeout().ms(), 5000.0, 1e-9);
}

/// @brief A non-positive timeout is the old wait-forever, asked for on purpose.
void testWaitUntilNoTimeout() {
  g_ms = 0;
  WaitUntilTimeoutCommand command([]() { return false; }, 0.0 * millisecond);
  command.initialize();

  g_ms = 60000;
  CHECK(!command.isFinished());
  CHECK(!command.timedOut());
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock(fakeClock);

  testUnderBudget();
  testExpiryTick();
  testGraceWindow();
  testAbandonKeepsWholeTotal();
  testGraceClampedToTotal();
  testReconfigureKeepsTheRun();
  testStepClamp();
  testTimeoutWithoutBudget();
  testDeadlineSkip();
  testWaitUntilTimeout();
  testWaitUntilCondition();
  testWaitUntilNoTimeout();

  return mclib::test::summary("time_budget");
}
