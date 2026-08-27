// mclib
#pragma once

#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>
#include <vector>

/**
 * @file time_budget.hpp
 * @brief The clock arithmetic behind a Routine's autonomous time budget.
 *
 * `auton/autonomous_routine.cpp` reaches the command scheduler and PROS, so
 * none of it runs on a host machine. Everything here is the part that only
 * needs a clock: how much of the period is gone, how much is left, what a
 * step's timeout becomes when less is left than the step asked for, and which
 * step runs next once the period is over. It reads `mclib::time::now()`, so a
 * test drives it with `mclib::time::ScopedClock` and asserts exact numbers.
 * `tests/time_budget_test.cpp` does exactly that.
 */

namespace mclib {
namespace auton {

/// @brief The VEX autonomous period. The default total when one is not given.
inline constexpr QTime kAutonomousPeriod = 15.0 * units::second;

/**
 * @brief Smallest timeout ever handed to a step.
 *
 * The motion API reads a timeout of zero as "no timeout", so clamping a step
 * down to exactly zero would turn a nearly-expired budget into an unbounded
 * motion - the opposite of what was asked for. Everything clamps to this
 * instead, which is one scheduler tick and change.
 */
inline constexpr QTime kMinStepTimeout = 20.0 * units::millisecond;

/// @brief Default slice of the budget reserved for the must-run steps.
inline constexpr QTime kDefaultGrace = 1.5 * units::second;

/**
 * @brief What happens to the steps that are left when the budget runs out.
 *
 * Neither choice lets the routine keep going as if nothing happened. The
 * question is only whether anything at all still runs.
 */
enum class DeadlinePolicy {
  /**
   * @brief Interrupt the running step and finish. Nothing else runs.
   *
   * The honest default for a routine whose every step is a drive: there is no
   * point starting a motion with no time to finish it.
   */
  Abandon,
  /**
   * @brief Interrupt the running step, skip the rest, but still run the steps
   *        marked `mustRun()`.
   *
   * This is the useful one on a field. A routine that overruns is usually one
   * or two motions behind, and the step that actually scores - open the claw,
   * fire the intake, drop the goal - takes a fraction of a second and is worth
   * more than the motion it was waiting on. Marking that step `mustRun()` says
   * "if you run out of time, skip the driving and do this". Those steps share
   * the grace window - the tail of the budget held back for them - so a
   * must-run step cannot hang either.
   */
  FinishMustRun,
};

/// @brief How long a routine may run and what it does when that is up.
struct TimeBudgetConfig {
  /// @brief Total allowance. Not positive disables the budget entirely.
  QTime total = 0.0 * units::second;
  /**
   * @brief The tail of @p total held back for the must-run steps.
   *
   * Held back, not added on. A grace bolted onto the end of a 15 s budget puts
   * the must-run steps at 15.0 to 16.5 s, which is after the field has
   * disabled the robot - the one place they must not be. The driving stops at
   * `total - grace` instead, and the scoring action runs in the last 1.5 s of
   * the same 15 s.
   *
   * Ignored under DeadlinePolicy::Abandon, which runs nothing after expiry and
   * so has no reason to hold time back. Clamped to @p total.
   */
  QTime grace = kDefaultGrace;
  DeadlinePolicy policy = DeadlinePolicy::FinishMustRun;
};

/**
 * @brief A start timestamp and an allowance, in two phases.
 *
 * Before expiry `remaining()` counts down to the start of the grace window, so
 * a step's timeout is clamped against the time left for driving. On expiry the
 * owner calls `enterGrace()` and `remaining()` counts down to the end of the
 * total instead. Both windows end at the same wall-clock instant the routine
 * was given, which is the point.
 */
class TimeBudget {
public:
  /**
   * @brief Install a configuration.
   *
   * @details Leaves a run in progress alone: changing the policy or the grace
   * from inside a routine must not hand it a fresh 15 seconds. Only start()
   * and reset() move the start stamp.
   */
  void configure(const TimeBudgetConfig& config);

  /// @brief The configuration in force.
  const TimeBudgetConfig& config() const { return m_config; }

  /// @brief True when a positive total was configured.
  bool active() const { return m_config.total > QTime{}; }

  /**
   * @brief The part of the total the must-run steps are not entitled to.
   *
   * The configured grace, clamped to the total, and zero when the policy runs
   * nothing after expiry.
   */
  QTime graceReserve() const;

  /// @brief How long the routine may drive for: the total less the reserve.
  QTime driveAllowance() const;

  /// @brief Stamp the start of the run at the current time.
  void start() { start(time::now()); }

  /// @brief Stamp the start of the run at @p start_time.
  void start(QTime start_time);

  /// @brief Forget the start stamp and the grace window. Keeps the config.
  void reset();

  /// @brief True once start() has been called and reset() has not.
  bool started() const { return m_started; }

  /**
   * @brief Time since start().
   * @return Zero before start(), never negative.
   */
  QTime elapsed() const { return elapsedAt(time::now()); }

  /// @brief @copydoc elapsed
  QTime elapsedAt(QTime instant) const;

  /**
   * @brief Time left before the current phase ends.
   *
   * Counts down to the start of the grace window before expiry, and to the end
   * of the total after it. Never negative. Zero when there is no budget, which
   * callers must test with active() rather than reading as "no time left".
   */
  QTime remaining() const { return remainingAt(time::now()); }

  /// @brief @copydoc remaining
  QTime remainingAt(QTime instant) const;

  /**
   * @brief True when the driving allowance has run out.
   *
   * False again once enterGrace() has been called, because from then on the
   * question is graceExpired().
   */
  bool expired() const { return expiredAt(time::now()); }

  /// @brief @copydoc expired
  bool expiredAt(QTime instant) const;

  /// @brief Enter the grace window, the last `grace` of the total.
  void enterGrace() { m_in_grace = true; }

  /// @brief True once enterGrace() has been called.
  bool inGrace() const { return m_in_grace; }

  /// @brief True when the whole total has run out, grace included.
  bool graceExpired() const { return graceExpiredAt(time::now()); }

  /// @brief @copydoc graceExpired
  bool graceExpiredAt(QTime instant) const;

private:
  TimeBudgetConfig m_config{};
  QTime m_start{};
  bool m_started = false;
  bool m_in_grace = false;
};

/**
 * @brief The timeout a step actually gets, given what is left of the budget.
 *
 * A step that asks for 3 s when 800 ms of the period remain would run 2.2 s
 * past the end of autonomous with the drive still commanded. It gets 800 ms
 * instead, and the routine stops it there.
 *
 * @param requested The step's own timeout. Not positive means "no timeout",
 *   which the budget turns into "whatever is left".
 * @param remaining What is left of the budget, from TimeBudget::remaining().
 * @param floor The smallest timeout to hand back. See kMinStepTimeout.
 * @return @p requested when it fits, otherwise @p remaining, never below
 *   @p floor.
 */
QTime clampStepTimeout(QTime requested,
                       QTime remaining,
                       QTime floor = kMinStepTimeout);

/**
 * @brief The next step to run once the budget has expired.
 *
 * @param must_run One flag per step, in order.
 * @param from The step the routine was on when the budget expired. It is
 *   included in the search, so a must-run step that had not started yet is
 *   selected rather than skipped.
 * @param policy What to do with the rest.
 * @return The index to jump to, or `must_run.size()` when nothing else should
 *   run, which is the routine's own "finished" index.
 */
std::size_t nextStepAfterDeadline(const std::vector<bool>& must_run,
                                  std::size_t from,
                                  DeadlinePolicy policy);

}  // namespace auton
}  // namespace mclib
