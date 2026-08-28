// mclib
#pragma once

#include "mclib/auton/time_budget.hpp"
#include "mclib/chassis/chassis_controller.hpp"
#include "mclib/command/command.h"
#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace mclib {
namespace auton {

/// @brief A field position. Compass frame, origin and axes per `mclib/math.hpp`.
struct Point {
  QLength x{};
  QLength y{};

  constexpr Point() = default;
  constexpr Point(QLength x_in, QLength y_in) : x(x_in), y(y_in) {}
  /// @brief From an odometry vector, whose components are inches.
  explicit Point(const Vec2& point)
      : x(point.x() * units::inch), y(point.y() * units::inch) {}
};

/**
 * @brief How long `waitUntil()` waits when the caller does not say.
 *
 * A predicate that never becomes true used to hang the routine for the rest of
 * the match. Five seconds is a third of the autonomous period: long enough
 * that no honest wait for a mechanism hits it, short enough that a stuck one
 * still leaves time for the steps behind it.
 */
inline constexpr QTime kDefaultWaitUntilTimeout = 5.0 * units::second;

/**
 * @brief A sequence of motions and commands, built by chaining calls.
 *
 * Every motion returns a MotionStep, whose `withXxx()` setters rebuild that
 * step in place, so an autonomous reads as one expression:
 *
 * @code
 * routine.driveTo(24_in, 1_s).withMaxVoltage(8_V)
 *        .turnToAngle(90_deg, 800_ms)
 *        .moveToPoint(Point{24_in, 36_in}, 1, 1500_ms).withoutExit();
 * @endcode
 *
 * ## The time budget
 *
 * Autonomous is 15 seconds. A routine used to have no idea of that: it knew
 * each step's own timeout and nothing about the period they all had to fit
 * inside, so a routine that ran long was simply cut off by the field, drive
 * still commanded, last step never reached.
 *
 * `withTimeBudget()` gives the routine a start stamp and an allowance:
 *
 * @code
 * routine.withTimeBudget(14_s)
 *        .driveTo(24_in, 2_s)
 *        .moveToPoint(Point{24_in, 36_in}, 1, 3_s)
 *        .runOnce([] { claw.open(); }).mustRun();
 * @endcode
 *
 * Three things follow from it.
 *
 * 1. `elapsed()` and `remaining()` answer how much autonomous is left, both to
 *    the caller and to a `runOnce()` lambda inside the routine.
 * 2. A motion whose timeout is longer than what is left is rebuilt with the
 *    shorter one before it starts, so it ends itself inside the budget instead
 *    of overrunning it. A motion with no timeout at all gets the time that
 *    remains as one.
 * 3. When the allowance runs out the routine interrupts the running step and
 *    puts the drive at rest - see stopCurrentStep() and brakeDrive() in the
 *    source - and applies its DeadlinePolicy. The default, FinishMustRun,
 *    skips the rest except the steps marked
 *    `mustRun()`, which share the grace window: the last 1.5 s of the budget,
 *    held back for them rather than added on the end. That is the case this is
 *    built for - the driving is behind schedule, but the half-second action
 *    that scores what the robot is already carrying still happens, and it
 *    happens before the field disables the robot.
 *
 * Without `withTimeBudget()` nothing here is on and the routine behaves as it
 * always did.
 */
class Routine : public Command {
public:
  class MotionStep {
  public:
    /// @brief Voltage cap for this step.
    MotionStep& withMaxVoltage(QVoltage max_voltage);
    /// @brief Voltage floor for this step; negative selects the `min_output` default.
    MotionStep& withMinVoltage(QVoltage min_voltage);
    MotionStep& withExit(bool exit = true);
    MotionStep& withoutExit();
    MotionStep& withStop(bool stop = true);
    MotionStep& withoutStop();
    MotionStep& withDirection(int direction);
    MotionStep& reversed(bool reverse = true);
    MotionStep& withOverturn(bool overturn = true);
    MotionStep& withoutOverturn();
    /// @brief Voltage `wallReset()` pushes into the wall with. Negative reverses.
    MotionStep& withDrivePower(QVoltage drive_power);
    /// @brief Motor current above which `wallReset()` calls it a stall.
    MotionStep& withCurrentThreshold(QCurrent current_threshold);
    /// @brief Motor speed below which `wallReset()` calls it a stall.
    MotionStep& withVelocityThreshold(QAngularVelocity velocity_threshold);

    /**
     * @brief Run this step even if the routine's time budget has expired.
     *
     * Marking a long drive `mustRun()` mostly defeats the point of the budget;
     * this is for the short actions at the end of a routine.
     */
    MotionStep& mustRun(bool must_run = true);

    Routine& done();
    Routine& add(std::unique_ptr<Command> command);
    Routine& then(std::unique_ptr<Command> command);
    Routine& trigger(std::unique_ptr<Command> command);
    Routine& runOnce(std::function<void()> action);
    Routine& wait(QTime duration);
    Routine& waitUntil(std::function<bool()> condition,
                       QTime timeout = kDefaultWaitUntilTimeout);

    MotionStep driveDistance(QLength distance, QTime timeout = 0.0 * units::second);
    MotionStep turnToHeading(QAngle heading, QTime timeout = 0.0 * units::second);
    MotionStep turnToAngle(QAngle turn_angle, QTime timeout);
    MotionStep driveTo(QLength distance, QTime timeout);
    MotionStep curveCircle(QAngle result_angle, QLength center_radius, QTime timeout);
    MotionStep curveCircleReverse(QAngle result_angle,
                                  QLength center_radius,
                                  QTime timeout);
    MotionStep swing(QAngle swing_angle, double drive_direction, QTime timeout);
    MotionStep wallReset(QLength reset_x,
                         QLength reset_y,
                         QAngle reset_heading,
                         QTime timeout);
    MotionStep turnToPoint(Point point, QTime timeout = 1000.0 * units::millisecond);
    MotionStep turnToPoint(QLength x,
                           QLength y,
                           QTime timeout = 1000.0 * units::millisecond);
    MotionStep moveToPoint(Point point, QTime timeout);
    MotionStep moveToPoint(Point point, int dir, QTime timeout);
    MotionStep moveToPoint(QLength x, QLength y, int dir, QTime timeout);
    MotionStep boomerang(Point point,
                         int dir,
                         QAngle final_heading,
                         double lead,
                         QTime timeout);
    MotionStep boomerang(QLength x,
                         QLength y,
                         int dir,
                         QAngle final_heading,
                         double lead,
                         QTime timeout);

  private:
    friend class Routine;

    MotionStep(Routine& routine, std::size_t index);
    MotionStep& rebuild();

    Routine* m_routine = nullptr;
    std::size_t m_index = 0;
  };

  Routine() = default;
  explicit Routine(ChassisController& chassis);

  Routine& setChassis(ChassisController& chassis);

  Routine& add(std::unique_ptr<Command> command);
  Routine& then(std::unique_ptr<Command> command);
  /**
   * @brief Start a command alongside the routine and move straight on.
   *
   * The step finishes on the tick it starts, so the triggered command keeps
   * running while the steps behind it run. The routine stops it when the
   * routine itself ends, whether or not it is marked mustRun(): mustRun() only
   * exempts it from the sweep the time budget's deadline policy makes, so it
   * keeps running through the remaining steps rather than past the end of the
   * routine. Nothing outlives runBlocking().
   *
   * A triggered command runs under the routine's reservation. The routine
   * reserves every subsystem its motion steps need, so a triggered command that
   * wants one of them - the chassis, most of all - shares it instead of
   * fighting for it:
   *
   * @code
   * routine.driveTo(24_in, 1_s)
   *        .trigger(chassis.makeCorrectHeadingCommand())
   *        .turnToAngle(90_deg, 800_ms);
   * @endcode
   *
   * Subsystems the routine does not hold are claimed the usual way, so
   * `.trigger(intake.spin())` still displaces the intake's default command.
   */
  Routine& trigger(std::unique_ptr<Command> command);
  Routine& runOnce(std::function<void()> action);
  Routine& wait(QTime duration);

  /**
   * @brief Wait for a predicate, giving up after @p timeout.
   *
   * @param condition Checked once per scheduler tick.
   * @param timeout Give up after this long. Not positive waits forever, which
   *   is what this used to do unconditionally; say so on purpose if you mean
   *   it. See kDefaultWaitUntilTimeout.
   *
   * @details Giving up advances to the next step, the same as the condition
   * coming true. timedOutWaits() and a line on the terminal are what tell the
   * two apart.
   */
  Routine& waitUntil(std::function<bool()> condition,
                     QTime timeout = kDefaultWaitUntilTimeout);

  // -------------------------------------------------------------------------
  // Time budget. See the class documentation.
  // -------------------------------------------------------------------------

  /// @brief Give the routine @p total to run in, with the default policy.
  Routine& withTimeBudget(QTime total = kAutonomousPeriod);
  /// @brief Give the routine a fully specified budget.
  Routine& withTimeBudget(const TimeBudgetConfig& config);
  /// @brief What the routine does when the budget runs out.
  Routine& withDeadlinePolicy(DeadlinePolicy policy);
  /**
   * @brief How much of the budget to hold back for the must-run steps.
   *
   * Held back out of the total, not added to it: with a 15 s budget and a
   * 1.5 s grace the driving stops at 13.5 s and the must-run steps run from
   * there to 15 s.
   */
  Routine& withGrace(QTime grace);
  /// @brief Remove the budget; the routine runs to completion however long.
  Routine& withoutTimeBudget();

  /**
   * @brief Mark the most recently added step must-run.
   *
   * @details Applies to the last step of any kind, so it reads the same after
   * a `runOnce()` as after a motion: `.runOnce(...).mustRun()`. On a
   * `trigger()` step it means something slightly different but consistent: the
   * triggered command keeps running through the deadline instead of being
   * cancelled with everything else - the intake the scoring step still needs.
   * Does nothing on an empty routine.
   */
  Routine& mustRun(bool must_run = true);

  /// @brief True when a positive budget is configured.
  bool hasTimeBudget() const;
  /// @brief The budget in force.
  const TimeBudget& timeBudget() const;
  /// @brief Time since the routine started. Zero before it does.
  QTime elapsed() const;
  /**
   * @brief Time left before the routine must stop.
   *
   * Counts down the budget, then the grace window once the budget has expired.
   * Zero when there is no budget - test hasTimeBudget() first.
   */
  QTime remaining() const;
  /// @brief True once the budget ran out and the deadline policy was applied.
  bool budgetExpired() const;
  /// @brief Number of steps the deadline policy skipped. Zero until it fires.
  std::size_t skippedSteps() const;

  /**
   * @brief How many `waitUntil()` steps gave up instead of seeing their
   *        condition come true.
   *
   * @details A wait that times out looks like a wait that succeeded - the
   * routine moves on either way - so this is the only thing that tells them
   * apart afterwards. Each one also prints a line to the terminal as it
   * happens. Non-zero means a mechanism did not do what the routine assumed.
   */
  std::size_t timedOutWaits() const;

  /**
   * @brief The timeout step @p index was last built with.
   *
   * @details The step's own timeout, or the shorter one the budget clamped it
   * to. Zero for a step that has none and for an index past the end.
   */
  QTime stepTimeout(std::size_t index) const;

  /**
   * @brief True when a motion was added with no chassis to run it on.
   *
   * @details Those motions become no-ops instead of taking the program down.
   * See the note on the missing-chassis path in the source.
   */
  bool hasConfigurationError() const;

  MotionStep driveDistance(ChassisController& chassis,
                           QLength distance,
                           QTime timeout = 0.0 * units::second);
  MotionStep driveDistance(QLength distance, QTime timeout = 0.0 * units::second);
  MotionStep turnToHeading(ChassisController& chassis,
                           QAngle heading,
                           QTime timeout = 0.0 * units::second);
  MotionStep turnToHeading(QAngle heading, QTime timeout = 0.0 * units::second);
  MotionStep turnToAngle(ChassisController& chassis, QAngle turn_angle, QTime timeout);
  MotionStep turnToAngle(QAngle turn_angle, QTime timeout);

  /**
   * @brief Drive a relative distance in a straight line.
   *
   * This used to be overloaded three ways: `driveTo(24, 1000)` drove 24 inches
   * forward while `driveTo(24, 36, 1000)` drove to the field point (24, 36) -
   * two different motions, told apart only by argument count. There is now one
   * `driveTo`, and it always means a distance. For a field point, say so:
   * `moveToPoint(Point{24_in, 36_in}, 1500_ms)`, which is what the coordinate
   * overloads dispatched to anyway.
   */
  MotionStep driveTo(ChassisController& chassis, QLength distance, QTime timeout);
  MotionStep driveTo(QLength distance, QTime timeout);

  MotionStep curveCircle(ChassisController& chassis,
                         QAngle result_angle,
                         QLength center_radius,
                         QTime timeout);
  MotionStep curveCircle(QAngle result_angle, QLength center_radius, QTime timeout);
  MotionStep curveCircleReverse(ChassisController& chassis,
                                QAngle result_angle,
                                QLength center_radius,
                                QTime timeout);
  MotionStep curveCircleReverse(QAngle result_angle,
                                QLength center_radius,
                                QTime timeout);
  MotionStep swing(ChassisController& chassis,
                   QAngle swing_angle,
                   double drive_direction,
                   QTime timeout);
  MotionStep swing(QAngle swing_angle, double drive_direction, QTime timeout);
  MotionStep wallReset(ChassisController& chassis,
                       QLength reset_x,
                       QLength reset_y,
                       QAngle reset_heading,
                       QTime timeout);
  MotionStep wallReset(QLength reset_x,
                       QLength reset_y,
                       QAngle reset_heading,
                       QTime timeout);
  MotionStep turnToPoint(ChassisController& chassis,
                         Point point,
                         QTime timeout = 1000.0 * units::millisecond);
  MotionStep turnToPoint(Point point, QTime timeout = 1000.0 * units::millisecond);
  MotionStep turnToPoint(ChassisController& chassis,
                         QLength x,
                         QLength y,
                         QTime timeout = 1000.0 * units::millisecond);
  MotionStep turnToPoint(QLength x,
                         QLength y,
                         QTime timeout = 1000.0 * units::millisecond);

  /// @brief Drive to a field point, direction taken from the step's options.
  MotionStep moveToPoint(ChassisController& chassis, Point point, QTime timeout);
  MotionStep moveToPoint(Point point, QTime timeout);
  MotionStep moveToPoint(ChassisController& chassis,
                         Point point,
                         int dir,
                         QTime timeout);
  MotionStep moveToPoint(Point point, int dir, QTime timeout);
  MotionStep moveToPoint(ChassisController& chassis,
                         QLength x,
                         QLength y,
                         int dir,
                         QTime timeout);
  MotionStep moveToPoint(QLength x, QLength y, int dir, QTime timeout);
  MotionStep boomerang(ChassisController& chassis,
                       Point point,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);
  MotionStep boomerang(Point point,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);
  MotionStep boomerang(ChassisController& chassis,
                       QLength x,
                       QLength y,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);
  MotionStep boomerang(QLength x,
                       QLength y,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);

  void clear();
  bool empty() const;
  std::size_t size() const;

  /**
   * @brief Stop earlier trigger()s that share a subsystem with a starting one.
   *
   * An internal hook the trigger() steps call, not part of the building API.
   * A triggered command runs under this routine's reservation, which means the
   * CommandScheduler never sees it claim the subsystems the routine holds and
   * so cannot arbitrate them. This does that arbitration instead, with the
   * scheduler's own CancelRunning rule: the trigger starting now wins.
   *
   * @param requester The trigger step that is starting. Left alone.
   * @param subsystems The subsystems hidden from the scheduler for it.
   */
  void cancelTriggersSharing(const Command* requester,
                             const std::vector<Subsystem*>& subsystems);
  void runBlocking(std::uint32_t period_ms = 10);

  void initialize() override;
  void execute() override;
  bool isFinished() override;
  void end(bool interrupted) override;
  std::vector<Subsystem*> getRequirements() override;

private:
  /**
   * @brief The per-step knobs the `withXxx()` setters write.
   *
   * `max_voltage` was called `max_speed`, and `withMaxSpeed()` was literally
   * `return withMaxVoltage(...)`. It has always been volts - it lands in
   * `MotorGroup::setVoltage()` - so it is now named and typed for what it is.
   * The same goes for `min_voltage`, formerly `min_speed`.
   */
  struct MotionOptions {
    QVoltage max_voltage = 12.0 * units::volt;
    /// @brief Negative selects the `min_output` default. See motion.hpp.
    QVoltage min_voltage = -1.0 * units::volt;
    bool exit = true;
    bool stop_at_end = true;
    int direction = 1;
    bool reverse = false;
    bool overturn = true;
    /// @brief Negative drives backward into the wall, which is the usual case.
    QVoltage drive_power = -4.0 * units::volt;
    QCurrent current_threshold = 2500.0 * units::milliampere;
    QAngularVelocity velocity_threshold = 5.0 * units::rpm;
    /**
     * @brief The timeout the command is built with. Zero means none.
     *
     * @details Here rather than captured in the factory so the time budget can
     * shorten it: `initializeCurrent()` writes the clamped value and rebuilds
     * the command, and the motion then honours the shorter timeout itself,
     * settling and stopping the way it would at any other timeout.
     */
    QTime timeout{};
  };

  using MotionFactory =
      std::function<std::unique_ptr<Command>(const MotionOptions&)>;

  struct Step {
    std::unique_ptr<Command> command;
    bool reserve_requirements = true;
    MotionFactory factory;
    MotionOptions options;
    /**
     * @brief The timeout the step was written with, before any clamping.
     *
     * `options.timeout` is what the command was last built with, which the
     * budget may have shortened. This one never moves, so a step that runs
     * again is clamped against its own number, not against the clamp.
     */
    QTime timeout{};
    /// @brief Run this step even after the budget expires. See mustRun().
    bool must_run = false;
    /// @brief This step is a trigger(), whose inner command outlives it.
    bool is_trigger = false;
    /// @brief This step is a waitUntil(), which can end by giving up.
    bool is_wait_until = false;
  };

  MotionStep addMotion(MotionFactory factory, QTime timeout);
  MotionStep addMotion(MotionFactory factory,
                       MotionOptions options,
                       QTime timeout);
  /// @brief The placeholder step a motion becomes when there is no chassis.
  MotionStep addMissingChassisStep();
  void rebuildMotion(std::size_t index);
  void initializeCurrent();
  /// @brief End the running step as interrupted and drop it.
  void stopCurrentStep();
  /// @brief Put the drive at rest, whatever the interrupted step asked for.
  void brakeDrive();
  /// @brief Move to the next step, skipping the skippable ones after expiry.
  void advanceStep();
  /// @brief The next step to run from @p from, under the deadline policy.
  std::size_t nextRunnableStep(std::size_t from) const;
  /// @brief Apply the deadline policy. Called once, when the budget runs out.
  void applyDeadlinePolicy();
  /**
   * @brief Cancel the fire-and-forget commands trigger() left running.
   * @param keep_must_run Leave the triggers marked mustRun() running.
   */
  void cancelTriggeredCommands(bool keep_must_run = false);

  /// @brief Add the steps between the cursor and @p next to the skipped count.
  void countSkipped(std::size_t next);
  /// @brief Record and report a waitUntil() step that gave up.
  void noteWaitTimeout(Command& command);
  static void mergeRequirements(std::vector<Subsystem*>& requirements,
                                const std::vector<Subsystem*>& next);

  ChassisController* m_chassis = nullptr;
  std::vector<Step> m_steps;
  std::size_t m_index = 0;
  bool m_current_initialized = false;
  TimeBudget m_budget{};
  /// @brief Set once applyDeadlinePolicy() has run, so it runs only once.
  bool m_deadline_applied = false;
  std::size_t m_skipped_steps = 0;
  std::size_t m_timed_out_waits = 0;
  bool m_configuration_error = false;
};

using AutonomousRoutine = Routine;

}  // namespace auton
}  // namespace mclib
