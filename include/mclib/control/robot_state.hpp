// mclib
#pragma once

#include "mclib/math.hpp"
#include "mclib/sync.hpp"
#include "mclib/units/units.hpp"

/**
 * @file robot_state.hpp
 * @brief The shared mutable state of the robot, behind one lock.
 *
 * This replaces six bare globals (`xpos`, `ypos`, `correct_angle`,
 * `is_turning`, `prev_left_output`, `prev_right_output`) that three contexts
 * wrote at the same time with no synchronisation at all: the odometry task at
 * 100 Hz, the blocking motion routines on the "mclib chassis" task, and the
 * command scheduler.
 *
 * ## Why a mutex and not a seqlock
 *
 * A seqlock is tempting - one 100 Hz writer, several readers, no writer
 * blocking. It is the wrong fit here for three reasons:
 *
 * 1. There is more than one writer. The odometry task writes the pose every
 *    tick, and `wallReset()` / `Chassis::setPose()` write it from the motion
 *    task. A seqlock with two writers needs a writer lock anyway, so the
 *    mutex does not disappear, it just gets a second mechanism stacked on it.
 * 2. A seqlock reader spins until the writer's sequence number settles. PROS
 *    runs FreeRTOS on a single core. A higher-priority reader that preempts
 *    the writer mid-update spins forever on a counter only the preempted
 *    writer can advance - a livelock, not a slow path.
 * 3. The critical section is six scalar stores. At 100 Hz that is a few
 *    microseconds per second of lock occupancy. `pros::Mutex` is a FreeRTOS
 *    mutex with priority inheritance, which is precisely the primitive for a
 *    short critical section shared across priorities.
 *
 * ## Consistency
 *
 * `pose()` returns x, y and theta from a single locked read, so a caller can
 * never pair an x from one odometry tick with a y from the next. Every
 * expression in `motion.cpp` that used to read `xpos` and `ypos` as two
 * separate loads now takes one `Pose2D` first.
 */

namespace mclib {
namespace control {

/**
 * @brief Robot pose plus the scalars the blocking motion routines share.
 *
 * Every accessor takes the lock. Copy what you need into locals and work on
 * the copy; do not call back into RobotState in a hot expression.
 */
class RobotState {
 public:
  RobotState() = default;

  RobotState(const RobotState&) = delete;
  RobotState& operator=(const RobotState&) = delete;

  /**
   * @brief The current field pose, read atomically.
   * @return x and y in inches and theta in radians, all from the same update.
   */
  Pose2D pose() const;

  /// @brief Replace the whole pose.
  void setPose(const Pose2D& pose);

  /**
   * @brief Replace x and y, leaving theta alone.
   *
   * What `wallReset()` wants: the wall fixes the position, the IMU still owns
   * the heading.
   */
  void setPosition(double x_in, double y_in);

  /// @brief Field X in inches. Prefer pose() when you also need y.
  double x() const;
  /// @brief Field Y in inches. Prefer pose() when you also need x.
  double y() const;
  /// @brief Heading in radians, compass frame. Prefer pose() when you also need x/y.
  double headingRad() const;

  /// @brief The current pose position as a unit-typed pair.
  QLength xLength() const;
  /// @brief @copydoc xLength
  QLength yLength() const;
  /// @brief The current heading as a QAngle.
  QAngle heading() const;

  /// @brief True while a blocking motion routine is turning the robot.
  bool isTurning() const;
  /// @brief @copydoc isTurning
  void setTurning(bool turning);

  /**
   * @brief The heading, in degrees, that `correctHeading()` holds between moves.
   *
   * Degrees, not radians: this is the public motion API's unit and every
   * caller in `motion.cpp` is in degrees. See the units note in math.hpp.
   */
  double correctAngleDeg() const;
  /// @brief @copydoc correctAngleDeg
  void setCorrectAngleDeg(double angle_deg);

  /// @brief Last commanded left drive voltage, for the slew limiter.
  double prevLeftOutput() const;
  /// @brief Last commanded right drive voltage, for the slew limiter.
  double prevRightOutput() const;
  /// @brief Store both slew-limiter outputs in one locked write.
  void setPrevOutputs(double left, double right);

  /**
   * @brief Put the motion-owned scalars back to rest.
   *
   * Called when a motion routine is cancelled, so the next one does not
   * inherit a stale slew value or a stuck `is_turning`. Does not touch the
   * pose - the robot is still where it is.
   */
  void clearMotionOutputs();

 private:
  mutable sync::Mutex m_mutex;
  Pose2D m_pose{};
  bool m_is_turning = false;
  double m_correct_angle_deg = 0.0;
  double m_prev_left_output = 0.0;
  double m_prev_right_output = 0.0;
};

/**
 * @brief The process-wide robot state.
 *
 * A function-local static so there is no static-initialisation-order question
 * between this and the mechanisms constructed at namespace scope in
 * `config.cpp`.
 */
RobotState& robotState();

// ---------------------------------------------------------------------------
// Cooperative cancellation
//
// The blocking routines in motion.cpp are `while (timeout) { ... delay(10); }`
// loops running on their own task. They used to be stopped with
// pros::Task::remove(), a hard kill that can land between two stores and
// leaves the shared scalars arbitrary. Instead every loop now also tests
// cancelRequested(), so it exits at a delay boundary with its state intact.
// ---------------------------------------------------------------------------

/**
 * @brief Which family of routines a cancel request applies to.
 *
 * The routines being cancelled are free functions with no handle to check, so
 * the flags are process-wide rather than per-command. One flag would be wrong:
 * `correctHeading()` is built to run *alongside* a motion - it gates on
 * `isTurning()` precisely so it can - so a single shared flag would silently
 * kill the heading hold every time any other routine was interrupted, and
 * nothing would restart it. Splitting the two keeps them independent.
 *
 * Within a family the flag is still shared, so do not run two blocking motion
 * routines at once. They would be fighting over the same drive motors anyway.
 */
enum class CancelToken {
  /// @brief The blocking motions: driveTo, turnToAngle, moveToPoint, and so on.
  Motion,
  /// @brief The long-running correctHeading() loop.
  HeadingCorrection,
};

/// @brief True when routines of @p token have been asked to stop.
bool cancelRequested(CancelToken token = CancelToken::Motion);

/// @brief Ask routines of @p token to stop at their next loop boundary.
void requestCancel(CancelToken token = CancelToken::Motion);

/// @brief Clear @p token's flag. Call before starting a new routine.
void clearCancel(CancelToken token = CancelToken::Motion);

}  // namespace control
}  // namespace mclib
