// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/feedforward.hpp"
#include "mclib/control/profile.hpp"
#include "mclib/units/units.hpp"

/**
 * @file profiled_moves.hpp
 * @brief Drive the stopping moves on a planned speed instead of a distance PID.
 *
 * By default `driveTo()`, `turnToAngle()`, `swing()` and `moveToPoint()` set
 * their voltage from a PID on the distance or angle left. The voltage shrinks
 * with the error, so the robot slows for the whole last part of every move.
 * After `useProfiledMoves()`, each of them with `exit` true plans its speed
 * instead: speed up at `limits.max_acceleration`, cruise, and slow down at
 * `limits.max_deceleration` to stop on the target. The voltage is the
 * feedforward for the planned wheel speed and acceleration, plus `kp` on
 * how far the wheels are behind the plan and `kd` on how much slower than
 * the plan they went over the last 50 ms.
 *
 * - `driveTo()` plans the distance. Once the plan ends, its distance PID
 *   settles as before.
 * - `turnToAngle()` plans the arc each wheel rolls, the turn times half of
 *   `track_width`. Once the plan ends, its turn PID settles as before.
 * - `swing()` plans the arc the moving side rolls, the turn times
 *   `track_width`. Once the plan ends, its turn PID settles as before.
 * - `moveToPoint()` sets its speed each tick from the distance left along
 *   its heading: no faster than it can stop from at
 *   `limits.max_deceleration`, and no more than `limits.max_acceleration`
 *   faster than the tick before. It first turns in place until it faces
 *   within 30 degrees of the target, not 60: steering more than that at
 *   speed slid the robot sideways. It turns in place only below the speed
 *   it stops from within `distance_exit.small_error`, and brakes at 0 V
 *   until then. Its `min_speed` floor stops once the plan has to slow down
 *   or once it turns in place. Inside `distance_exit.small_error` its
 *   distance PID settles as before.
 *
 * `driveTo()` and `moveToPoint()` plan from the robot's forward speed when
 * they start, the mean of `DriveHardware::driveVelocitiesRpm()`, no faster
 * than `limits.max_velocity`: a move chained onto a moving robot starts at
 * its speed, and one after a turn starts from rest.
 *
 * Chained moves (`exit` false), `curveCircle()`, `turnToPoint()`,
 * `boomerang()` and `followTrajectory()` run as before.
 */

namespace mclib::control {

/**
 * @brief Measured drive model, speed limits and gains for the profiled moves.
 */
struct ProfiledMoveConfig {
  /**
   * @brief Feedforward and gains on how far the wheels are behind the plan.
   * @details `follower.gains` is the feedforward for **one side** of the
   * drive, measured as in `docs/motion.md`: the same numbers as
   * `RamseteConfig::feedforward`. `kV` must be positive. `kp` works on the
   * inches a wheel is behind the plan; `kd`, in volts per in/s, on how much
   * slower than the plan it went over the last 50 ms; `follower.max_voltage` is
   * replaced by each move's `max_output`.
   */
  ProfileFollowerConfig follower{};
  /**
   * @brief Feedforward for one side while the robot turns in place.
   * @details A turn scrubs the wheels sideways, so it takes more voltage at
   * low speed than driving straight. Fit it like `follower.gains`, from spins
   * in place at two or more voltages, with the wheel speed taken as the turn
   * rate in rad/s times half of `track_width`. `kV` must be positive.
   * `turnToAngle()` and `swing()` use it.
   */
  FeedforwardGains turn_gains{};
  /**
   * @brief Wheel speed and acceleration limits.
   * @details `max_velocity` and `max_acceleration` must be positive. A move
   * with a lower `max_output` plans no faster than the feedforward says that
   * voltage drives. A positive `max_jerk` plans S-curves for moves that start
   * at rest.
   */
  ProfileConstraints limits{};
  /**
   * @brief Track width that turns a turn rate into wheel speed.
   * @details 0 uses the drive geometry's. A skid-steer drive turns slower
   * than its real track width predicts; use the effective width measured as
   * for `RamseteConfig::track_width`.
   */
  units::QLength track_width{};
};

/**
 * @brief Plan the speed of `driveTo()`, `turnToAngle()`, `swing()` and
 *        `moveToPoint()` when they stop at the end. See the file comment.
 * @return False, with the reason printed, when @p config has a `kV` that is
 *         not positive, a `max_velocity` or `max_acceleration` that is not
 *         positive, or a gain, limit or track width that is negative or not
 *         finite. The moves are then left as they were.
 */
bool useProfiledMoves(const ProfiledMoveConfig& config);

/// @brief Go back to the PID moves.
void usePidMoves();

/**
 * @brief Why @p config can't be used, or nullptr when it can.
 */
const char* profiledMoveConfigProblem(const ProfiledMoveConfig& config);

}  // namespace mclib::control
