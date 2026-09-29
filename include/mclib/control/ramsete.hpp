// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/feedforward.hpp"
#include "mclib/math.hpp"
#include "mclib/path/pure_pursuit.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/units/units.hpp"

/**
 * @file ramsete.hpp
 * @brief A trajectory follower for tank drives that steers on the odometry
 *        pose.
 *
 * `driveTo()` and `curveCircle()` measure progress with the drive encoders.
 * When a wheel slips, the encoder still counts, so the robot misses. RAMSETE
 * compares the odometry pose with where the trajectory says the robot should
 * be, and corrects both speed and turn rate toward it.
 *
 * ## The control law
 *
 * With the error to the target in the robot's own frame (`e_x` forward,
 * `e_y` to the right, `e_theta` clockwise) and the target's speed `v_d` and
 * turn rate `w_d`:
 *
 * ```
 * k = 2 * zeta * sqrt(w_d^2 + b * v_d^2)
 * v = v_d * cos(e_theta) + k * e_x
 * w = w_d + k * e_theta + b * v_d * sinc(e_theta) * e_y
 * ```
 *
 * This is the usual RAMSETE law. It is written for the counter-clockwise
 * frame with `e_y` to the left; flipping both the turn direction and the side
 * leaves every term unchanged, so the same formula works in mclib's
 * clockwise compass frame.
 *
 * `b` and `zeta` are in SI units, the same as WPILib: `b` in rad²/m² and
 * `zeta` in 1/rad. The defaults, 2.0 and 0.7, are the usual starting point.
 *
 * `k` is zero when the target is standing still, so RAMSETE cannot correct a
 * robot that has stopped off target. It only corrects while the trajectory is
 * moving.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace control {

/// @brief RAMSETE tuning, in SI units.
struct RamseteGains {
  /// @brief How hard to correct, rad²/m². Larger pulls the robot back onto
  ///        the path faster. Must be positive.
  double b = 2.0;
  /// @brief Damping, 1/rad. Between 0 and 1. Larger overshoots less.
  double zeta = 0.7;
};

/// @brief Everything `followTrajectory()` needs besides the trajectory.
struct RamseteConfig {
  RamseteGains gains{};
  /// @brief Feedforward for **one side** of the drive: volts for a wheel
  ///        speed and acceleration. Measure it as in docs/motion.md. `kV`
  ///        must be positive; without feedforward nothing drives the robot.
  FeedforwardGains feedforward{};
};

/// @brief Speed and turn rate for the robot's centre.
struct RamseteOutput {
  units::QVelocity velocity{};
  /// @brief Clockwise-positive.
  units::QAngularVelocity angular_velocity{};
};

/// @brief One voltage per side of a tank drive.
struct DriveVoltages {
  units::QVoltage left{};
  units::QVoltage right{};
};

/**
 * @brief The RAMSETE control law. Stateless.
 */
class Ramsete {
 public:
  explicit Ramsete(RamseteGains gains = {}) : m_gains(gains) {}

  const RamseteGains& gains() const { return m_gains; }

  /**
   * @brief Speed and turn rate that steer from @p pose toward @p target.
   * @param pose Odometry pose: inches, radians, compass frame.
   * @param target The trajectory sample for this tick.
   */
  RamseteOutput calculate(const Pose2D& pose,
                          const path::TrajectoryState& target) const;

 private:
  RamseteGains m_gains;
};

/**
 * @brief Split a speed and turn rate onto the two sides of a tank drive.
 *
 * Turning clockwise (positive) speeds up the left side.
 */
path::WheelSpeeds tankWheelSpeeds(const RamseteOutput& command,
                                  units::QLength track_width);

/**
 * @brief Voltages for a RAMSETE command.
 *
 * Each side gets `feedforward.calculate(side speed, acceleration)`. When
 * either side is over @p max_voltage, both are scaled down by the same
 * factor so the robot keeps the same curve.
 *
 * @param acceleration The trajectory's acceleration at this tick. Applied to
 *        both sides.
 */
DriveVoltages ramseteVoltages(const RamseteOutput& command,
                              units::QAcceleration acceleration,
                              units::QLength track_width,
                              const SimpleMotorFeedforward& feedforward,
                              units::QVoltage max_voltage);

}  // namespace control
}  // namespace mclib
