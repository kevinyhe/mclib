// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/math.hpp"

/**
 * @file slide_estimator.hpp
 * @brief How far the robot slides sideways, worked out from the IMU's
 *        accelerometer.
 *
 * Odometry with no sideways tracking wheel assumes the robot never moves
 * sideways. An omni drive does: in a fast curve the rollers can't supply the
 * pull the turn needs, and the robot slides outward. In the physics simulator
 * that is 4 to 10 in on a 90 degree, 24 in arc, all of it missing from the
 * odometry.
 *
 * The accelerometer feels the sideways push. Take away the part the turn
 * explains (turn rate times forward speed) and what is left changes the
 * sideways speed. Adding that up gives the slide.
 *
 * Adding up an accelerometer also adds up its error. The IMU chip's zero can
 * be off by 40 mg once soldered, and a floor tilted 0.2 degrees adds 3.5 mg
 * more. 40 mg added up for one second is 15 in/s. So this is a small Kalman
 * filter over two numbers, the sideways speed and the accelerometer's zero:
 *
 * - Until the robot first moves, the zero is the average reading. Odometry
 *   should start with the robot still, as calibrating the IMU needs anyway.
 * - Each tick it adds up the accelerometer, less its zero, into the speed.
 * - Each tick it also assumes the robot is not sliding, as sure of that as
 *   the sideways push is small. Standing still or driving straight, that
 *   holds the speed at zero and teaches the filter the zero. In a fast turn,
 *   or when something shoves the robot, the assumption is weak and the
 *   accelerometer decides.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace control {

/**
 * @brief Where the accelerometer is and how noisy it is.
 */
struct SlideEstimatorConfig {
  /// @brief IMU position ahead of the tracking centre, inches.
  double imu_forward_in = 0.0;
  /// @brief IMU position to the right of the tracking centre, inches.
  double imu_right_in = 0.0;
  /// @brief How well that position is measured, inches. Spinning at 6 rad/s,
  ///        an IMU 0.25 in from where it is said to be reads 9 in/s^2 that
  ///        neither the turn nor a slide explains.
  double imu_position_sigma_in = 0.25;
  /// @brief Noise on one reading, in/s^2. 10 is about 25 mg: the chip itself
  ///        has 1 mg; the rest is the drive shaking it.
  double accel_sigma_ips2 = 10.0;
  /// @brief How far off the zero may be before the robot has stood still,
  ///        in/s^2. 15 is about 40 mg, the chip's tolerance once soldered.
  double zero_sigma_ips2 = 15.0;
  /// @brief How much the zero may change per square-root inch driven, in/s^2:
  ///        the floor is not level everywhere.
  double zero_walk_ips2_per_sqrt_in = 0.7;
  /// @brief How fast the robot may slide sideways with nothing pushing it
  ///        sideways, in/s.
  double grip_sigma_ips = 0.5;
  /// @brief How much faster it may slide per in/s^2 of sideways push,
  ///        seconds. The push is the pull the turn needs plus whatever the
  ///        accelerometer feels that neither the turn nor the error in the
  ///        IMU position could explain.
  double slide_sigma_s = 1.0;
};

/// @brief True when every number in @p config is finite, every sigma and
///        walk is positive, and `imu_position_sigma_in` is not negative.
bool slideEstimatorConfigIsValid(const SlideEstimatorConfig& config);

/**
 * @brief Estimates the sideways speed of the tracking centre. Not
 *        thread-safe; `Odometry` owns one.
 */
class SlideEstimator {
 public:
  explicit SlideEstimator(SlideEstimatorConfig config = {});

  /// @brief Forget the speed and the zero.
  void reset();

  /**
   * @brief One tick.
   * @param dt_s Seconds since the last tick. Zero or less changes nothing.
   * @param forward_in How far the tracking centre went forward this tick.
   * @param turn_rad How far the robot turned this tick, clockwise positive.
   * @param accel_right_ips2 The accelerometer's reading toward the robot's
   *        right, in/s^2.
   * @return How far the tracking centre slid toward the robot's right this
   *         tick, inches.
   */
  double update(double dt_s, double forward_in, double turn_rad, double accel_right_ips2);

  /// @brief Sideways speed of the tracking centre, in/s, right positive.
  double sidewaysSpeedIps() const { return m_speed; }
  /// @brief What the accelerometer reads toward the right when nothing
  ///        accelerates the robot, in/s^2.
  double accelZeroIps2() const { return m_zero; }

 private:
  SlideEstimatorConfig m_config;
  double m_speed = 0.0;
  double m_zero = 0.0;
  Mat2 m_cov = Mat2::Zero();
  double m_last_rate = 0.0;
  int m_still_readings = 0;
  bool m_moved = false;
};

}  // namespace control
}  // namespace mclib
