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
 * - An IMU off the tracking centre also feels the turn itself. That is taken
 *   away using where the IMU is. What an error of `imu_position_sigma_in` in
 *   that position could read doesn't count as a shove.
 *
 * The pull a turn needs is turn rate times forward speed. Drive wheels count
 * too much forward speed when they slip, as they do when a robot comes out of
 * a hard turn at full power, and that would read as a slide. So with forward
 * travel from the drive wheels, a second filter of the same kind works out
 * the forward speed from the accelerometer's forward axis, held to the
 * wheels' speed while they grip. A wheel speed further from the filter's
 * than the noise explains is slip, and counts for less the further off it
 * is. The difference between that forward speed and the wheels' is added to
 * the forward travel.
 *
 * The heading gives the turn rate averaged over a tick, half a tick behind
 * the accelerometer, and how fast the rate changes a whole tick behind. An
 * IMU 3 in off the centre feels 3 in times that change: 180 in/s^2 as a
 * spin stops. So each tick works on the previous tick's accelerometer
 * reading, which lines up with the heading. The slide and the correction to
 * the forward travel arrive one tick late; the wheels' own travel does not.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace control {

/**
 * @brief Where the accelerometer is, how noisy it is, and how far to trust
 *        the drive wheels.
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
  /// @brief How far the drive wheels' forward speed may be from the robot's
  ///        while they grip, in/s.
  double wheel_sigma_ips = 0.5;
  /// @brief How much further per in/s that the wheels' speed is off the
  ///        filter's, beyond what the noise explains. 2: wheels 5 in/s off
  ///        count as 10 in/s uncertain, and barely move the speed.
  double wheel_slip_sigma_ratio = 2.0;
};

/// @brief True when every number in @p config is finite, every sigma and
///        walk is positive, and `imu_position_sigma_in` and
///        `wheel_slip_sigma_ratio` are not negative.
bool slideEstimatorConfigIsValid(const SlideEstimatorConfig& config);

/// @brief Where odometry's forward travel comes from.
enum class ForwardTravel {
  /// @brief The drive wheels, which slip. The accelerometer's forward axis
  ///        corrects them.
  FromDriveWheels,
  /// @brief A forward tracking wheel, which doesn't slip. The forward axis
  ///        is not read.
  FromTrackingWheel,
};

/**
 * @brief Estimates the forward and sideways speed of the tracking centre.
 *        Not thread-safe; `Odometry` owns one.
 */
class SlideEstimator {
 public:
  explicit SlideEstimator(SlideEstimatorConfig config = {},
                          ForwardTravel forward_travel = ForwardTravel::FromDriveWheels);

  /// @brief Forget the speeds and the zeros.
  void reset();

  /// @brief How far the tracking centre went in one tick, inches.
  struct Travel {
    double forward_in;
    double right_in;
  };

  /**
   * @brief One tick.
   * @param dt_s Seconds since the last tick. Zero or less changes nothing.
   * @param forward_in How far the drive wheels or tracking wheel say the
   *        tracking centre went forward this tick.
   * @param turn_rad How far the robot turned this tick, clockwise positive.
   * @param accel_right_ips2 The accelerometer's reading toward the robot's
   *        right, in/s^2.
   * @param accel_forward_ips2 Its reading toward the robot's front, in/s^2.
   *        Not read with `ForwardTravel::FromTrackingWheel`.
   * @return The forward travel, @p forward_in plus the last tick's
   *         correction when it came from the drive wheels, and the last
   *         tick's sideways slide, right positive.
   */
  Travel update(double dt_s, double forward_in, double turn_rad, double accel_right_ips2,
                double accel_forward_ips2);

  /// @brief Sideways speed of the tracking centre, in/s, right positive.
  double sidewaysSpeedIps() const { return m_right.speed; }
  /// @brief Forward speed of the tracking centre, in/s.
  double forwardSpeedIps() const { return m_forward.speed; }
  /// @brief What the accelerometer reads toward the right when nothing
  ///        accelerates the robot, in/s^2.
  double accelZeroIps2() const { return m_right.zero; }
  /// @brief The same toward the front.
  double forwardAccelZeroIps2() const { return m_forward.zero; }

 private:
  /// @brief The speed along one axis and the accelerometer's zero on it,
  ///        with their covariance.
  struct Axis {
    double speed = 0.0;
    double zero = 0.0;
    Mat2 cov = Mat2::Zero();

    void reset(double zero_sigma);
    void averageStillReading(double reading, int count, double accel_sigma);
    void predict(double accel, double dt_s, double speed_var, double zero_var);
    void measureSpeed(double measured, double sigma);
  };

  /// @brief One tick's readings.
  struct Tick {
    double dt_s;
    double wheel_speed;
    double rate;
    double accel_right;
    double accel_forward;
  };

  /// @brief Until the robot first moves, the zeros are the average readings.
  ///        A reading far off that average is the robot starting to move
  ///        before its wheels and heading show it. True while still.
  bool standingStill(double forward_in, double turn_rad, double accel_right_ips2,
                     double accel_forward_ips2);

  /// @brief Runs the filters over @p last, with @p next for the turn rate at
  ///        its end. Returns what to add to the wheels' forward travel over
  ///        @p last, and the sideways travel over it.
  Travel filter(const Tick& last, const Tick& next);

  SlideEstimatorConfig m_config;
  ForwardTravel m_forward_travel;
  Axis m_right;
  Axis m_forward;
  Tick m_last{};
  bool m_has_last = false;
  int m_still_readings = 0;
  bool m_moved = false;
};

}  // namespace control
}  // namespace mclib
