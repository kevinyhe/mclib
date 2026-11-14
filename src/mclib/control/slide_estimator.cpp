// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/slide_estimator.hpp"

#include <cmath>

namespace mclib {
namespace control {

namespace {
// How many standard deviations of the IMU position error a reading may be
// off before it counts as a push.
constexpr double kPositionErrorBound = 3.0;
// A reading this many accel_sigma off the average while the wheels are
// still is the robot starting to move.
constexpr double kStillReadingBound = 4.0;
// How many standard deviations the wheels' speed may be off the filter's
// before the rest counts as slip.
constexpr double kWheelSpreadBound = 2.0;
}  // namespace

bool slideEstimatorConfigIsValid(const SlideEstimatorConfig& config) {
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  return std::isfinite(config.imu_forward_in) && std::isfinite(config.imu_right_in) &&
         std::isfinite(config.imu_position_sigma_in) && config.imu_position_sigma_in >= 0.0 &&
         positive(config.accel_sigma_ips2) && positive(config.zero_sigma_ips2) &&
         positive(config.zero_walk_ips2_per_sqrt_in) && positive(config.grip_sigma_ips) &&
         positive(config.slide_sigma_s) && positive(config.wheel_sigma_ips) &&
         std::isfinite(config.wheel_slip_sigma_ratio) && config.wheel_slip_sigma_ratio >= 0.0;
}

void SlideEstimator::Axis::reset(double zero_sigma) {
  speed = 0.0;
  zero = 0.0;
  cov = Mat2::Zero();
  cov(1, 1) = zero_sigma * zero_sigma;
}

void SlideEstimator::Axis::averageStillReading(double reading, int count, double accel_sigma) {
  zero += (reading - zero) / count;
  cov(1, 1) = accel_sigma * accel_sigma / count;
}

void SlideEstimator::Axis::predict(double accel, double dt_s, double speed_var, double zero_var) {
  speed += accel * dt_s;
  Mat2 F = Mat2::Identity();
  F(0, 1) = -dt_s;
  Mat2 Q = Mat2::Zero();
  Q(0, 0) = speed_var;
  Q(1, 1) = zero_var;
  cov = F * cov * F.transpose() + Q;
}

void SlideEstimator::Axis::measureSpeed(double measured, double sigma) {
  const double innovation_var = cov(0, 0) + sigma * sigma;
  const double gain_speed = cov(0, 0) / innovation_var;
  const double gain_zero = cov(1, 0) / innovation_var;
  const double innovation = measured - speed;
  speed += gain_speed * innovation;
  zero += gain_zero * innovation;
  Mat2 I_KH = Mat2::Identity();
  I_KH(0, 0) -= gain_speed;
  I_KH(1, 0) -= gain_zero;
  Mat2 KKt;
  KKt << gain_speed * gain_speed, gain_speed * gain_zero, gain_speed * gain_zero,
      gain_zero * gain_zero;
  cov = I_KH * cov * I_KH.transpose() + KKt * sigma * sigma;
}

SlideEstimator::SlideEstimator(SlideEstimatorConfig config, ForwardTravel forward_travel)
    : m_config(config), m_forward_travel(forward_travel) {
  reset();
}

void SlideEstimator::reset() {
  m_right.reset(m_config.zero_sigma_ips2);
  m_forward.reset(m_config.zero_sigma_ips2);
  m_has_last = false;
  m_still_readings = 0;
  m_moved = false;
}

bool SlideEstimator::standingStill(double forward_in, double turn_rad, double accel_right_ips2,
                                   double accel_forward_ips2) {
  if (m_moved) return false;
  const bool uses_forward_axis = m_forward_travel == ForwardTravel::FromDriveWheels;
  const double bound = kStillReadingBound * m_config.accel_sigma_ips2;
  const bool wheels_still = forward_in == 0.0 && turn_rad == 0.0;
  const bool readings_still =
      m_still_readings == 0 ||
      (std::fabs(accel_right_ips2 - m_right.zero) < bound &&
       (!uses_forward_axis || std::fabs(accel_forward_ips2 - m_forward.zero) < bound));
  if (!(wheels_still && readings_still)) {
    m_moved = true;
    return false;
  }
  ++m_still_readings;
  m_right.averageStillReading(accel_right_ips2, m_still_readings, m_config.accel_sigma_ips2);
  if (uses_forward_axis) {
    m_forward.averageStillReading(accel_forward_ips2, m_still_readings,
                                  m_config.accel_sigma_ips2);
  }
  return true;
}

SlideEstimator::Travel SlideEstimator::update(double dt_s, double forward_in, double turn_rad,
                                              double accel_right_ips2,
                                              double accel_forward_ips2) {
  if (!(dt_s > 0.0)) return {forward_in, 0.0};
  const Tick tick{dt_s, forward_in / dt_s, turn_rad / dt_s, accel_right_ips2, accel_forward_ips2};
  // Calibrating the IMU needs the robot still, so odometry starts still.
  const bool still = standingStill(forward_in, turn_rad, accel_right_ips2, accel_forward_ips2);
  const bool has_last = m_has_last;
  const Tick last = m_last;
  m_last = tick;
  m_has_last = true;
  if (still || !has_last) return {forward_in, 0.0};
  const Travel last_tick = filter(last, tick);
  return {forward_in + last_tick.forward_in, last_tick.right_in};
}

SlideEstimator::Travel SlideEstimator::filter(const Tick& last, const Tick& next) {
  const double dt_s = last.dt_s;
  const double rate = 0.5 * (last.rate + next.rate);
  const double spin_up = (next.rate - last.rate) / (0.5 * (last.dt_s + next.dt_s));
  const double wheel_speed = 0.5 * (last.wheel_speed + next.wheel_speed);

  const double position_error_accel =
      m_config.imu_position_sigma_in * (rate * rate + std::fabs(spin_up));
  const double accel_var = std::pow(m_config.accel_sigma_ips2 * dt_s, 2) +
                           std::pow(position_error_accel * dt_s, 2);
  const double zero_var =
      std::pow(m_config.zero_walk_ips2_per_sqrt_in, 2) * std::fabs(last.wheel_speed * dt_s);

  // An IMU off the tracking centre also feels the turn itself: the pull
  // toward the centre of the spin, and the push of the spin speeding up.
  const double centre_accel = last.accel_right - m_right.zero +
                              rate * rate * m_config.imu_right_in -
                              m_config.imu_forward_in * spin_up;
  const double forward_before = m_forward.speed;
  if (m_forward_travel == ForwardTravel::FromDriveWheels) {
    const double body_forward_accel =
        last.accel_forward - m_forward.zero + rate * m_right.speed +
        spin_up * m_config.imu_right_in + rate * rate * m_config.imu_forward_in;
    m_forward.predict(body_forward_accel, dt_s, accel_var, zero_var);
    const double expected_spread =
        std::sqrt(m_forward.cov(0, 0) + m_config.wheel_sigma_ips * m_config.wheel_sigma_ips);
    const double slip = std::fmax(0.0, std::fabs(wheel_speed - m_forward.speed) -
                                           kWheelSpreadBound * expected_spread);
    m_forward.measureSpeed(wheel_speed,
                           m_config.wheel_sigma_ips + m_config.wheel_slip_sigma_ratio * slip);
  } else {
    m_forward.speed = wheel_speed;
  }

  const double turn_pull = rate * m_forward.speed;
  const double unexplained = centre_accel - turn_pull;
  const double right_before = m_right.speed;
  m_right.predict(unexplained, dt_s, accel_var, zero_var);
  const double push =
      std::fabs(turn_pull) +
      std::fmax(0.0, std::fabs(unexplained) - kPositionErrorBound * position_error_accel);
  m_right.measureSpeed(0.0, m_config.grip_sigma_ips + m_config.slide_sigma_s * push);

  const double forward_correction =
      m_forward_travel == ForwardTravel::FromDriveWheels
          ? (0.5 * (forward_before + m_forward.speed) - last.wheel_speed) * dt_s
          : 0.0;
  return {forward_correction, 0.5 * (right_before + m_right.speed) * dt_s};
}

}  // namespace control
}  // namespace mclib
