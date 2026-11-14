// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/slide_estimator.hpp"

#include <cmath>

namespace mclib {
namespace control {

bool slideEstimatorConfigIsValid(const SlideEstimatorConfig& config) {
  const auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  return std::isfinite(config.imu_forward_in) && std::isfinite(config.imu_right_in) &&
         positive(config.accel_sigma_ips2) && positive(config.zero_sigma_ips2) &&
         positive(config.zero_walk_ips2_per_sqrt_in) && positive(config.grip_sigma_ips) &&
         positive(config.slide_sigma_s);
}

SlideEstimator::SlideEstimator(SlideEstimatorConfig config) : m_config(config) { reset(); }

void SlideEstimator::reset() {
  m_speed = 0.0;
  m_zero = 0.0;
  m_cov = Mat2::Zero();
  m_cov(1, 1) = m_config.zero_sigma_ips2 * m_config.zero_sigma_ips2;
  m_last_rate = 0.0;
  m_still_readings = 0;
  m_moved = false;
}

double SlideEstimator::update(double dt_s, double forward_in, double turn_rad,
                              double accel_right_ips2) {
  if (!(dt_s > 0.0)) return 0.0;
  const double rate = turn_rad / dt_s;
  const double forward_speed = forward_in / dt_s;
  const double rate_change = rate - m_last_rate;
  m_last_rate = rate;

  // Calibrating the IMU needs the robot still, so odometry starts still.
  // Until the robot first moves, the zero is the average reading. A reading
  // far off that average is the robot starting to move before its wheels
  // and heading show it.
  if (!m_moved) {
    const bool wheels_still = forward_in == 0.0 && turn_rad == 0.0;
    const bool reading_still =
        m_still_readings == 0 ||
        std::fabs(accel_right_ips2 - m_zero) < 4.0 * m_config.accel_sigma_ips2;
    if (wheels_still && reading_still) {
      ++m_still_readings;
      m_zero += (accel_right_ips2 - m_zero) / m_still_readings;
      m_cov(1, 1) = m_config.accel_sigma_ips2 * m_config.accel_sigma_ips2 / m_still_readings;
      return 0.0;
    }
    m_moved = true;
  }

  // An IMU off the tracking centre also feels the turn itself: the pull
  // toward the centre of the spin, and the push of the spin speeding up.
  const double centre_accel = accel_right_ips2 - m_zero +
                              rate * rate * m_config.imu_right_in -
                              m_config.imu_forward_in * rate_change / dt_s;
  const double turn_pull = rate * forward_speed;
  const double unexplained = centre_accel - turn_pull;

  const double speed_before = m_speed;
  m_speed += unexplained * dt_s;
  Mat2 F = Mat2::Identity();
  F(0, 1) = -dt_s;
  Mat2 Q = Mat2::Zero();
  Q(0, 0) = std::pow(m_config.accel_sigma_ips2 * dt_s, 2);
  Q(1, 1) = std::pow(m_config.zero_walk_ips2_per_sqrt_in, 2) * std::fabs(forward_in);
  m_cov = F * m_cov * F.transpose() + Q;

  const double slide_sigma = m_config.grip_sigma_ips +
                             m_config.slide_sigma_s * (std::fabs(turn_pull) + std::fabs(unexplained));
  const double innovation_var = m_cov(0, 0) + slide_sigma * slide_sigma;
  const double gain_speed = m_cov(0, 0) / innovation_var;
  const double gain_zero = m_cov(1, 0) / innovation_var;
  const double innovation = -m_speed;
  m_speed += gain_speed * innovation;
  m_zero += gain_zero * innovation;
  Mat2 I_KH = Mat2::Identity();
  I_KH(0, 0) -= gain_speed;
  I_KH(1, 0) -= gain_zero;
  Mat2 KRKt;
  KRKt << gain_speed * gain_speed, gain_speed * gain_zero, gain_speed * gain_zero,
      gain_zero * gain_zero;
  m_cov = I_KH * m_cov * I_KH.transpose() + KRKt * slide_sigma * slide_sigma;

  return 0.5 * (speed_before + m_speed) * dt_s;
}

}  // namespace control
}  // namespace mclib
