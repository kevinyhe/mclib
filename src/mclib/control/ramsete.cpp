// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/ramsete.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace control {

namespace {

double sinc(double x) {
  return std::fabs(x) < 1e-9 ? 1.0 - x * x / 6.0 : std::sin(x) / x;
}

}  // namespace

RamseteOutput Ramsete::calculate(const Pose2D& pose,
                                 const path::TrajectoryState& target) const {
  // Field error in metres, so the SI gains apply.
  const double dx = (target.x - pose.x * units::inch).raw();
  const double dy = (target.y - pose.y * units::inch).raw();
  // Compass frame: forward is (sin, cos), right is (cos, -sin).
  const double s = std::sin(pose.theta);
  const double c = std::cos(pose.theta);
  const double e_x = dx * s + dy * c;
  const double e_y = dx * c - dy * s;
  const double e_theta = wrapAngle(target.heading.rad() - pose.theta);

  const double v_d = target.velocity.raw();
  const double w_d = target.angularVelocity().raw();
  const double k = 2.0 * m_gains.zeta * std::sqrt(w_d * w_d + m_gains.b * v_d * v_d);

  RamseteOutput out;
  out.velocity = units::QVelocity::fromBase(v_d * std::cos(e_theta) + k * e_x);
  out.angular_velocity = units::QAngularVelocity::fromBase(
      w_d + k * e_theta + m_gains.b * v_d * sinc(e_theta) * e_y);
  return out;
}

path::WheelSpeeds tankWheelSpeeds(const RamseteOutput& command,
                                  units::QLength track_width) {
  const double spread = command.angular_velocity.raw() * track_width.raw() / 2.0;
  return path::WheelSpeeds{
      command.velocity + units::QVelocity::fromBase(spread),
      command.velocity - units::QVelocity::fromBase(spread)};
}

DriveVoltages ramseteVoltages(const RamseteOutput& command,
                              units::QAcceleration acceleration,
                              units::QLength track_width,
                              const SimpleMotorFeedforward& feedforward,
                              units::QVoltage max_voltage) {
  const path::WheelSpeeds wheels = tankWheelSpeeds(command, track_width);
  double left = feedforward.calculate(wheels.left, acceleration).raw();
  double right = feedforward.calculate(wheels.right, acceleration).raw();
  const double cap = std::fabs(max_voltage.raw());
  const double worst = std::max(std::fabs(left), std::fabs(right));
  if (worst > cap && worst > 0.0) {
    left *= cap / worst;
    right *= cap / worst;
  }
  return DriveVoltages{units::QVoltage::fromBase(left),
                       units::QVoltage::fromBase(right)};
}

}  // namespace control
}  // namespace mclib
