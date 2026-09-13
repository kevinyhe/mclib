// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/chassis/holonomic_chassis.hpp"

#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {

HolonomicChassis::HolonomicChassis(std::initializer_list<std::int8_t> front_left_ports,
                                   std::initializer_list<std::int8_t> front_right_ports,
                                   std::initializer_list<std::int8_t> back_left_ports,
                                   std::initializer_list<std::int8_t> back_right_ports,
                                   device::Gearset gearset,
                                   holonomic::Kind kind,
                                   std::shared_ptr<device::Inertial> imu)
    : HolonomicChassis(std::vector<std::int8_t>(front_left_ports),
                       std::vector<std::int8_t>(front_right_ports),
                       std::vector<std::int8_t>(back_left_ports),
                       std::vector<std::int8_t>(back_right_ports),
                       gearset,
                       kind,
                       std::move(imu)) {}

HolonomicChassis::HolonomicChassis(std::vector<std::int8_t> front_left_ports,
                                   std::vector<std::int8_t> front_right_ports,
                                   std::vector<std::int8_t> back_left_ports,
                                   std::vector<std::int8_t> back_right_ports,
                                   device::Gearset gearset,
                                   holonomic::Kind kind,
                                   std::shared_ptr<device::Inertial> imu)
    : m_front_left(std::move(front_left_ports), gearset),
      m_front_right(std::move(front_right_ports), gearset),
      m_back_left(std::move(back_left_ports), gearset),
      m_back_right(std::move(back_right_ports), gearset),
      m_kind(kind),
      m_imu(std::move(imu)) {
  tare();
}

void HolonomicChassis::drive(double forward, double strafe, double turn) {
  writeFractions(holonomic::mix(forward, strafe, turn, m_kind));
}

void HolonomicChassis::driveVoltage(const holonomic::WheelSpeeds& volts) {
  // MotorGroup::setVoltage() clamps to the 12 V rail itself; a NaN would
  // still get through to the cast, so squash it here.
  const auto safe = [](double v) { return std::isfinite(v) ? v : 0.0; };
  m_front_left.setVoltage(safe(volts.front_left));
  m_front_right.setVoltage(safe(volts.front_right));
  m_back_left.setVoltage(safe(volts.back_left));
  m_back_right.setVoltage(safe(volts.back_right));
}

void HolonomicChassis::driveFieldCentric(double field_x, double field_y, double turn) {
  const holonomic::RobotFrameInput robot =
      holonomic::fieldToRobot(field_x, field_y, headingRad());
  drive(robot.forward, robot.strafe, turn);
}

void HolonomicChassis::stop(device::BrakeMode mode) {
  setBrakeMode(mode);
  m_front_left.brake();
  m_front_right.brake();
  m_back_left.brake();
  m_back_right.brake();
}

void HolonomicChassis::setBrakeMode(device::BrakeMode mode) {
  m_front_left.setBrakeMode(mode);
  m_front_right.setBrakeMode(mode);
  m_back_left.setBrakeMode(mode);
  m_back_right.setBrakeMode(mode);
}

void HolonomicChassis::tare() {
  // Same order as Chassis::tare(): snapshot the pose before zeroing the
  // encoders, so an odometry tick that lands in between cannot read the tare
  // as a move the size of everything driven so far.
  const Pose2D pose = control::robotState().pose();
  m_front_left.tarePosition();
  m_front_right.tarePosition();
  m_back_left.tarePosition();
  m_back_right.tarePosition();
  control::resetOdometry(pose);
}

double HolonomicChassis::headingDeg() {
  if (m_imu != nullptr) {
    return m_imu->getRotationDeg();
  }
  return control::robotState().pose().theta * 180.0 / kPi;
}

double HolonomicChassis::headingRad() {
  return headingDeg() * kPi / 180.0;
}

QAngle HolonomicChassis::heading() {
  return headingDeg() * units::degree;
}

void HolonomicChassis::setPose(const Pose2D& pose) {
  // Mirror of Chassis::setPose(): the odometry heading is IMU deltas from the
  // last reset, so the IMU has to move with the odometry frame or the two
  // drift apart by exactly this offset.
  if (m_imu != nullptr) {
    m_imu->setRotationDeg(wrapAngle(pose.theta) * 180.0 / kPi);
  }
  control::resetOdometry(pose);
}

Pose2D HolonomicChassis::getPose() const {
  return control::robotState().pose();
}

holonomic::Kind HolonomicChassis::kind() const {
  return m_kind;
}

device::MotorGroup& HolonomicChassis::frontLeft() {
  return m_front_left;
}

device::MotorGroup& HolonomicChassis::frontRight() {
  return m_front_right;
}

device::MotorGroup& HolonomicChassis::backLeft() {
  return m_back_left;
}

device::MotorGroup& HolonomicChassis::backRight() {
  return m_back_right;
}

device::Inertial* HolonomicChassis::imu() {
  return m_imu.get();
}

void HolonomicChassis::writeFractions(const holonomic::WheelSpeeds& fractions) {
  m_front_left.setPercent(fractions.front_left);
  m_front_right.setPercent(fractions.front_right);
  m_back_left.setPercent(fractions.back_left);
  m_back_right.setPercent(fractions.back_right);
}

}  // namespace mclib
