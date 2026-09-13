// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/chassis/chassis.hpp"

#include "mclib/chassis/chassis_math.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace mclib {

Chassis::Chassis(std::initializer_list<std::int8_t> left_ports,
                 std::initializer_list<std::int8_t> right_ports,
                 device::Gearset gearset,
                 const ChassisDimensions& dimensions,
                 std::shared_ptr<device::Inertial> imu)
    : Chassis(std::vector<std::int8_t>(left_ports),
              std::vector<std::int8_t>(right_ports),
              gearset,
              dimensions,
              std::move(imu)) {}

Chassis::Chassis(std::vector<std::int8_t> left_ports,
                 std::vector<std::int8_t> right_ports,
                 device::Gearset gearset,
                 const ChassisDimensions& dimensions,
                 std::shared_ptr<device::Inertial> imu)
    : m_left(std::move(left_ports), gearset),
      m_right(std::move(right_ports), gearset),
      m_imu(std::move(imu)),
      m_dimensions(dimensions) {
  tare();
}

void Chassis::tank(double left_percent, double right_percent) {
  m_left.setPercent(left_percent);
  m_right.setPercent(right_percent);
}

void Chassis::tankVoltage(QVoltage left, QVoltage right) {
  m_left.setVoltage(left.volts());
  m_right.setVoltage(right.volts());
}

void Chassis::arcade(double forward_percent, double turn_percent) {
  // Left leads on a positive turn. The mix used to be the other way round,
  // which made this the one primitive in the library that turned the opposite
  // way to tank(+x, -x) and to a positive turnToHeading() delta -- the right
  // stick steered the robot left.
  const auto pair = chassis_math::arcadeMix(forward_percent, turn_percent);
  tank(pair.left, pair.right);
}

void Chassis::stop(device::BrakeMode mode) {
  setBrakeMode(mode);
  m_left.brake();
  m_right.brake();
}

void Chassis::setBrakeMode(device::BrakeMode mode) {
  m_left.setBrakeMode(mode);
  m_right.setBrakeMode(mode);
}

void Chassis::tare() {
  // Snapshot first: the odometry task samples these same encoders on another
  // task, so a tick landing between the tare and the reset would read the tare
  // as a delta the size of everything driven so far. Reading the pose after
  // the tare would then adopt that corrupted value permanently.
  const Pose2D pose = control::robotState().pose();
  m_left.tarePosition();
  m_right.tarePosition();
  if (m_imu == nullptr) m_encoder_heading_offset_deg = pose.theta * 180.0 / kPi;
  control::resetOdometry(pose);
}

double Chassis::leftPositionDeg() {
  return m_left.getAveragePositionDeg();
}

double Chassis::rightPositionDeg() {
  return m_right.getAveragePositionDeg();
}

double Chassis::averagePositionDeg() {
  return (leftPositionDeg() + rightPositionDeg()) * 0.5;
}

QLength Chassis::leftDistance() {
  return degreesToDistance(leftPositionDeg());
}

QLength Chassis::rightDistance() {
  return degreesToDistance(rightPositionDeg());
}

QLength Chassis::averageDistance() {
  return (leftDistance() + rightDistance()) * 0.5;
}

QAngle Chassis::heading() {
  return headingDeg() * units::degree;
}

double Chassis::leftDistanceIn() {
  return degreesToInches(leftPositionDeg());
}

double Chassis::rightDistanceIn() {
  return degreesToInches(rightPositionDeg());
}

double Chassis::averageDistanceIn() {
  return (leftDistanceIn() + rightDistanceIn()) * 0.5;
}

double Chassis::headingDeg() {
  if (m_imu != nullptr) {
    return m_imu->getRotationDeg();
  }
  return encoderHeadingDeg() + m_encoder_heading_offset_deg;
}

double Chassis::encoderHeadingDeg() {
  return chassis_math::encoderHeadingDeg(
      leftDistanceIn(), rightDistanceIn(), m_dimensions.track_width.in());
}

void Chassis::setPose(const Pose2D& pose) {
  // The odometry tracks heading as IMU deltas from whatever theta it was last
  // reset to, while the motion routines steer on raw IMU degrees. Move the IMU
  // too, or the two frames drift apart by exactly the offset introduced here
  // and every subsequent moveToPoint aims wrong.
  setHeadingDeg(wrapAngle(pose.theta) * 180.0 / kPi);
  control::resetOdometry(pose);
}

Pose2D Chassis::getPose() const {
  return control::robotState().pose();
}

ChassisDimensions Chassis::getDimensions() const {
  return m_dimensions;
}

device::MotorGroup& Chassis::leftMotors() {
  return m_left;
}

device::MotorGroup& Chassis::rightMotors() {
  return m_right;
}

device::Inertial* Chassis::imu() {
  return m_imu.get();
}

void Chassis::setDriveVoltage(double left_volts, double right_volts) {
  m_left.setVoltage(left_volts);
  m_right.setVoltage(right_volts);
}

void Chassis::setSideVoltage(bool left_side, double volts) {
  (left_side ? m_left : m_right).setVoltage(volts);
}

void Chassis::brakeDrive(device::BrakeMode mode) {
  stop(mode);
}

void Chassis::brakeSide(bool left_side, device::BrakeMode mode) {
  device::MotorGroup& side = left_side ? m_left : m_right;
  side.setBrakeMode(mode);
  side.brake();
}

void Chassis::tareDrive() {
  // Not tare(): that one also re-seeds the odometry, and resetChassis() -
  // the caller on this path - does that itself, in the right order.
  const double heading = headingDeg();
  m_left.tarePosition();
  m_right.tarePosition();
  if (m_imu == nullptr && std::isfinite(heading)) m_encoder_heading_offset_deg = heading;
}

void Chassis::setHeadingDeg(double heading_deg) {
  if (m_imu != nullptr) {
    m_imu->setRotationDeg(heading_deg);
  } else {
    const double raw = encoderHeadingDeg();
    if (std::isfinite(raw) && std::isfinite(heading_deg))
      m_encoder_heading_offset_deg = heading_deg - raw;
  }
}

std::vector<double> Chassis::driveCurrentsMa() {
  std::vector<double> all = m_left.getCurrentDraws();
  const std::vector<double> right = m_right.getCurrentDraws();
  all.insert(all.end(), right.begin(), right.end());
  return all;
}

std::vector<double> Chassis::driveVelocitiesRpm() {
  std::vector<double> all = m_left.getActualVelocities();
  const std::vector<double> right = m_right.getActualVelocities();
  all.insert(all.end(), right.begin(), right.end());
  return all;
}

const units::DriveGeometry& Chassis::driveGeometry() const {
  return m_dimensions;
}

int32_t Chassis::voltsToMillivolts(double volts) {
  volts = std::clamp(volts, -12.0, 12.0);
  return static_cast<int32_t>(volts * 1000.0);
}

int32_t Chassis::percentToMotorPower(double percent) {
  percent = std::clamp(percent, -1.0, 1.0);
  return static_cast<int32_t>(percent * 127.0);
}

double Chassis::degreesToInches(double deg) const {
  return (deg / 360.0) * m_dimensions.wheel.diameter().in() * kPi *
         m_dimensions.gear_ratio;
}

QLength Chassis::degreesToDistance(double deg) const {
  return degreesToInches(deg) * units::inch;
}

}  // namespace mclib
