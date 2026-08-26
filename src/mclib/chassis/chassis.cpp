// mclib
#include "mclib/chassis/chassis.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {

Chassis::Chassis(std::initializer_list<std::int8_t> left_ports,
                 std::initializer_list<std::int8_t> right_ports,
                 device::Gearset gearset,
                 ChassisDimensions dimensions,
                 std::shared_ptr<device::Inertial> imu)
    : Chassis(std::vector<std::int8_t>(left_ports),
              std::vector<std::int8_t>(right_ports),
              gearset,
              dimensions,
              std::move(imu)) {}

Chassis::Chassis(std::vector<std::int8_t> left_ports,
                 std::vector<std::int8_t> right_ports,
                 device::Gearset gearset,
                 ChassisDimensions dimensions,
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

void Chassis::tankVoltage(double left_volts, double right_volts) {
  m_left.setVoltage(left_volts);
  m_right.setVoltage(right_volts);
}

void Chassis::arcade(double forward_percent, double turn_percent) {
  tank(forward_percent - turn_percent, forward_percent + turn_percent);
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
  m_left.tarePosition();
  m_right.tarePosition();
  m_prev_left_in = 0.0;
  m_prev_right_in = 0.0;
  m_prev_heading_rad = headingDeg() * kPi / 180.0;
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
  return m_pose.theta * 180.0 / kPi;
}

void Chassis::setPose(const Pose2D& pose) {
  m_pose = Pose2D{pose.x, pose.y, wrapAngle(pose.theta)};
  m_prev_left_in = leftDistanceIn();
  m_prev_right_in = rightDistanceIn();
  m_prev_heading_rad = m_pose.theta;
}

Pose2D Chassis::getPose() const {
  return m_pose;
}

Pose2D Chassis::updateOdometry() {
  const double left_in = leftDistanceIn();
  const double right_in = rightDistanceIn();
  const double delta_left = left_in - m_prev_left_in;
  const double delta_right = right_in - m_prev_right_in;
  const double heading_rad = headingDeg() * kPi / 180.0;
  const double delta_distance = (delta_left + delta_right) * 0.5;
  const double avg_heading = m_prev_heading_rad + wrapAngle(heading_rad - m_prev_heading_rad) * 0.5;

  m_pose.x += delta_distance * std::sin(avg_heading);
  m_pose.y += delta_distance * std::cos(avg_heading);
  m_pose.theta = wrapAngle(heading_rad);

  m_prev_left_in = left_in;
  m_prev_right_in = right_in;
  m_prev_heading_rad = heading_rad;
  return m_pose;
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

int32_t Chassis::voltsToMillivolts(double volts) {
  volts = std::clamp(volts, -12.0, 12.0);
  return static_cast<int32_t>(volts * 1000.0);
}

int32_t Chassis::percentToMotorPower(double percent) {
  percent = std::clamp(percent, -1.0, 1.0);
  return static_cast<int32_t>(percent * 127.0);
}

double Chassis::degreesToInches(double deg) const {
  return (deg / 360.0) * m_dimensions.wheel_diameter_in * kPi *
         m_dimensions.drive_ratio;
}

}  // namespace mclib
