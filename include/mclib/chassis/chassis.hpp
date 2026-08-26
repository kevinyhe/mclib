// mclib
#pragma once

#include "mclib/device/inertial.hpp"
#include "mclib/device/motor_group.hpp"
#include "mclib/math.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {

struct ChassisDimensions {
  double wheel_diameter_in = 2.75;
  double track_width_in = 11.5;
  double drive_ratio = 1.0;
};

class Chassis {
public:
  Chassis(std::initializer_list<std::int8_t> left_ports,
          std::initializer_list<std::int8_t> right_ports,
          device::Gearset gearset = device::Gearset::Blue,
          ChassisDimensions dimensions = {},
          std::shared_ptr<device::Inertial> imu = nullptr);

  Chassis(std::vector<std::int8_t> left_ports,
          std::vector<std::int8_t> right_ports,
          device::Gearset gearset = device::Gearset::Blue,
          ChassisDimensions dimensions = {},
          std::shared_ptr<device::Inertial> imu = nullptr);

  void tank(double left_percent, double right_percent);
  void tankVoltage(double left_volts, double right_volts);
  void arcade(double forward_percent, double turn_percent);
  void stop(device::BrakeMode mode = device::BrakeMode::Brake);
  void setBrakeMode(device::BrakeMode mode);
  void tare();

  double leftPositionDeg();
  double rightPositionDeg();
  double averagePositionDeg();
  double leftDistanceIn();
  double rightDistanceIn();
  double averageDistanceIn();
  double headingDeg();

  void setPose(const Pose2D& pose);
  Pose2D getPose() const;
  Pose2D updateOdometry();

  ChassisDimensions getDimensions() const;
  device::MotorGroup& leftMotors();
  device::MotorGroup& rightMotors();
  device::Inertial* imu();

private:
  static int32_t voltsToMillivolts(double volts);
  static int32_t percentToMotorPower(double percent);

  double degreesToInches(double deg) const;

  device::MotorGroup m_left;
  device::MotorGroup m_right;
  std::shared_ptr<device::Inertial> m_imu;
  ChassisDimensions m_dimensions;
  Pose2D m_pose;
  double m_prev_left_in = 0.0;
  double m_prev_right_in = 0.0;
  double m_prev_heading_rad = 0.0;
};

}  // namespace mclib
