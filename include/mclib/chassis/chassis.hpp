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

  /**
   * @brief Teleport the odometry to a known pose.
   *
   * Goes straight to `mclib::control::resetOdometry()`. Chassis does not keep
   * a pose of its own -- it used to, and having two poses tracking the same
   * robot from the same sensors is how they drifted apart.
   */
  void setPose(const Pose2D& pose);

  /**
   * @brief The pose, from the one odometry, read consistently.
   *
   * @warning Nothing here updates it. The pose comes from the odometry task,
   * which someone has to start once with
   * `mclib::control::startOdometry()`; without that this sits at the origin
   * and never moves. `ChassisController::periodic()` used to run a second,
   * flat-approximation odometry of its own, and that is what it no longer
   * does.
   */
  Pose2D getPose() const;

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
};

}  // namespace mclib
