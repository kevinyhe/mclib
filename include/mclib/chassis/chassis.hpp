// mclib
#pragma once

#include "mclib/device/inertial.hpp"
#include "mclib/device/motor_group.hpp"
#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {

/**
 * @brief This Chassis's idea of the drive base.
 *
 * @warning These defaults are the **third** description of this robot's
 *          geometry, and they disagree with the other two. `config.cpp` says
 *          9.06 in of rolling circumference, which implies a 2.8839 in wheel -
 *          4.87% off the 2.75 here - and an 11.375 in track width, 1.1% off
 *          the 11.5 here. `mclib::config::robot_drive_geometry` is what
 *          `motion.cpp` and the odometry actually run on; this struct only
 *          feeds `Chassis::averageDistance()` and the `ChassisController`
 *          loops built on it. Picking a winner needs a tape measure, so both
 *          are left standing and the disagreement is pinned by
 *          `tests/geometry_test.cpp`.
 */
struct ChassisDimensions {
  QLength wheel_diameter = 2.75 * units::inch;
  QLength track_width = 11.5 * units::inch;
  /// @brief Wheel revolutions per motor revolution. Dimensionless.
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

  /// @brief Drive both sides as a fraction of full power, -1..1.
  void tank(double left_percent, double right_percent);
  /// @brief Drive both sides at a commanded voltage.
  void tankVoltage(QVoltage left, QVoltage right);
  /// @brief Forward and turn as fractions of full power, -1..1.
  void arcade(double forward_percent, double turn_percent);
  void stop(device::BrakeMode mode = device::BrakeMode::Brake);
  void setBrakeMode(device::BrakeMode mode);
  void tare();

  /// @brief Mean left motor position, degrees of motor shaft.
  double leftPositionDeg();
  /// @brief Mean right motor position, degrees of motor shaft.
  double rightPositionDeg();
  /// @brief Mean of the two sides, degrees of motor shaft.
  double averagePositionDeg();

  /// @brief Distance the left side has rolled, per getDimensions().
  QLength leftDistance();
  /// @brief Distance the right side has rolled, per getDimensions().
  QLength rightDistance();
  /// @brief Mean of the two sides. What ChassisController::driveDistance() tracks.
  QLength averageDistance();
  /// @brief Heading from this Chassis's IMU, or from the odometry without one.
  QAngle heading();

  /// @brief leftDistance() in inches.
  double leftDistanceIn();
  /// @brief rightDistance() in inches.
  double rightDistanceIn();
  /// @brief averageDistance() in inches.
  double averageDistanceIn();
  /// @brief heading() in degrees.
  double headingDeg();

  /**
   * @brief Teleport the odometry to a known pose.
   *
   * Goes straight to `mclib::control::resetOdometry()`. Chassis does not keep
   * a pose of its own -- it used to, and having two poses tracking the same
   * robot from the same sensors is how they drifted apart.
   *
   * @warning Also writes `pose.theta` back to this Chassis's IMU, so the
   * odometry's heading frame and the frame `motion.cpp` steers in stay
   * identical. That only works when this Chassis holds the same IMU the
   * odometry task reads -- the `inertial_sensor` from `config.cpp`. A Chassis
   * built with `imu == nullptr`, or with an `Inertial` on another port, moves
   * the odometry frame without moving the heading source, and the two drift
   * apart by exactly that offset with no diagnostic.
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

  /**
   * @brief Motor degrees to distance rolled.
   *
   * `degreesToInches()` is the primitive and `degreesToDistance()` wraps it,
   * not the other way round: computing in inches and typing the result keeps
   * the double path bit-identical to what it was before the dimensions grew
   * types, which matters because ChassisController's distance loop runs on it.
   */
  double degreesToInches(double deg) const;
  QLength degreesToDistance(double deg) const;

  device::MotorGroup m_left;
  device::MotorGroup m_right;
  std::shared_ptr<device::Inertial> m_imu;
  ChassisDimensions m_dimensions;
};

}  // namespace mclib
