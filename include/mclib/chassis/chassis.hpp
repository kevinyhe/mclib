// mclib
#pragma once

#include "mclib/device/inertial.hpp"
#include "mclib/device/motor_group.hpp"
#include "mclib/math.hpp"
#include "mclib/units/geometry.hpp"
#include "mclib/units/units.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {

/**
 * @brief This Chassis's idea of the drive base. The same type the motion and
 *        odometry code runs on.
 *
 * This used to be a separate struct with its own defaults - a 2.75 in wheel
 * and an 11.5 in track width - which was the **third** description of the same
 * robot and disagreed with the other two by 4.87% on distance and 1.1% on turn
 * arc. It is now an alias for `units::DriveGeometry`, so `Chassis` and
 * `motion.cpp` cannot describe the drive base differently: there is one type
 * and one value, `mclib::config::robot_drive_geometry`.
 *
 * There are deliberately **no defaults**. `units::Wheel` has no default
 * constructor, so `ChassisDimensions{}` does not compile and neither does a
 * `Chassis` built without geometry. Describe your robot once, at setup:
 *
 * @code
 * #include "mclib/config.hpp"
 * mclib::Chassis drive({-11, 13, 14}, {-16, 17, -18},
 *                      mclib::device::Gearset::Blue,
 *                      mclib::config::robot_drive_geometry);
 * @endcode
 *
 * or state it inline, saying which measurement you took:
 *
 * @code
 * mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(2.75_in), 11.5_in, 1.0}
 * @endcode
 *
 * A wrong wheel size is a silent 5% scaling error on every autonomous. A
 * compile error pointing at the one line that describes the robot is better.
 */
using ChassisDimensions = units::DriveGeometry;

class Chassis {
public:
  Chassis(std::initializer_list<std::int8_t> left_ports,
          std::initializer_list<std::int8_t> right_ports,
          device::Gearset gearset,
          const ChassisDimensions& dimensions,
          std::shared_ptr<device::Inertial> imu = nullptr);

  Chassis(std::vector<std::int8_t> left_ports,
          std::vector<std::int8_t> right_ports,
          device::Gearset gearset,
          const ChassisDimensions& dimensions,
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
   * That is also why this does not call `DriveGeometry::encoderToDistance()`,
   * which routes the same formula through metres and can land 2 ulp away.
   */
  double degreesToInches(double deg) const;
  QLength degreesToDistance(double deg) const;

  device::MotorGroup m_left;
  device::MotorGroup m_right;
  std::shared_ptr<device::Inertial> m_imu;
  ChassisDimensions m_dimensions;
};

}  // namespace mclib
