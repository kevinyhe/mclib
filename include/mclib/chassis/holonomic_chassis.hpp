// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/chassis/holonomic_math.hpp"
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
 * @brief A four-wheel holonomic drive base: X-drive or mecanum.
 *
 * The holonomic counterpart of `Chassis`. Four motor groups, one per
 * corner, plus an optional IMU. All the arithmetic is in
 * `holonomic_math.hpp`; this class only owns hardware and reads sensors.
 *
 * Signs are the library's: forward is +Y, strafe positive to the right,
 * turn positive clockwise. Wheel order everywhere is front_left,
 * front_right, back_left, back_right, looking down with forward at the top.
 * Reverse a motor by giving a negative port number, exactly as with
 * `Chassis`. Check the four signs with drive(1, 0, 0) before anything else:
 * a wheel wired backward makes forward look like a strafe-and-turn.
 *
 * @code
 * mclib::HolonomicChassis drive({1}, {-2}, {3}, {-4},
 *                               mclib::device::Gearset::Green,
 *                               mclib::holonomic::Kind::Mecanum,
 *                               imu);
 * drive.drive(0.5, 0.0, 0.0);          // half speed forward
 * drive.driveFieldCentric(1.0, 0.0, 0.0);  // toward field +X, whatever the heading
 * @endcode
 */
class HolonomicChassis {
 public:
  HolonomicChassis(std::initializer_list<std::int8_t> front_left_ports,
                   std::initializer_list<std::int8_t> front_right_ports,
                   std::initializer_list<std::int8_t> back_left_ports,
                   std::initializer_list<std::int8_t> back_right_ports,
                   device::Gearset gearset,
                   holonomic::Kind kind,
                   std::shared_ptr<device::Inertial> imu = nullptr);

  HolonomicChassis(std::vector<std::int8_t> front_left_ports,
                   std::vector<std::int8_t> front_right_ports,
                   std::vector<std::int8_t> back_left_ports,
                   std::vector<std::int8_t> back_right_ports,
                   device::Gearset gearset,
                   holonomic::Kind kind,
                   std::shared_ptr<device::Inertial> imu = nullptr);

  /**
   * @brief Robot-centric drive as fractions of full power, -1..1 each.
   *
   * @param forward Positive drives forward.
   * @param strafe  Positive drives to the robot's **right**.
   * @param turn    Positive turns **clockwise**.
   */
  void drive(double forward, double strafe, double turn);

  /// @brief Command each wheel at a voltage. Clamped to +/-12 V per motor.
  void driveVoltage(const holonomic::WheelSpeeds& volts);

  /**
   * @brief Field-centric drive: (x, y) is a direction on the field, not on the robot.
   *
   * Rotated into the robot frame with headingDeg() - the IMU when this
   * chassis has one, the odometry pose otherwise - then mixed like drive().
   *
   * @param field_x Command along field +X, -1..1.
   * @param field_y Command along field +Y, -1..1.
   * @param turn    Positive turns **clockwise**, -1..1.
   */
  void driveFieldCentric(double field_x, double field_y, double turn);

  void stop(device::BrakeMode mode = device::BrakeMode::Brake);
  void setBrakeMode(device::BrakeMode mode);
  void tare();

  /// @brief Heading in degrees: IMU rotation (unwrapped) with an IMU, odometry theta without.
  double headingDeg();
  /// @brief headingDeg() in radians.
  double headingRad();
  /// @brief headingDeg() as a QAngle.
  QAngle heading();

  /**
   * @brief Teleport the odometry to a known pose.
   *
   * Same as `Chassis::setPose()`: writes `pose.theta` to this chassis's IMU
   * and then calls `control::resetOdometry()`, so the heading source and
   * the odometry frame move together. The same warning applies - the IMU
   * here must be the one the odometry task reads, or the two frames drift
   * apart by exactly the offset set here.
   */
  void setPose(const Pose2D& pose);

  /// @brief The pose from the one odometry. Nothing here updates it; see Chassis::getPose().
  Pose2D getPose() const;

  holonomic::Kind kind() const;
  device::MotorGroup& frontLeft();
  device::MotorGroup& frontRight();
  device::MotorGroup& backLeft();
  device::MotorGroup& backRight();
  device::Inertial* imu();

 private:
  void writeFractions(const holonomic::WheelSpeeds& fractions);

  device::MotorGroup m_front_left;
  device::MotorGroup m_front_right;
  device::MotorGroup m_back_left;
  device::MotorGroup m_back_right;
  holonomic::Kind m_kind;
  std::shared_ptr<device::Inertial> m_imu;
};

}  // namespace mclib
