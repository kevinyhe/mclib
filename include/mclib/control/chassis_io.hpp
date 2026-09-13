// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/types.hpp"
#include "mclib/units/geometry.hpp"
#include "mclib/units/units.hpp"

#include <vector>

/**
 * @file chassis_io.hpp
 * @brief The drive base as the motion routines see it: two sides and a heading.
 *
 * `motion.cpp` and the odometry task do not know which motors they are
 * driving. They talk to whatever `DriveHardware` has been bound with
 * `mclib::control::bindDrive()`. `mclib::Chassis` implements the interface,
 * and `ChassisController`'s constructor binds the Chassis it is given, so a
 * program that builds a ChassisController never calls bindDrive() itself.
 *
 * There is one bound drive at a time. The blocking motion routines are free
 * functions with no handle to look up, so a second drivetrain would need its
 * own copy of them; a competition robot has one.
 *
 * Everything used to reach `left_chassis`, `right_chassis` and
 * `inertial_sensor` - three globals constructed on one team's ports in a
 * `config.cpp` that shipped inside the library archive. A user who built a
 * `Chassis` on other ports and called `moveToPoint()` moved the wrong motors.
 * The globals are gone; there is no default drive.
 *
 * The free functions below are the surface `motion.cpp` was written against.
 * They forward to the bound drive and are kept so the tuned control loops did
 * not have to change shape. The sensor readers return `double` rather than a
 * `QAngle` because the odometry pipeline is degrees-and-radians throughout.
 */

namespace mclib {
namespace control {

/**
 * @brief What a differential drive has to provide for the motion routines and
 *        the odometry task to run on it.
 *
 * Voltages are volts, positions are degrees of motor shaft, heading is the
 * IMU's unwrapped rotation in degrees (compass frame, clockwise positive),
 * currents are milliamps and speeds are RPM - the V5 motor's native units,
 * which is what the stall detector in `wallReset()` compares against.
 */
class DriveHardware {
 public:
  virtual ~DriveHardware() = default;

  /// @brief Drive both sides. Positive is forward on both.
  virtual void setDriveVoltage(double left_volts, double right_volts) = 0;
  /// @brief Drive one side, leaving the other alone. `swing()` uses this.
  virtual void setSideVoltage(bool left_side, double volts) = 0;
  /// @brief Set the brake mode on both sides and stop them.
  virtual void brakeDrive(device::BrakeMode mode) = 0;
  /// @brief Set the brake mode on one side and stop it.
  virtual void brakeSide(bool left_side, device::BrakeMode mode) = 0;
  /// @brief Zero both drive encoders.
  virtual void tareDrive() = 0;

  /// @brief Mean position of the left drive motors, degrees of motor shaft.
  virtual double leftPositionDeg() = 0;
  /// @brief Mean position of the right drive motors, degrees of motor shaft.
  virtual double rightPositionDeg() = 0;

  /**
   * @brief Heading in degrees, compass frame, unwrapped.
   *
   * Not wrapped to +/-180: it keeps counting past a full turn, which is what
   * normalizeTarget() exists to compensate for. NaN when the sensor is not
   * reporting.
   */
  virtual double headingDeg() = 0;
  /// @brief Stamp a known heading onto the heading sensor. `wallReset()` and
  ///        `Chassis::setPose()` use this to keep the IMU and the odometry in
  ///        one frame.
  virtual void setHeadingDeg(double heading_deg) = 0;

  /// @brief Current draw of every drive motor, milliamps, both sides.
  virtual std::vector<double> driveCurrentsMa() = 0;
  /// @brief Speed of every drive motor, RPM, both sides.
  virtual std::vector<double> driveVelocitiesRpm() = 0;

  /// @brief Wheel, track width and gear ratio. Every encoder-to-inches
  ///        conversion in the motion routines goes through this.
  virtual const units::DriveGeometry& driveGeometry() const = 0;
};

/**
 * @brief Make @p drive the drivetrain the motion routines and the odometry
 *        task run on. Pass nullptr to unbind.
 *
 * The pointer is kept, not copied: @p drive must outlive every motion. A
 * `Chassis` at namespace scope or a `static` in `initialize()` both qualify.
 */
void bindDrive(DriveHardware* drive);

/// @brief The bound drive, or nullptr when none has been bound.
DriveHardware* boundDrive();

/// @brief True when a drive is bound.
bool isDriveBound();

/**
 * @brief The bound drive, with a one-time diagnostic when there is none.
 *
 * Every motion routine calls this first and returns immediately when it gets
 * nullptr. Driving a robot that has not said which motors it has is not a
 * thing to do quietly: a loop spinning on a NaN heading until its timeout
 * looks exactly like a broken IMU, and that costs a match of debugging.
 */
DriveHardware* requireDrive(const char* caller);

}  // namespace control
}  // namespace mclib

// ---------------------------------------------------------------------------
// The surface motion.cpp was written against. Each forwards to the bound
// drive. With no drive bound the writers do nothing, the encoder readers
// return 0 and the heading reader returns NaN.
// ---------------------------------------------------------------------------

/**
 * @brief Drive both sides at a commanded voltage.
 *
 * The actuator boundary. Positive drives the robot forward on both sides;
 * `motion.cpp` turns by passing `(v, -v)`.
 */
void driveChassis(QVoltage left_power, QVoltage right_power);

/// @brief Set the brake mode on both sides and stop them.
void stopChassis(mclib::device::BrakeMode mode);

/**
 * @brief Zero both drive encoders and re-seed the odometry from the live pose.
 *
 * Snapshots the pose before taring, because the odometry task samples the same
 * encoders every 10 ms and would otherwise read the tare as a real delta.
 */
void resetChassis();

/// @brief Mean position of the left drive motors, in **degrees** of motor shaft.
double getLeftRotationDegree();

/// @brief Mean position of the right drive motors, in **degrees** of motor shaft.
double getRightRotationDegree();

/**
 * @brief The heading in **degrees**, compass frame, unwrapped.
 *
 * See DriveHardware::headingDeg().
 */
double getInertialHeading();

/**
 * @brief Shift @p angle by whole turns until it is within 180 deg of the
 *        current heading.
 *
 * Both the argument and the result are **degrees** in the same unwrapped frame
 * as getInertialHeading(). Turning to the normalised target is what makes a
 * heading PID take the short way round.
 */
double normalizeTarget(double angle);
