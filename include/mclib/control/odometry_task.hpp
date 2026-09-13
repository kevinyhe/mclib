// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/odometry.hpp"
#include "mclib/units/geometry.hpp"
#include "mclib/units/units.hpp"

#include <cmath>
#include <functional>
#include <optional>

/**
 * @file odometry_task.hpp
 * @brief The task that feeds `Odometry` from the real sensors.
 *
 * Everything PROS-shaped about odometry lives here: the `pros::Task`, the
 * delay, and the reads off the sensors. The math is in `odometry.hpp` and
 * stays testable on a host.
 *
 * The sensors are handed in as readers, not looked up from globals, so any
 * robot can describe its own layout:
 *
 * @code
 * mclib::Chassis chassis({-11, 13, 14}, {-16, 17, -18}, Gearset::Blue, geometry, imu);
 * mclib::device::Rotation vertical_tracker(-6);
 *
 * void initialize() {
 *   imu->reset(true);
 *   auto setup = mclib::control::odometrySetupFrom(chassis);
 *   setup.vertical = {mclib::control::encoderReader(vertical_tracker),
 *                     mclib::units::TrackingWheel{Wheel::fromDiameter(2_in), 0_in, 1.0}};
 *   mclib::control::startOdometry(setup);
 * }
 * @endcode
 *
 * With no tracking wheels the drive encoders measure forward travel. Add a
 * vertical wheel to be immune to drive slip; add a horizontal wheel to catch
 * sideways push. Both are honoured now - the old task cleared
 * `use_horizontal_tracker` because there was no global to read.
 */

namespace mclib {
namespace control {

/**
 * @brief One tracking wheel: how to read it and where it sits.
 *
 * `position_deg` returns the wheel's accumulated rotation in degrees, or NaN
 * when the sensor is not reporting - `Odometry` skips a non-finite sample
 * rather than integrating it. `encoderReader()` builds such a reader from any
 * sensor with a `position()` returning `std::optional<QAngle>`.
 */
struct TrackingWheelSensor {
  std::function<double()> position_deg;
  units::TrackingWheel wheel;
};

/**
 * @brief Every sensor the odometry task reads, and the geometry to read it
 *        with.
 *
 * `heading_deg` is required. The drive encoders are optional as a pair; leave
 * both empty when the robot measures forward travel with a vertical tracking
 * wheel instead. At least one of the two must be present.
 */
struct OdometrySetup {
  /// @brief Unwrapped heading in degrees, compass frame. NaN on a fault.
  std::function<double()> heading_deg;
  /// @brief Left drive encoder, degrees of motor shaft.
  std::function<double()> left_deg;
  /// @brief Right drive encoder, degrees of motor shaft.
  std::function<double()> right_deg;
  /// @brief Wheel, track width and gear ratio for the drive-encoder path.
  units::DriveGeometry drive{units::Wheel::fromDiameter(1.0 * units::inch),
                             1.0 * units::inch, 1.0};
  /// @brief Forward-rolling tracking wheel, if fitted.
  std::optional<TrackingWheelSensor> vertical;
  /// @brief Sideways-rolling tracking wheel, if fitted.
  std::optional<TrackingWheelSensor> horizontal;
};

/**
 * @brief A reader for any sensor whose `position()` returns
 *        `std::optional<units::QAngle>` - `device::Rotation`,
 *        `device::AdiEncoder`.
 *
 * The sensor is captured by reference and must outlive the odometry task.
 */
template <class Sensor>
std::function<double()> encoderReader(const Sensor& sensor) {
  return [&sensor]() -> double {
    const std::optional<units::QAngle> position = sensor.position();
    return position.has_value() ? position->deg() : NAN;
  };
}

class DriveHardware;

/**
 * @brief Heading, drive encoders and geometry from a bound-style drive.
 *
 * The drive is captured by reference and must outlive the odometry task.
 * Add tracking wheels to the result before passing it to `startOdometry()`.
 */
OdometrySetup odometrySetupFrom(DriveHardware& drive);

/**
 * @brief Build an OdometryConfig from an explicit geometry pair.
 *
 * Both lengths go through `encoderToDistance(360 deg)`, which is
 * `2 * pi * radius * gear_ratio` - the same formula `motion.cpp`'s
 * `encoderDegreesToInches()` uses. A bare `wheel.circumference()` would drop
 * `gear_ratio`: on a 36:48 drive odometry would integrate 4/3 of the distance
 * `driveTo()` measured.
 *
 * `use_vertical_tracker` and `use_horizontal_tracker` come out false; the
 * `OdometrySetup` overload sets them from which sensors are present.
 *
 * @param drive    Drive base geometry, for the drive-encoder path.
 * @param vertical Vertical tracking wheel, for its circumference and offset.
 */
inline OdometryConfig odometryConfigFrom(const units::DriveGeometry& drive,
                                         const units::TrackingWheel& vertical) {
  OdometryConfig config;
  config.drive_inches_per_revolution =
      drive.encoderToDistance(360.0 * units::degree);
  config.use_vertical_tracker = false;
  config.vertical_circumference =
      vertical.encoderToDistance(360.0 * units::degree);
  config.vertical_offset_right = vertical.offset;
  config.use_horizontal_tracker = false;
  return config;
}

/**
 * @brief The OdometryConfig an OdometrySetup implies.
 *
 * Each tracker flag is set exactly when the setup carries that sensor. A
 * setup with a vertical wheel and no drive encoders still works: the
 * drive-encoder fields are ignored when `use_vertical_tracker` is true.
 */
inline OdometryConfig odometryConfigFrom(const OdometrySetup& setup) {
  OdometryConfig config;
  config.drive_inches_per_revolution =
      setup.drive.encoderToDistance(360.0 * units::degree);
  config.use_vertical_tracker = setup.vertical.has_value();
  if (setup.vertical.has_value()) {
    config.vertical_circumference =
        setup.vertical->wheel.encoderToDistance(360.0 * units::degree);
    config.vertical_offset_right = setup.vertical->wheel.offset;
  }
  config.use_horizontal_tracker = setup.horizontal.has_value();
  if (setup.horizontal.has_value()) {
    config.horizontal_circumference =
        setup.horizontal->wheel.encoderToDistance(360.0 * units::degree);
    config.horizontal_offset_forward = setup.horizontal->wheel.offset;
  }
  return config;
}

/**
 * @brief True when @p setup can drive the odometry: a heading reader plus
 *        either both drive encoders or a vertical tracking wheel.
 */
bool odometrySetupIsUsable(const OdometrySetup& setup);

/**
 * @brief Read every sensor in @p setup once, into a sample for
 *        `odometryTick()`.
 *
 * Exposed so a caller can integrate on its own schedule instead of starting
 * the task. Readers that are not set contribute 0.
 */
OdometrySample sampleOdometrySensors(const OdometrySetup& setup);

/**
 * @brief Start the odometry task.
 *
 * Idempotent: a second call while the task is running does nothing and
 * returns false. So does a setup that `odometrySetupIsUsable()` rejects. The
 * task runs until `stopOdometry()`.
 *
 * @param setup  Sensors and geometry. Copied; the readers' referents must
 *               outlive the task.
 * @param period How long to sleep between samples. 10 ms matches the control
 *               loops in `motion.cpp`.
 * @return True if this call started the task.
 */
bool startOdometry(const OdometrySetup& setup, QTime period = 10 * millisecond);

/**
 * @brief Start the odometry task on the bound drive's heading and encoders,
 *        with no tracking wheels.
 *
 * Returns false when no drive is bound - build the ChassisController first.
 */
bool startOdometry(QTime period = 10 * millisecond);

/**
 * @brief Ask the odometry task to stop and wait for it to exit.
 *
 * Cooperative - the task checks a flag at its delay boundary - so it never
 * dies holding the odometry lock.
 */
void stopOdometry();

/// @brief True while the odometry task is running.
bool isOdometryRunning();

}  // namespace control
}  // namespace mclib
