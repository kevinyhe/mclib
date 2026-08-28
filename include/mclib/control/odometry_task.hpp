// mclib
#pragma once

#include "mclib/control/odometry.hpp"
#include "mclib/units/geometry.hpp"
#include "mclib/units/units.hpp"

/**
 * @file odometry_task.hpp
 * @brief The task that feeds `Odometry` from the real sensors.
 *
 * Everything PROS-shaped about odometry lives here: the `pros::Task`, the
 * delay, and the reads off the devices in `config.cpp`. The math is in
 * `odometry.hpp` and stays testable on a host.
 *
 * Start it once, early - `initialize()` is the natural place:
 *
 * @code
 * void initialize() {
 *   inertial_sensor.reset(true);
 *   mclib::control::startOdometry();
 * }
 * @endcode
 *
 * With no argument the layout comes from `mclib::config::robot_drive_geometry`
 * and `mclib::config::vertical_tracking_wheel` in `config.hpp` - the one place
 * this robot's drive base is described - and uses the drive encoders. Pass an
 * `OdometryConfig` to use the vertical tracking wheel instead.
 *
 * @warning The task cannot honour `use_horizontal_tracker`: `config.cpp` has
 * no horizontal sensor, so there is nothing for `sampleOdometrySensors()` to
 * read. `startOdometry()` clears the flag rather than integrating a constant
 * zero reading against a real offset, which would invent a sideways
 * displacement on every turn. `Odometry` itself supports two wheels - wire the
 * sensor up and call `odometryTick()` on your own schedule.
 */

namespace mclib {
namespace control {

/**
 * @brief Build an OdometryConfig from `mclib::config::robot_drive_geometry`
 *        and `mclib::config::vertical_tracking_wheel`.
 *
 * The name is historical: it used to read four `double` globals in
 * `config.cpp`, which are gone. It now reads the one typed geometry in
 * `mclib/robot_geometry.hpp`, at call time - so an override assigned in
 * `initialize()` is picked up.
 *
 * Drive encoders only - `use_vertical_tracker` is false - because that is what
 * the shipped robot has wired up. The vertical tracker fields are filled in
 * from config anyway, so turning it on is a one-line change.
 */
OdometryConfig odometryConfigFromGlobals();

/**
 * @brief Build an OdometryConfig from an explicit geometry pair.
 *
 * The body of `odometryConfigFromGlobals()`, split out so a host test can link
 * it: `odometry_task.cpp` pulls in PROS and `config.hpp` and cannot be built
 * on a host. The globals version passes the two geometries in
 * `mclib/robot_geometry.hpp`.
 *
 * Both lengths go through `encoderToDistance(360 deg)`, which is
 * `2 * pi * radius * gear_ratio` - the same formula `motion.cpp`'s
 * `encoderDegreesToInches()` uses. The bare `wheel.circumference()` that used
 * to be here dropped `gear_ratio`: on a 36:48 drive odometry integrated 4/3 of
 * the distance `driveTo()` measured, and nothing said which one was right.
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
 * @brief Read every sensor once, into a sample for `odometryTick()`.
 *
 * Exposed so a caller can integrate on its own schedule instead of starting
 * the task.
 */
OdometrySample sampleOdometrySensors();

/**
 * @brief Start the odometry task.
 *
 * Idempotent: a second call while the task is running does nothing and
 * returns false. The task runs until `stopOdometry()`.
 *
 * @param config Sensor layout. `use_horizontal_tracker` is cleared; see the
 *        file comment.
 * @param period How long to sleep between samples. 10 ms matches the control
 *        loops in `motion.cpp`.
 * @return True if this call started the task.
 */
bool startOdometry(const OdometryConfig& config, QTime period = 10 * millisecond);

/// @brief Start the odometry task with `odometryConfigFromGlobals()`.
bool startOdometry();

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
