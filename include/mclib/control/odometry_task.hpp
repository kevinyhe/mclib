// mclib
#pragma once

#include "mclib/control/odometry.hpp"
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
 * With no argument the layout comes from the globals in `config.cpp`
 * (`wheel_distance_in`, `vertical_tracker_diameter`,
 * `vertical_tracker_dist_from_center`) and uses the drive encoders. Pass an
 * `OdometryConfig` to use a tracking wheel instead.
 */

namespace mclib {
namespace control {

/**
 * @brief Build an OdometryConfig from the globals in `config.cpp`.
 *
 * Drive encoders only - `use_vertical_tracker` is false - because that is what
 * the shipped robot has wired up. The vertical tracker fields are filled in
 * from config anyway, so turning it on is a one-line change.
 */
OdometryConfig odometryConfigFromGlobals();

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
 * @param config Sensor layout.
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
