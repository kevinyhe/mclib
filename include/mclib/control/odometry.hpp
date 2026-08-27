// mclib
#pragma once

#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include <cstdint>

/**
 * @file odometry.hpp
 * @brief Arc-based odometry: one implementation, three sensor layouts.
 *
 * `Odometry` is pure arithmetic. It touches no device, no PROS header and no
 * clock; you hand it a sample of raw sensor readings and it hands back a pose.
 * That is deliberate - it is the only way to test odometry without a robot,
 * and `tests/odometry_test.cpp` drives it with synthetic sequences.
 *
 * The task that reads the real sensors and calls this lives in
 * `control/odometry_task.hpp`.
 *
 * ## Supported layouts
 *
 * - **0 tracking wheels.** IMU for heading, the drive encoders for forward
 *   travel. Fine on a traction drive, wrong the moment a wheel slips.
 * - **1 tracking wheel.** IMU plus one unpowered vertical (forward-rolling)
 *   wheel. Immune to drive slip along the forward axis.
 * - **2 tracking wheels.** Adds a horizontal (sideways-rolling) wheel, which
 *   is what catches lateral push and the sideways component of an arc.
 *
 * ## The math
 *
 * Over one tick the robot is assumed to move at constant body velocity and
 * constant turn rate, i.e. along a circular arc. Write the body-frame
 * displacement of the tracking centre as `(dx_body, dy_body)` - right and
 * forward - and the heading change as `dtheta`, clockwise-positive.
 *
 * A forward-rolling wheel sitting `px` inches to the robot's **right** of the
 * tracking centre measures `dy_body - dtheta * px`, because rotation carries
 * it backwards relative to the centre. A sideways-rolling wheel `py` inches
 * **ahead** of the centre measures `dx_body + dtheta * py`. Invert those:
 *
 * @code
 * dy_body = forward_reading  + dtheta * forward_wheel_right_offset
 * dx_body = sideways_reading - dtheta * sideways_wheel_forward_offset
 * @endcode
 *
 * Integrating the arc gives the displacement in the frame the tick started in:
 *
 * @code
 * k     = 2 * sin(dtheta / 2) / dtheta          // -> 1 as dtheta -> 0
 * psi   = theta_prev + dtheta / 2
 * dx_field = k * ( dx_body * cos(psi) + dy_body * sin(psi))
 * dy_field = k * (-dx_body * sin(psi) + dy_body * cos(psi))
 * @endcode
 *
 * The `sin` and `cos` are that way round because this is the compass frame:
 * theta = 0 is +Y and grows clockwise. See the frame section of math.hpp.
 *
 * ## Bugs this replaces
 *
 * The two dead functions this file used to declare had four:
 *
 * 1. Both drive wheels used `+ track_width / 2`. The two sides sit on opposite
 *    sides of the centre, so the offsets have opposite signs. With `+/+` a
 *    pure in-place spin produced a phantom translation of half a track width
 *    per revolution. Folding the two encoders into one centre distance makes
 *    the sign impossible to get wrong.
 * 2. `prev_heading_rad` started at 0 without reading the IMU, so a non-zero
 *    starting heading became a full-size heading delta on the first tick.
 *    Here the first sample only seeds the previous readings; see `update()`.
 * 3. `Inertial::getRotationDeg()` returns NAN on a sensor fault and nothing
 *    checked. One dropout poisoned the pose permanently. A non-finite sample
 *    is now skipped whole, and the travel it covered is caught up on the next
 *    good sample.
 * 4. A comment claimed a centidegree conversion that `Rotation::getPositionDeg()`
 *    had already done.
 */

namespace mclib {
namespace control {

/**
 * @brief Which sensors the odometry has and where they sit.
 *
 * Offsets are signed and measured from the tracking centre - the point whose
 * pose the odometry reports, normally the centre of rotation.
 */
struct OdometryConfig {
  /**
   * @brief Inches the robot travels per revolution of a drive motor's output.
   *
   * Wheel circumference divided by the external gear ratio. Only used when
   * `use_vertical_tracker` is false.
   */
  QLength drive_inches_per_revolution = 9.06 * units::inch;

  /// @brief Use the vertical tracking wheel instead of the drive encoders.
  bool use_vertical_tracker = false;

  /// @brief Circumference of the vertical tracking wheel.
  QLength vertical_circumference = 2.0 * units::pi * units::inch;

  /**
   * @brief Lateral offset of the vertical tracking wheel, positive to the
   *        robot's **right** of the tracking centre.
   */
  QLength vertical_offset_right = 0.0 * units::inch;

  /// @brief Use a horizontal (sideways-rolling) tracking wheel.
  bool use_horizontal_tracker = false;

  /// @brief Circumference of the horizontal tracking wheel.
  QLength horizontal_circumference = 2.0 * units::pi * units::inch;

  /**
   * @brief Longitudinal offset of the horizontal tracking wheel, positive
   *        **ahead** of the tracking centre.
   */
  QLength horizontal_offset_forward = 0.0 * units::inch;
};

/**
 * @brief One raw reading of every sensor the odometry can use.
 *
 * Raw on purpose: degrees off the encoders, radians off the IMU, no
 * pre-processing. Fields for sensors the config does not enable are ignored,
 * so a caller with no horizontal tracker can leave that one at zero.
 */
struct OdometrySample {
  /// @brief IMU heading in radians, compass frame. NAN or inf marks a fault.
  double heading_rad = 0.0;
  /// @brief Left drive encoder position, degrees.
  double left_deg = 0.0;
  /// @brief Right drive encoder position, degrees.
  double right_deg = 0.0;
  /// @brief Vertical tracking wheel position, degrees.
  double vertical_deg = 0.0;
  /// @brief Horizontal tracking wheel position, degrees.
  double horizontal_deg = 0.0;
};

/**
 * @brief Integrates sensor samples into a field pose.
 *
 * Not thread-safe on its own. The library's single instance is reached through
 * `odometryTick()` / `resetOdometry()` below, which hold a lock.
 */
class Odometry {
 public:
  explicit Odometry(OdometryConfig config = {});

  /// @brief Replace the configuration. Does not disturb the pose.
  void setConfig(const OdometryConfig& config);
  /// @brief The configuration in use.
  OdometryConfig getConfig() const;

  /**
   * @brief Move the robot to a known pose and re-seed the sensor baseline.
   *
   * The next `update()` only records the sensor readings and reports @p pose
   * unchanged, so a reset can never manufacture a delta out of whatever the
   * encoders happened to read before it.
   *
   * @warning Heading is tracked as IMU *deltas* from `pose.theta`, so a reset
   * to a theta that disagrees with the IMU permanently offsets the pose frame
   * from the frame `motion.cpp` steers in - it drives on raw
   * `getInertialHeading()` degrees. Reset the IMU to the same heading in the
   * same breath. `Chassis::setPose()` and `wallReset()` both do.
   *
   * A non-finite `pose.theta` is refused and the current heading kept.
   */
  void reset(const Pose2D& pose);

  /**
   * @brief Fold one sample into the pose.
   *
   * @param sample Raw sensor readings.
   * @return The pose after this sample.
   *
   * The first call after construction or `reset()` seeds the baseline and
   * returns the pose untouched. A sample whose heading or enabled encoder
   * reading is not finite is skipped whole - the baseline is left alone, so
   * the distance covered during the dropout is integrated against the next
   * good heading rather than lost or turned into NaN.
   *
   * @note That catch-up is a single arc, and the heading delta across it is
   * wrapped into [-pi, pi]. A dropout long enough for the robot to turn more
   * than 180 degrees is therefore recorded as the short way round, in the
   * wrong direction. At a 10 ms tick that needs a fault lasting most of a
   * second while the robot is spinning; `faultCount()` is there so a caller
   * that cares can notice and re-seed.
   */
  Pose2D update(const OdometrySample& sample);

  /// @brief The current pose.
  Pose2D getPose() const;

  /// @brief False until the first good sample has seeded the baseline.
  bool hasBaseline() const;

  /// @brief How many samples have been rejected as non-finite since construction.
  std::uint32_t faultCount() const;

 private:
  OdometryConfig m_config;
  Pose2D m_pose{};
  OdometrySample m_previous{};
  bool m_has_baseline = false;
  std::uint32_t m_fault_count = 0;
};

// ---------------------------------------------------------------------------
// The library's single odometry instance.
//
// These are the only pose writers in mclib. Each one holds the odometry lock
// and then publishes into RobotState under its lock, always in that order, so
// a reset and a tick can never interleave into a half-reset pose.
// ---------------------------------------------------------------------------

/**
 * @brief Fold one sample in and publish the result to `robotState()`.
 * @return The new pose.
 */
Pose2D odometryTick(const OdometrySample& sample);

/// @brief Teleport the odometry and `robotState()` to a known pose.
void resetOdometry(const Pose2D& pose);

/// @brief Configure the library's odometry instance.
void setOdometryConfig(const OdometryConfig& config);

/// @brief The library odometry instance's configuration.
OdometryConfig getOdometryConfig();

/// @brief How many samples the library instance has rejected as non-finite.
std::uint32_t odometryFaultCount();

}  // namespace control
}  // namespace mclib
