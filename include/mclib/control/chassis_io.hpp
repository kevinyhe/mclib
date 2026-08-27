// mclib
#pragma once

#include "mclib/device/types.hpp"
#include "mclib/units/units.hpp"

/**
 * @file chassis_io.hpp
 * @brief The drive base as the motion routines see it: two sides and an IMU.
 *
 * Everything here talks to the globals in `config.cpp` directly. The four
 * sensor readers still return `double` rather than a `QAngle`, because
 * `control/odometry_task.cpp` reads them on the odometry task's hot path and
 * the odometry pipeline is degrees-and-radians throughout; typing them is a
 * separate change to a file this one does not own. Their units are stated
 * below instead.
 */

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
 * @brief The IMU heading in **degrees**, compass frame, unwrapped.
 *
 * Not wrapped to +/-180: it keeps counting past a full turn, which is what
 * normalizeTarget() exists to compensate for.
 */
double getInertialHeading();

/**
 * @brief Shift @p angle by whole turns until it is within 180 deg of the IMU.
 *
 * Both the argument and the result are **degrees** in the same unwrapped frame
 * as getInertialHeading(). Turning to the normalised target is what makes a
 * heading PID take the short way round.
 */
double normalizeTarget(double angle);
