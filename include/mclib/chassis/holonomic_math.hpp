// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @file holonomic_math.hpp
 * @brief The arithmetic behind HolonomicChassis and HolonomicController.
 *
 * Same split as `chassis_math.hpp`: everything here is numbers in, numbers
 * out, with no motors, sensors or clock, so it is built into the host tests
 * (`HOST_TEST_SRC`) and covered by `tests/holonomic_math_test.cpp`.
 *
 * ## Frames and signs
 *
 * The robot frame is the one in `math.hpp`: **+Y forward, +X to the robot's
 * right**. The field frame is the compass frame: 0 = +Y, clockwise positive.
 * Turn is positive **clockwise**, like every other turn in the library.
 *
 * Wheels are named by where they sit looking down at the robot with forward
 * at the top: front_left, front_right, back_left, back_right. Positive wheel
 * speed drives that wheel's contact patch so as to push the robot forward.
 */

namespace mclib {
namespace holonomic {

/// @brief Which holonomic layout the wheels are in. See mix().
enum class Kind {
  /// Four omni wheels at 45 deg in an X.
  XDrive,
  /// Four mecanum wheels, rollers forming an X when viewed from above.
  Mecanum,
};

/// @brief One command per wheel. Fractions of full power, or volts - the caller's choice.
struct WheelSpeeds {
  double front_left = 0.0;
  double front_right = 0.0;
  double back_left = 0.0;
  double back_right = 0.0;
};

/**
 * @brief Robot-centric forward, strafe and turn to four wheel commands.
 *
 * The four sums are
 *
 *     front_left  = forward + strafe + turn
 *     front_right = forward - strafe - turn
 *     back_left   = forward - strafe + turn
 *     back_right  = forward + strafe - turn
 *
 * Pure forward drives every wheel the same way. A right strafe pushes the
 * front-left and back-right wheels forward and the other diagonal backward,
 * which is the pair whose rollers (or 45 deg axes) point front-right. A
 * clockwise turn drives the left side forward and the right side backward.
 *
 * Both kinds use the same sums. On an X-drive the wheels sit at 45 deg, so
 * forward and strafe reach each wheel with the same weight; on mecanum the
 * rollers do the same job. What differs is what a unit of strafe *does* -
 * an X-drive strafes as fast as it drives, mecanum noticeably slower - and
 * that is a property of the robot, not of the mix. @p kind is taken so a
 * caller says which robot it has and so the two can diverge without an API
 * change.
 *
 * If any wheel would exceed 1 in magnitude, all four are divided by the
 * largest magnitude, so ratios (and therefore the direction of travel) are
 * preserved and `max |w| <= 1`. Non-finite inputs are treated as 0.
 *
 * @param forward Forward command, -1..1. Positive drives forward.
 * @param strafe  Sideways command, -1..1. Positive drives to the robot's **right**.
 * @param turn    Turn command, -1..1. Positive turns **clockwise**.
 * @param kind    XDrive or Mecanum.
 */
WheelSpeeds mix(double forward, double strafe, double turn, Kind kind);

/// @brief A field-frame command expressed in the robot frame.
struct RobotFrameInput {
  double forward = 0.0;  ///< Along the robot's +Y.
  double strafe = 0.0;   ///< Along the robot's +X (to its right).
};

/**
 * @brief Rotate a field-frame (x, y) command into the robot frame.
 *
 * The same rotation as `mclib::fieldToRobot()` in `math.hpp`, without the
 * Eigen dependency: at heading 0 the two frames coincide, at heading +90 deg
 * (facing +X) a field +X command becomes pure forward and a field +Y command
 * becomes a strafe to the **left**.
 *
 * @param field_x     Command along field +X.
 * @param field_y     Command along field +Y.
 * @param heading_rad Robot heading, compass frame (0 = +Y, clockwise
 *                    positive), radians. Need not be wrapped.
 */
RobotFrameInput fieldToRobot(double field_x, double field_y, double heading_rad);

}  // namespace holonomic
}  // namespace mclib
