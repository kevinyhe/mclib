// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @file chassis_math.hpp
 * @brief The pure arithmetic behind Chassis and ChassisController.
 *
 * `chassis.cpp` and `chassis_controller.cpp` both reach PROS through
 * `device/`, so neither compiles on a host machine and neither can be tested
 * there. Everything in this header is the part of those classes that has no
 * motors, no sensors and no clock in it: same numbers in, same numbers out.
 * It is compiled into the host test build (`HOST_TEST_SRC`) and covered by
 * `tests/chassis_math_test.cpp`.
 *
 * Same split, and same reasoning, as `control/motion_math.hpp`: raw `double`
 * volts and degrees, because this is the inner loop and round-tripping through
 * the unit types is not bit-exact.
 *
 * ## Sign convention
 *
 * One convention across the whole library: **positive turn is clockwise**,
 * and the left side leads. `tank(+x, -x)` turns clockwise,
 * `arcadeMix(0, +turn)` turns clockwise, and a positive heading error
 * (target above current in the compass frame) asks for a clockwise turn.
 */

namespace mclib {
namespace chassis_math {

/// @brief A left/right command pair. Volts, or -1..1 fractions - the caller's choice.
struct DrivePair {
  double left = 0.0;
  double right = 0.0;
};

/// Unwrapped clockwise-positive heading from differential wheel travel.
/// Distances and track width must use the same unit. Invalid geometry returns NaN.
double encoderHeadingDeg(double left_distance, double right_distance, double track_width);

/**
 * @brief Shift @p target_deg by whole turns until it is within 180 deg of
 *        @p current_deg.
 *
 * Both are degrees in the **unwrapped** frame that `Inertial::getRotationDeg()`
 * reports - it keeps counting past a full turn, so after two clockwise
 * revolutions "0 deg" is 720, not 0. Turning to the normalised target is what
 * makes a heading PID take the short way round.
 *
 * Identical semantics to `control::normalizeTarget()` in `chassis_io.cpp`,
 * which is the same function with the IMU read baked in. The two turn paths in
 * this library agreed on nothing before; they agree now.
 *
 * The result is always within [-180, +180] of @p current_deg. A target exactly
 * 180 away is left on whichever side it arrived, same as `normalizeTarget()`:
 * both are half a turn, and nothing sensible distinguishes them.
 */
double normalizeHeadingTarget(double target_deg, double current_deg);

/**
 * @brief Split a common drive term and a heading correction into a left/right
 *        pair that fits the voltage rail **without losing the differential**.
 *
 * The obvious version - clamp `drive + correction` and `drive - correction`
 * independently - throws the correction away exactly when it is needed. With
 * the default `distance_pid.kp = 0.4`, any move longer than 30 in starts with
 * a drive term above 12 V, both clamps return the cap, and `left - right` is
 * zero: no correction authority at all for the whole first half of the move.
 *
 * This caps the pair by shifting **both** sides by the same amount, so
 * `left - right` stays exactly `2 * correction`. Worked example - a 40 in move
 * with a 5 deg heading error, at the default gains:
 *
 * | | drive | correction | left | right | left - right |
 * |---|---|---|---|---|---|
 * | before | 16.0 V | 1.5 V | 12.0 V | 12.0 V | **0.0 V** |
 * | after  | 16.0 V | 1.5 V | 12.0 V |  9.0 V | **3.0 V** |
 *
 * Not `scaleToMax()` from `control/scaling.hpp`: that preserves the *ratio* of
 * the two sides, which shrinks the differential in proportion to how far into
 * saturation the drive term is. A common-mode shift preserves the difference
 * itself, which is the quantity the heading loop is actually asking for.
 *
 * The correction is capped at **half** the rail first. That is already a
 * differential of the whole rail - hard over - and it leaves the other half
 * for the drive term. Letting the correction have the full rail instead makes
 * `left` and `right` equal and opposite: a pure turn, mean zero, no forward
 * progress at all. On a slow approach (`max_voltage = 3 V`, heading kp = 0.3)
 * that is reached at 10 deg of error, and a move whose heading error will not
 * clear - a wheel against a wall - would then sit spinning for its whole
 * timeout having travelled nowhere.
 *
 * @param drive_volts      Common forward term, volts. Clamp it to the rail
 *                         before calling if it comes from an unbounded PID.
 * @param correction_volts Heading PID output, volts. Positive steers clockwise.
 *                         Capped at half of @p max_volts.
 * @param max_volts        Voltage cap, magnitude. Non-positive returns {0, 0}.
 */
DrivePair mixDriveCorrection(double drive_volts,
                             double correction_volts,
                             double max_volts);

/**
 * @brief The arcade mix: forward plus turn, left side leading.
 *
 * @param forward Forward command; positive drives forward.
 * @param turn    Turn command; positive turns **clockwise**, matching
 *                `tank(+x, -x)` and a positive `turnToHeading()` delta.
 *                It used to be the other way round here and nowhere else, so
 *                the right stick turned the robot left.
 */
DrivePair arcadeMix(double forward, double turn);

/**
 * @brief The curvature ("cheesy drive") mix: turn authority scales with speed.
 *
 * At full forward a small stick deflection is a gentle arc; at low speed the
 * same deflection is a tight one. The differential is `|forward| * turn`, so
 * the turn stick sets the *curvature* of the path rather than a fixed
 * left/right difference. A car steers this way, which is why drivers find it
 * easier to hold a line at speed than with arcadeMix().
 *
 * With @p forward at exactly 0 the differential would also be 0 and the robot
 * could not turn at all, so that case switches to a plain in-place turn when
 * @p turn_in_place_when_stopped is true, and returns {0, 0} when it is false.
 * The switch is on an exact zero: shape the stick with
 * `control::shapeDriveInput()` first, so a stick resting at 0.008 reads as 0
 * instead of giving almost no turn authority.
 *
 * The pair is scaled down to fit [-1, 1] preserving its ratio, the same way
 * `control::scaleToMax()` does, so `curvatureMix(1, 1)` is {1, 0}, not {2, 0}.
 *
 * `|forward|`, not `forward`: the differential keeps the sign of @p turn, so
 * a positive turn rotates the robot clockwise whether it is driving forward
 * or backward. That matches every other turn in the library. (A car in
 * reverse does the opposite; drivers of a robot generally do not want that.)
 *
 * @param forward Forward command, -1..1; positive drives forward.
 * @param turn    Turn command, -1..1; positive turns **clockwise**.
 * @param turn_in_place_when_stopped What `forward == 0` does: spin in place
 *                (true) or nothing (false).
 */
DrivePair curvatureMix(double forward,
                       double turn,
                       bool turn_in_place_when_stopped = true);

}  // namespace chassis_math
}  // namespace mclib
