// mclib
#pragma once

#include "Eigen/Core"

/**
 * @file math.hpp
 * @brief Core geometry types and the canonical mclib coordinate frame.
 *
 * ## Canonical frame: compass / field frame
 *
 * Every pose, heading, and angle that describes where the robot is on the
 * field uses the compass convention:
 *
 * - `theta = 0` points along **+Y**.
 * - `theta` increases **clockwise**, so +90 deg points along **+X**.
 * - The unit vector for a heading is `(sin(theta), cos(theta))` -- x uses sin,
 *   y uses cos. This is the transpose of the usual textbook formula.
 * - A bearing from A to B is `atan2(b.x - a.x, b.y - a.y)` -- x first, y
 *   second. Again the transpose of the usual `atan2(y, x)`.
 *
 * This matches how VEX field diagrams are drawn, and it is what
 * `control/odometry.cpp`, `control/motion.cpp`, the odometry task in
 * `control/odometry_task.cpp`, and `snapshot/raycast.cpp` already do. Do not introduce a second
 * convention. If you need to turn a heading into a vector, or a vector into a
 * heading, call `headingVector()` / `headingToward()` below instead of writing
 * sin/cos by hand.
 *
 * ## Units
 *
 * - Internally, every angle in this header is **radians**. `Pose2D::theta`
 *   and `wrapAngle()` are radians.
 * - The public motion API (`Chassis`, `control/motion.cpp`, and
 *   `RobotState::correctAngleDeg()`) is **degrees**. Convert at that boundary with `degToRad` / `radToDeg`
 *   from `utils.hpp`; `Chassis::headingDeg()` already does.
 * - Translations are inches.
 *
 * ## Robot frame
 *
 * The robot frame is chosen so it coincides with the field frame at
 * `theta = 0`: **+Y is forward, +X is to the robot's right**. That keeps
 * `fieldToRobot(v, 0) == v`.
 */

namespace mclib {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTwoPi = 2.0 * kPi;

using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;

/// @brief Clamp `val` into `[min, max]`. Swaps the bounds if they arrive reversed.
double clamp(double val, double min, double max);

/**
 * @brief Wrap an angle in radians into `[-pi, pi]`.
 * @note The interval is closed at both ends: `wrapAngle(-pi)` returns `-pi`,
 *       not `+pi`. Do not assume 180 deg always normalizes to the positive
 *       end.
 */
double wrapAngle(double rad);

/**
 * @brief A field pose: position in inches, heading in radians, compass frame.
 *
 * `theta = 0` faces +Y, positive is clockwise. See the file comment.
 */
struct Pose2D {
  double x;      ///< Field X, inches.
  double y;      ///< Field Y, inches.
  double theta;  ///< Heading, radians, 0 = +Y, clockwise-positive.

  constexpr Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  constexpr Pose2D(double x_in, double y_in, double theta_in)
      : x(x_in), y(y_in), theta(theta_in) {}

  /**
   * @brief Component-wise addition, NOT an SE(2) compose.
   *
   * `x`, `y`, and `theta` are added independently and `theta` is wrapped. The
   * translation of `other` is **not** rotated by this pose's heading, so this
   * does not mean "move `other` in my own frame". Use `compose()` for that.
   *
   * Kept for callers that want to add a field-frame offset to a pose.
   */
  Pose2D operator+(const Pose2D& other) const;

  /// @brief Straight-line distance to `other`, inches. Ignores heading.
  double distanceTo(const Pose2D& other) const;

  /// @brief `(x, y)` as a vector, inches.
  Vec2 translation() const;

  /// @brief `(x, y, theta)` as a vector; theta in radians.
  Vec3 vector() const;
};

/**
 * @brief SE(2) compose in the compass frame: apply `local` in this pose's frame.
 *
 * `local.x` is inches to the robot's right, `local.y` is inches forward, and
 * `local.theta` is a clockwise turn in radians. The translation is rotated by
 * `base.theta` before it is added, which is what `operator+` does not do.
 */
Pose2D compose(const Pose2D& base, const Pose2D& local);

/**
 * @brief Unit vector pointing along a compass heading.
 * @param rad Heading in radians, 0 = +Y, clockwise-positive.
 * @return `(sin(rad), cos(rad))`.
 */
Vec2 headingVector(double rad);

/**
 * @brief Compass bearing from `from` to `to`.
 * @return Radians in `[-pi, pi]` (the `std::atan2` range, closed at both
 *         ends), 0 = +Y, clockwise-positive. Zero if the two points coincide.
 */
double headingToward(const Vec2& from, const Vec2& to);

/**
 * @brief Rotate a field-frame vector into the robot frame.
 * @param field_vec A displacement in field coordinates (inches). This is a
 *        vector, not a point -- no translation is applied.
 * @param heading_rad Robot heading, compass radians.
 * @return `(right, forward)` in robot coordinates. Identity at heading 0.
 */
Vec2 fieldToRobot(const Vec2& field_vec, double heading_rad);

/**
 * @brief Rotate a robot-frame vector into the field frame. Inverse of
 *        `fieldToRobot()`.
 * @param robot_vec `(right, forward)` in inches.
 * @param heading_rad Robot heading, compass radians.
 */
Vec2 robotToField(const Vec2& robot_vec, double heading_rad);

/**
 * @brief Field point -> robot-relative point: translate, then rotate.
 * @return `(right, forward)` of `field_point` as seen from `robot_pose`.
 */
Vec2 fieldPointToRobot(const Vec2& field_point, const Pose2D& robot_pose);

/// @brief Robot-relative point -> field point. Inverse of `fieldPointToRobot()`.
Vec2 robotPointToField(const Vec2& robot_point, const Pose2D& robot_pose);

/**
 * @brief Signed radius of the arc from `from` that passes through `target`.
 *
 * The circle is tangent to `from.theta` at `from` and passes through
 * `target` -- the curvature a pure-pursuit style follower wants.
 *
 * @param from   Robot pose, inches and compass radians.
 * @param target Field point, inches.
 * @return Radius in inches. **Signed**: positive curves to the robot's right,
 *         negative to its left. `+infinity` when `target` is straight ahead
 *         or straight behind, which is a straight line, not an arc.
 *
 * @note The straight-line test is a tolerance, not `lateral == 0.0`. An exact
 *       test split two identical cases: a target 10 in dead ahead of a robot
 *       at heading 0 has a lateral offset of exactly 0 and gave `+infinity`,
 *       but the same target dead behind (heading pi) has a lateral offset of
 *       1.2e-15 from `sin(pi)` and gave -4.08e16 - a finite negative "radius"
 *       whose `sqrt()` is NaN. `+infinity` is now returned whenever
 *       `|lateral| <= 1e-12 * |target - from|^2`, i.e. whenever the radius
 *       would exceed 5e11 inches in magnitude; the derivation is in
 *       `math.cpp`. Note the sentinel is `+infinity` on both sides, including
 *       where the finite result would have been negative: a straight line has
 *       no turn direction to report.
 *
 * @note `target` equal to `from` is the same straight-line answer, not a
 *       `0 / 0` NaN.
 *
 * @note This is the correct compass-frame version of `getRadius()` in
 *       `utils.hpp`. Prefer this one; see that function's warning.
 */
double arcRadius(const Pose2D& from, const Vec2& target);

/**
 * @brief Standard-frame (textbook) rotation matrix. **Not the field frame.**
 *
 * Counter-clockwise-positive with 0 along +X:
 * `[[cos, -sin], [sin, cos]]`. Use it for generic linear algebra only.
 *
 * Feeding it a compass heading turns the wrong way: `rotate({0, 1}, rad)`
 * gives `(-sin, cos)`, while a robot at that heading actually points at
 * `(sin, cos)`. Because the two frames are transposes, `rotationMatrix(rad)`
 * happens to equal the field->robot matrix, so `rotate()` with a compass
 * heading silently performs `fieldToRobot()` -- the inverse of what someone
 * writing "rotate my local offset into the field" expects.
 *
 * To move a pose or a waypoint between the field and the robot, call
 * `fieldToRobot()` / `robotToField()` instead.
 */
Mat2 rotationMatrix(double rad);

/**
 * @brief Rotate `vec` counter-clockwise by `rad` in the standard frame.
 *        **Not the field frame** -- see `rotationMatrix()`.
 */
Vec2 rotate(const Vec2& vec, double rad);

}  // namespace mclib
