// mclib
#pragma once

/**
 * @file utils.hpp
 * @brief Angle and geometry helpers. Global namespace, degrees at the boundary.
 *
 * The library's canonical field frame is documented in `mclib/math.hpp`:
 * heading 0 points along +Y and increases clockwise. These helpers are the
 * degree/radian bridge between that internal radian representation and the
 * public motion API, which speaks degrees.
 */

/// @brief Degrees -> radians.
double degToRad(double deg);

/// @brief Radians -> degrees.
double radToDeg(double rad);

/**
 * @brief Legacy curvature helper used by `boomerang`. **Frame-buggy -- prefer
 *        `mclib::arcRadius()`.**
 *
 * Literally computes `((x1-x)^2 + (y1-y)^2) / (2 * (y1-y) * sin(90 - angle))`.
 *
 * @param x     Current X, inches (field frame).
 * @param y     Current Y, inches (field frame).
 * @param x1    Target X, inches (field frame).
 * @param y1    Target Y, inches (field frame).
 * @param angle Robot heading in **degrees**, compass frame (0 = +Y,
 *              clockwise-positive).
 * @return A signed length in inches, or `+infinity` when the denominator
 *         vanishes.
 *
 * @warning This is **not** the radius of the circle tangent to the heading
 *          through the target, despite being used as one. The denominator
 *          should be twice the target's lateral offset in the robot frame,
 *          `2 * ((x1-x)*cos(theta) - (y1-y)*sin(theta))`; the code uses
 *          `(y1-y)` where the lateral offset belongs -- the same +X/+Y
 *          transpose this library exists to stamp out. Concretely, a target
 *          10 in dead ahead of a robot at heading 0 is a straight line
 *          (infinite radius), and this returns 5. `mclib::arcRadius()` in
 *          `math.hpp` computes it correctly. Rewiring `boomerang` onto it is
 *          Phase 3 work -- the existing tuning was fitted around this
 *          behavior, so it is left alone here.
 *
 * @note The denominator vanishes when the target is level with the robot in Y
 *       (`y1 == y`) or the heading is exactly +/-90 deg. That is a degenerate
 *       case of a wrong formula, not a real straight line. It previously
 *       returned a magic `999`, which silently became a finite speed limit;
 *       infinity means "do not limit" instead.
 * @note The one caller, `control/motion.cpp:1074`, computes
 *       `sqrt(chase_power * getRadius(...) * 9.8)` as a slip-speed limit.
 *       That expression is dimensionally incoherent -- it mixes a voltage-ish
 *       tuning constant, a radius in inches, and g in m/s^2 -- and it takes
 *       the square root of a value that can be negative. With `chase_power`
 *       at 0 it now yields `0 * inf = NaN`, and every NaN comparison is false,
 *       so the limiter drops out. Fixing that caller is Phase 3 work; this
 *       helper only reports its own geometry honestly.
 */
double getRadius(double x, double y, double x1, double y1, double angle);
