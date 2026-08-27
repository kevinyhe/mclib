// mclib
#pragma once

/**
 * @file spline.hpp
 * @brief Smooth path generation from waypoints: centripetal Catmull-Rom.
 *
 * ## Why Catmull-Rom and not Bezier
 *
 * A VEX auton is written as "drive through these field positions". Catmull-Rom
 * **interpolates**: the curve passes through every waypoint you type, and the
 * only tuning knob is how tight the corners are. A cubic or quintic Bezier
 * approximates instead - the interior control points are not on the curve, so
 * the author has to place handles that have no physical meaning on a field
 * diagram, and the path does not go through the numbers they wrote down.
 *
 * The parameterisation is **centripetal** (alpha = 0.5) by default. Uniform
 * Catmull-Rom (alpha = 0) overshoots and can form a cusp or a self-
 * intersecting loop when waypoint spacing is uneven, which is exactly the
 * shape that makes a pure-pursuit follower spin. Centripetal is proven not to
 * cusp or self-intersect, at the cost of slightly wider corners.
 *
 * ## Continuity
 *
 * The tangent at every knot is shared by the segments on both sides, so the
 * curve is **C1**: position and heading are continuous across a waypoint. It
 * is not C2 - curvature steps at a waypoint. That is the accepted trade for
 * interpolation with local control, and the velocity limiter reads curvature
 * over a whole lookahead window rather than at a point, so a step does not
 * become a torque step.
 *
 * Frame and curvature sign are those of `path.hpp`: compass headings,
 * positive curvature curves to the robot's right.
 */

#include "mclib/path/path.hpp"
#include "mclib/units/units.hpp"

#include <vector>

namespace mclib {
namespace path {

/**
 * @brief Knobs for `generateSpline()`.
 */
struct SplineConfig {
  /**
   * @brief Target arc-length spacing between baked samples.
   *
   * @details Smaller is a better approximation and a longer bake. One inch
   * over a 100 inch route is ~100 samples, which is nothing on the brain and
   * keeps the follower's segment geometry accurate to well under a wheel
   * width.
   */
  QLength spacing = 1.0 * units::inch;

  /**
   * @brief Corner tightness, 0 to 1.
   *
   * @details Scales the knot tangents. 0 is plain Catmull-Rom. 1 zeroes the
   * tangents, which turns the curve into a set of straight-ish segments that
   * meet at the waypoints with a hard heading change - almost never what you
   * want. Values around 0.2 pull the curve closer to the polyline.
   */
  double tension = 0.0;

  /**
   * @brief Use centripetal (alpha = 0.5) parameterisation.
   *
   * @details Leave this on. Off means uniform Catmull-Rom, which can cusp and
   * loop; it exists so a test can demonstrate the difference.
   */
  bool centripetal = true;

  /// @brief Lower bound on samples per segment, whatever `spacing` says.
  int min_samples_per_segment = 4;
};

/**
 * @brief Bake a centripetal Catmull-Rom spline through @p waypoints.
 *
 * @param waypoints Field positions to pass through, in order. Consecutive
 *        duplicates are dropped.
 * @param config Sampling and shape knobs.
 * @return A baked Path with heading and signed curvature on every sample.
 *         Fewer than two distinct waypoints yields an unfollowable Path
 *         (`valid()` is false); exactly two yields the straight line between
 *         them, since a spline through two points is a line.
 *
 * @note End conditions: the phantom points before the first and after the last
 *       waypoint are reflections (`2*P0 - P1`), which makes the curve leave
 *       the first waypoint and arrive at the last one aimed straight at its
 *       neighbour.
 */
Path generateSpline(const std::vector<Waypoint>& waypoints,
                    const SplineConfig& config = {});

}  // namespace path
}  // namespace mclib
