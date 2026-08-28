// mclib
#pragma once

/**
 * @file path.hpp
 * @brief A baked, arc-length-parameterised field path and its waypoints.
 *
 * @details A Path is an ordered list of samples in field coordinates. Each
 * sample carries its position, the compass heading of the tangent there, the
 * signed curvature, and the arc length from the start of the path. Queries by
 * arc length or by normalised parameter interpolate between samples.
 *
 * Baking the path into samples once, instead of evaluating a spline inside the
 * control loop, is deliberate. The follower then only ever does segment
 * geometry - projection and circle/segment intersection - which is cheap on
 * the V5 brain and identical for a smoothed spline and a raw waypoint
 * polyline.
 *
 * Everything here is PROS-free and host-testable.
 *
 * ## Frame
 *
 * Field frame, compass convention: heading 0 is +Y, clockwise-positive. See
 * `math.hpp`. Curvature is **signed the same way as `mclib::arcRadius()`**:
 * positive curves to the robot's right (clockwise), negative to its left.
 */

#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>
#include <vector>

namespace mclib {
namespace path {

using units::QAngle;
using units::QCurvature;
using units::QLength;

/**
 * @brief A user-supplied point the path should pass through, in field inches.
 */
struct Waypoint {
  QLength x{};  ///< Field X.
  QLength y{};  ///< Field Y.

  constexpr Waypoint() = default;
  constexpr Waypoint(QLength x_in, QLength y_in) : x(x_in), y(y_in) {}

  /// @brief `(x, y)` in inches, for the geometry helpers in `math.hpp`.
  Vec2 point() const { return Vec2{x.in(), y.in()}; }
};

/**
 * @brief One baked sample along a path.
 */
struct PathPoint {
  QLength x{};             ///< Field X.
  QLength y{};             ///< Field Y.
  QAngle heading{};        ///< Compass heading of the tangent.
  QCurvature curvature{};  ///< Signed: positive curves right (clockwise).
  QLength distance{};      ///< Arc length from the start of the path.

  /// @brief `(x, y)` in inches.
  Vec2 point() const { return Vec2{x.in(), y.in()}; }

  /// @brief Position plus tangent heading as a field pose (inches, radians).
  Pose2D pose() const { return Pose2D{x.in(), y.in(), heading.rad()}; }
};

/**
 * @brief An ordered sequence of baked samples in field coordinates.
 *
 * @details A Path with fewer than two points cannot be followed; `valid()`
 * says so. Samples are in increasing arc-length order, which every builder
 * here guarantees.
 */
class Path {
 public:
  Path() = default;

  /**
   * @brief Adopt an already-built sample list.
   * @param points Samples in order. `distance` is recomputed from the chord
   *        lengths, so callers only have to get the positions right.
   */
  explicit Path(std::vector<PathPoint> points);

  /**
   * @brief A straight-segment path through the waypoints, with no smoothing.
   *
   * @details Heading at each waypoint is the heading of the segment *leaving*
   * it (the last waypoint carries the segment that arrived), and curvature is
   * zero everywhere. Consecutive duplicate waypoints are dropped: a repeated
   * point has no direction, and `headingToward()` answers 0 for coincident
   * points, which is a real heading pointing along +Y and wrong.
   *
   * A polyline's heading is only defined per segment, and `atDistance()`
   * reports exactly that: the bearing of the segment the query lands on, with
   * the change stepping at the vertex. Code that wants the segment direction
   * without an `atDistance()` call - `PurePursuit` signing its cross-track
   * error, for one - can take the bearing between the bracketing samples
   * instead.
   *
   * Use this when the route really is a polyline, or in tests where an exactly
   * known geometry matters more than smoothness. For a smooth route use
   * `generateSpline()` in `spline.hpp`.
   */
  static Path fromWaypoints(const std::vector<Waypoint>& waypoints);

  /// @brief Number of samples.
  std::size_t size() const { return m_points.size(); }

  /// @brief True when there are no samples at all.
  bool empty() const { return m_points.empty(); }

  /// @brief True when the path has at least one segment to follow.
  bool valid() const { return m_points.size() >= 2; }

  /// @brief The samples, in order.
  const std::vector<PathPoint>& points() const { return m_points; }

  /// @brief Sample @p index, clamped to the ends. Returns a zero sample if empty.
  const PathPoint& at(std::size_t index) const;

  /// @brief Sample @p index, unchecked. Prefer `at()` outside hot loops.
  const PathPoint& operator[](std::size_t index) const { return m_points[index]; }

  /// @brief First sample, or a zero sample if empty.
  const PathPoint& front() const { return at(0); }

  /// @brief Last sample, or a zero sample if empty.
  const PathPoint& back() const { return at(m_points.empty() ? 0 : m_points.size() - 1); }

  /// @brief Total arc length, i.e. the last sample's `distance`. Zero if empty.
  QLength length() const;

  /**
   * @brief The sample at arc length @p distance from the start.
   * @details Linearly interpolates position and curvature between the two
   *          bracketing samples. Clamped to the ends, so running off either
   *          end returns an endpoint rather than extrapolating.
   *
   *          Heading is interpolated the short way round, but **paced by the
   *          curvature, not by the arc length**: curvature is dtheta/ds, so
   *          integrating it across the segment says where inside the segment
   *          the turn happens. On a spline that is the plain lerp. On a
   *          polyline the curvature is zero, so the heading holds at the
   *          segment's own bearing and steps at the vertex, which is where the
   *          corner is. A hand-built `Path` that carries per-sample headings
   *          but leaves `curvature` at zero gets the stepped behaviour too;
   *          fill in curvature if you want the headings blended.
   */
  PathPoint atDistance(QLength distance) const;

  /**
   * @brief The sample at normalised parameter @p t: 0 is the start, 1 the end.
   *        Parameterised by arc length, not by spline knot.
   */
  PathPoint atParameter(double t) const;

  /**
   * @brief Largest absolute curvature over the arc-length window
   *        `[from, to]`, clamped to the path.
   *
   * @details What a velocity limit wants: the tightest thing coming up inside
   * the braking window, not just the curvature underfoot.
   */
  QCurvature maxAbsCurvature(QLength from, QLength to) const;

 private:
  /// @brief Recompute `distance` on every sample from the chord lengths.
  void recomputeDistances();

  std::vector<PathPoint> m_points;
};

}  // namespace path
}  // namespace mclib
