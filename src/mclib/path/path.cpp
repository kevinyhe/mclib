// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/path/path.hpp"

#include <cmath>

namespace mclib {
namespace path {

namespace {

/// @brief A shared zero sample, so at()/front()/back() on an empty path have
///        something to return by reference instead of dangling.
const PathPoint& zeroPoint() {
  static const PathPoint kZero{};
  return kZero;
}

/// @brief Interpolate two compass headings the short way round.
QAngle lerpHeading(QAngle from, QAngle to, double t) {
  const double delta = wrapAngle(to.rad() - from.rad());
  return QAngle::fromBase(wrapAngle(from.rad() + delta * t));
}

/**
 * @brief What fraction of a segment's heading change has happened @p s of arc
 *        length into it.
 *
 * @details Curvature *is* dtheta/ds, so integrating it across the segment says
 * where inside the segment the turning actually happens, instead of assuming
 * it is spread evenly along the arc.
 *
 * On a spline the curvature is roughly constant across one sample spacing, the
 * integral comes out as `s / span`, and this is the plain lerp it replaces. On
 * a polyline the curvature is zero, the integral is zero, and the heading
 * holds at the segment's own bearing until the far vertex - which is where a
 * polyline's corner is. The old lerp spread a corner over the whole segment
 * leading into it, so the reported heading was off by up to the full corner
 * angle for the entire leg.
 *
 * @param curvature_a Curvature at the near sample, base units (1/m).
 * @param curvature_b Curvature at the far sample.
 * @param span Arc length of the segment, base units (m). Must be > 0.
 * @param s Arc length into the segment, 0 to @p span.
 * @return 0 to 1. Always 1 at `s == span`, so the heading is continuous across
 *         samples whatever the curvature does.
 */
double turnFraction(double curvature_a, double curvature_b, double span, double s) {
  // Magnitudes, not signed values. A segment that straddles an inflection point
  // carries `+k` at one end and `-k` at the other, and the signed integral
  // cancels to zero there - which would read as "straight" and step the heading
  // at the vertex, on a curve that is anything but. Turning right then left is
  // still turning all the way along.
  const double near = std::fabs(curvature_a);
  const double far = std::fabs(curvature_b);
  const double total = span * 0.5 * (near + far);
  // A genuinely straight segment turns nowhere inside itself: the whole heading
  // change belongs to the vertex at its far end, which is where a polyline's
  // corner is.
  if (span <= 0.0 || total < 1e-9) {
    return 0.0;
  }
  const double u = clamp(s / span, 0.0, 1.0);
  const double partial = span * (near * u + (far - near) * u * u * 0.5);
  // The integrand is non-negative, so this is already monotone in [0, 1] and
  // exactly 1 at u = 1. Clamped anyway, against rounding.
  return clamp(partial / total, 0.0, 1.0);
}

}  // namespace

Path::Path(std::vector<PathPoint> points) {
  m_points.reserve(points.size());
  for (const PathPoint& point : points) {
    // Apply the same invariant as fromWaypoints to raw samples. Zero-length
    // segments have no tangent and corrupt projection/cross-track telemetry.
    // Keep the first sample's metadata, matching the waypoint builder.
    if (!m_points.empty() &&
        (point.point() - m_points.back().point()).norm() < 1e-9) {
      continue;
    }
    m_points.push_back(point);
  }
  recomputeDistances();
}

Path Path::fromWaypoints(const std::vector<Waypoint>& waypoints) {
  std::vector<PathPoint> points;
  points.reserve(waypoints.size());
  for (const Waypoint& waypoint : waypoints) {
    PathPoint point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    // A repeated waypoint has no direction, and headingToward() answers 0 for
    // coincident points - a real heading, pointing along +Y, and wrong. Drop
    // the duplicate instead of recording it.
    if (!points.empty() && (point.point() - points.back().point()).norm() < 1e-9) {
      continue;
    }
    points.push_back(point);
  }

  // Heading of the segment *leaving* each vertex; the last vertex carries the
  // segment that arrived at it. Curvature stays zero: a polyline is straight
  // between its vertices and its corners are not differentiable.
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    points[i].heading = QAngle::fromBase(
        headingToward(points[i].point(), points[i + 1].point()));
  }
  if (points.size() >= 2) {
    points.back().heading = points[points.size() - 2].heading;
  }

  return Path(std::move(points));
}

void Path::recomputeDistances() {
  if (m_points.empty()) {
    return;
  }
  m_points[0].distance = QLength{};
  for (std::size_t i = 1; i < m_points.size(); ++i) {
    const double dx = m_points[i].x.raw() - m_points[i - 1].x.raw();
    const double dy = m_points[i].y.raw() - m_points[i - 1].y.raw();
    m_points[i].distance =
        m_points[i - 1].distance + QLength::fromBase(std::hypot(dx, dy));
  }
}

const PathPoint& Path::at(std::size_t index) const {
  if (m_points.empty()) {
    return zeroPoint();
  }
  if (index >= m_points.size()) {
    return m_points.back();
  }
  return m_points[index];
}

QLength Path::length() const {
  return m_points.empty() ? QLength{} : m_points.back().distance;
}

PathPoint Path::atDistance(QLength distance) const {
  if (m_points.empty()) {
    return PathPoint{};
  }
  if (m_points.size() == 1 || distance <= QLength{}) {
    return m_points.front();
  }
  if (distance >= length()) {
    return m_points.back();
  }

  // Binary search for the last sample at or before `distance`.
  std::size_t lo = 0;
  std::size_t hi = m_points.size() - 1;
  while (hi - lo > 1) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (m_points[mid].distance <= distance) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const PathPoint& a = m_points[lo];
  const PathPoint& b = m_points[hi];
  const double span = b.distance.raw() - a.distance.raw();
  const double t = span > 0.0 ? (distance.raw() - a.distance.raw()) / span : 0.0;

  PathPoint out;
  out.x = a.x + (b.x - a.x) * t;
  out.y = a.y + (b.y - a.y) * t;
  out.heading = lerpHeading(
      a.heading, b.heading,
      turnFraction(a.curvature.raw(), b.curvature.raw(), span,
                   distance.raw() - a.distance.raw()));
  out.curvature = a.curvature + (b.curvature - a.curvature) * t;
  out.distance = distance;
  return out;
}

PathPoint Path::atParameter(double t) const {
  return atDistance(length() * clamp(t, 0.0, 1.0));
}

QCurvature Path::maxAbsCurvature(QLength from, QLength to) const {
  if (m_points.empty()) {
    return QCurvature{};
  }
  if (to < from) {
    const QLength tmp = from;
    from = to;
    to = tmp;
  }

  // The endpoints of the window matter as much as the samples inside it, so
  // include the interpolated values at both ends.
  double best = std::fabs(atDistance(from).curvature.raw());
  best = std::fmax(best, std::fabs(atDistance(to).curvature.raw()));

  // Binary search to the first sample inside the window, so a caller that
  // walks a long path does not rescan the part already driven on every tick.
  std::size_t lo = 0;
  std::size_t hi = m_points.size();
  while (lo < hi) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (m_points[mid].distance < from) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  for (std::size_t i = lo; i < m_points.size() && m_points[i].distance <= to; ++i) {
    best = std::fmax(best, std::fabs(m_points[i].curvature.raw()));
  }
  return QCurvature::fromBase(best);
}

}  // namespace path
}  // namespace mclib
