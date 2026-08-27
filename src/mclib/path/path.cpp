// mclib
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

}  // namespace

Path::Path(std::vector<PathPoint> points) : m_points(std::move(points)) {
  recomputeDistances();
}

Path Path::fromWaypoints(const std::vector<Waypoint>& waypoints) {
  std::vector<PathPoint> points;
  points.reserve(waypoints.size());
  for (const Waypoint& waypoint : waypoints) {
    PathPoint point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    points.push_back(point);
  }

  // Heading of the segment arriving at each point; the first point borrows the
  // segment leaving it. Curvature stays zero: a polyline is straight between
  // its vertices and the corners are not differentiable.
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double bearing = headingToward(points[i].point(), points[i + 1].point());
    points[i + 1].heading = QAngle::fromBase(bearing);
    if (i == 0) {
      points[0].heading = QAngle::fromBase(bearing);
    }
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
  out.heading = lerpHeading(a.heading, b.heading, t);
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
