// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/path/pose_path.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mclib {
namespace path {

namespace {

struct Bezier {
  Vec2 p0, p1, p2, p3;

  Vec2 position(double t) const {
    const double u = 1 - t;
    return p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t);
  }
  Vec2 velocity(double t) const {
    const double u = 1 - t;
    return (p1 - p0) * (3 * u * u) + (p2 - p1) * (6 * u * t) + (p3 - p2) * (3 * t * t);
  }
  Vec2 acceleration(double t) const {
    return (p2 - p1 * 2 + p0) * (6 * (1 - t)) + (p3 - p2 * 2 + p1) * (6 * t);
  }
};

}  // namespace

Path planToPose(const Pose2D& start, const Pose2D& end, bool reversed, double lead,
                units::QLength spacing) {
  const double inch = units::inch.raw();
  const Vec2 from{start.x * inch, start.y * inch};
  const Vec2 to{end.x * inch, end.y * inch};
  const double chord = (to - from).norm();
  if (!std::isfinite(chord) || !std::isfinite(start.theta) || !std::isfinite(end.theta) ||
      !std::isfinite(lead) || !(lead > 0) || chord < 1e-6 * inch) {
    return Path{};
  }
  const double travel_flip = reversed ? kPi : 0.0;
  const double arm = lead * chord;
  const Bezier curve{from, from + headingVector(start.theta + travel_flip) * arm,
                     to - headingVector(end.theta + travel_flip) * arm, to};

  const double polygon = (curve.p1 - curve.p0).norm() + (curve.p2 - curve.p1).norm() +
                         (curve.p3 - curve.p2).norm();
  const double step = std::max(spacing.raw(), 1e-3 * inch);
  const int samples = std::max(2, static_cast<int>(std::ceil(polygon / step)));

  std::vector<PathPoint> points;
  points.reserve(samples + 1);
  Vec2 previous_velocity = curve.velocity(0);
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    const Vec2 position = curve.position(t);
    const Vec2 velocity = curve.velocity(t);
    const double speed = velocity.norm();
    // A target behind the start heading, or arriving against it, folds the
    // curve back on itself: the robot would have to stop and reverse.
    if (!(speed > 1e-6 * chord) || !(velocity.dot(previous_velocity) > 0)) return Path{};
    previous_velocity = velocity;
    const Vec2 acceleration = curve.acceleration(t);
    PathPoint point;
    point.x = units::QLength::fromBase(position.x());
    point.y = units::QLength::fromBase(position.y());
    point.heading = units::QAngle::fromBase(std::atan2(velocity.x(), velocity.y()));
    const double cross = velocity.y() * acceleration.x() - velocity.x() * acceleration.y();
    point.curvature = units::QCurvature::fromBase(cross / (speed * speed * speed));
    points.push_back(point);
  }
  return Path(points);
}

}  // namespace path
}  // namespace mclib
