// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/path/arc.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mclib {
namespace path {

ArcPlan planArc(const Pose2D& start, units::QAngle end_heading, units::QLength signed_radius,
                units::QLength spacing) {
  ArcPlan plan;
  const double r = signed_radius.in();
  const double turn = wrapAngle(end_heading.rad() - start.theta);
  if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.theta) ||
      !std::isfinite(r) || !std::isfinite(turn) || std::fabs(r) < 1e-9 ||
      std::fabs(turn) < 1e-9) {
    return plan;
  }
  // Forward when the centre side and the turn direction agree.
  plan.reversed = (r > 0) != (turn > 0);

  // Centre r along the robot's right at the start: right is (cos, -sin).
  const double cx = start.x + r * std::cos(start.theta);
  const double cy = start.y - r * std::sin(start.theta);
  const double length = std::fabs(r * turn);
  const double step = std::max(spacing.in(), 1e-3);
  const int samples = std::max(2, static_cast<int>(std::ceil(length / step)));
  // Curvature relative to travel: positive when the travel heading turns
  // clockwise, which is the body's turn direction either way.
  const units::QCurvature curvature =
      units::QCurvature::fromBase((turn > 0 ? 1.0 : -1.0) / (std::fabs(r) * units::inch).raw());

  std::vector<PathPoint> points;
  points.reserve(samples + 1);
  for (int i = 0; i <= samples; ++i) {
    const double theta = start.theta + turn * i / samples;
    PathPoint point;
    point.x = (cx - r * std::cos(theta)) * units::inch;
    point.y = (cy + r * std::sin(theta)) * units::inch;
    point.heading = units::QAngle::fromBase(wrapAngle(theta + (plan.reversed ? kPi : 0.0)));
    point.curvature = curvature;
    points.push_back(point);
  }
  plan.path = Path(points);
  return plan;
}

}  // namespace path
}  // namespace mclib
