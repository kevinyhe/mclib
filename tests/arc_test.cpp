// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file arc_test.cpp
 * @brief planArc() matches curveCircle()'s geometry and sign rules.
 *
 * Every case starts at the origin facing +Y. A 90 deg turn on a 24 in radius
 * ends 24 in over and 24 in along, and the four sign combinations pick the
 * four quadrants. The same targets are what the physics simulator's arc
 * scenarios check.
 */

#include "mclib/math.hpp"
#include "mclib/path/arc.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>

using namespace mclib::units::literals;
using mclib::Pose2D;
using mclib::path::ArcPlan;
using mclib::path::planArc;

namespace {

constexpr double kDeg = mclib::kPi / 180.0;

void checkArc(const char* name, const Pose2D& start, QAngle end, QLength radius,
              double x, double y, bool reversed, double end_travel_deg) {
  const ArcPlan plan = planArc(start, end, radius);
  const auto& back = plan.path.back();
  std::printf("   %-28s ends (%.3f, %.3f), travel heading %.1f deg, %s, %.3f in long\n",
              name, back.x.in(), back.y.in(), back.heading.rad() / kDeg,
              plan.reversed ? "backward" : "forward", plan.path.length().in());
  CHECK(plan.path.valid());
  CHECK_NEAR(back.x.in(), x, 1e-9);
  CHECK_NEAR(back.y.in(), y, 1e-9);
  CHECK_EQ(plan.reversed, reversed);
  CHECK_NEAR(mclib::wrapAngle(back.heading.rad() - end_travel_deg * kDeg), 0.0, 1e-9);
  // Samples lie on the circle, so the baked length is a hair under r * turn.
  const double turn = std::fabs(mclib::wrapAngle(end.rad() - start.theta));
  CHECK_NEAR(plan.path.length().in(), std::fabs(radius.in()) * turn, 0.01);
  // Constant curvature, positive when the travel heading turns clockwise.
  const double k = plan.path.front().curvature.raw();
  CHECK_NEAR(std::fabs(k), 1.0 / (std::fabs(radius.in()) * mclib::units::inch).raw(), 1e-9);
  CHECK_EQ(k > 0, mclib::wrapAngle(end.rad() - start.theta) > 0);
}

}  // namespace

int main() {
  const Pose2D origin{0, 0, 0};
  std::printf("-- the four sign combinations from the origin, facing +Y\n");
  // Centre right, turning clockwise: forward, curving right.
  checkArc("forward right (+24, +90)", origin, 90_deg, 24_in, 24, 24, false, 90);
  // Centre left, turning counter-clockwise: forward, curving left.
  checkArc("forward left (-24, -90)", origin, -90_deg, -24_in, -24, 24, false, -90);
  // Centre right, turning counter-clockwise: backing up, curving back-right.
  // curveCircleReverse(-90_deg, 24_in) in the simulator ends here.
  checkArc("backward right (+24, -90)", origin, -90_deg, 24_in, 24, -24, true, 90);
  // Centre left, turning clockwise: backing up, curving back-left.
  checkArc("backward left (-24, +90)", origin, 90_deg, -24_in, -24, -24, true, -90);

  std::printf("-- from a pose that isn't the origin\n");
  // Facing +X at (10, 5): centre right is -Y of it.
  checkArc("facing +X, (+12, +90)", Pose2D{10, 5, 90 * kDeg}, 180_deg, 12_in, 22, -7, false, 180);

  std::printf("-- the short way round\n");
  {
    // 270 deg is -90 the short way: a counter-clockwise turn.
    const ArcPlan plan = planArc(origin, 270_deg, -24_in);
    CHECK(!plan.reversed);
    CHECK_NEAR(plan.path.back().x.in(), -24, 1e-9);
  }

  std::printf("-- nothing to plan\n");
  CHECK(!planArc(origin, 90_deg, 0_in).path.valid());
  CHECK(!planArc(origin, 0_deg, 24_in).path.valid());
  CHECK(!planArc(origin, 360_deg, 24_in).path.valid());
  CHECK(!planArc(Pose2D{NAN, 0, 0}, 90_deg, 24_in).path.valid());

  return mclib::test::summary("arc");
}
