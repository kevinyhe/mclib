// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file pose_path_test.cpp
 * @brief planToPose() leaves along the start heading, arrives on the target
 * pose along its heading, and refuses curves the robot can't drive.
 */

#include "mclib/control/profile.hpp"
#include "mclib/math.hpp"
#include "mclib/path/pose_path.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>

using namespace mclib::units::literals;
using mclib::Pose2D;
using mclib::path::Path;
using mclib::path::planToPose;

namespace {

constexpr double kDeg = mclib::kPi / 180.0;

void checkEnds(const char* name, const Path& path, const Pose2D& start, const Pose2D& end,
               double start_travel_deg, double end_travel_deg) {
  std::printf("   %-34s %.2f in long, ends (%.3f, %.3f) travelling %.1f deg\n", name,
              path.length().in(), path.back().x.in(), path.back().y.in(),
              path.back().heading.rad() / kDeg);
  CHECK(path.valid());
  CHECK_NEAR(path.front().x.in(), start.x, 1e-9);
  CHECK_NEAR(path.front().y.in(), start.y, 1e-9);
  CHECK_NEAR(path.back().x.in(), end.x, 1e-9);
  CHECK_NEAR(path.back().y.in(), end.y, 1e-9);
  CHECK_NEAR(mclib::wrapAngle(path.front().heading.rad() - start_travel_deg * kDeg), 0.0, 1e-9);
  CHECK_NEAR(mclib::wrapAngle(path.back().heading.rad() - end_travel_deg * kDeg), 0.0, 1e-9);
}

}  // namespace

int main() {
  const Pose2D origin{0, 0, 0};

  std::printf("-- a right-hand corner, forward\n");
  const Pose2D corner{24, 24, 90 * kDeg};
  const Path right = planToPose(origin, corner, false, 0.39);
  checkEnds("lead 0.39", right, origin, corner, 0, 90);
  // A lead of 0.39 bends the corner into nearly a 24 in quarter circle:
  // 37.7 in long, with curvature near 1/24 in all the way.
  CHECK_NEAR(right.length().in(), 24 * mclib::kPi / 2, 0.2);
  for (const auto& point : right.points()) {
    const double radius_in = 1 / point.curvature.raw() / mclib::units::inch.raw();
    CHECK(radius_in > 20 && radius_in < 29);
  }

  std::printf("-- a left-hand corner curves left\n");
  const Pose2D left_corner{-24, 24, -90 * kDeg};
  const Path left = planToPose(origin, left_corner, false, 0.39);
  checkEnds("lead 0.39", left, origin, left_corner, 0, -90);
  for (const auto& point : left.points()) CHECK(point.curvature.raw() < 0);

  std::printf("-- backing into a corner travels opposite the body heading\n");
  const Pose2D behind{24, -24, -90 * kDeg};
  const Path backward = planToPose(origin, behind, true, 0.39);
  checkEnds("reversed", backward, origin, behind, 180, 90);
  mclib::path::TrajectoryConstraints limits;
  limits.max_velocity = 40 * mclib::units::inps;
  limits.max_acceleration = 60 * mclib::control::inps2;
  limits.reversed = true;
  const auto plan = mclib::path::Trajectory::generate(backward, limits);
  CHECK(!plan.empty());
  CHECK(plan.sample(plan.duration() * 0.5).velocity.raw() < 0);
  CHECK_NEAR(mclib::wrapAngle(plan.states().back().heading.rad() - (-90 * kDeg)), 0.0, 1e-6);

  std::printf("-- straight ahead is a straight line whatever the lead\n");
  const Pose2D ahead{0, 36, 0};
  for (const double lead : {0.1, 0.5, 0.9}) {
    const Path straight = planToPose(origin, ahead, false, lead);
    checkEnds("straight", straight, origin, ahead, 0, 0);
    CHECK_NEAR(straight.length().in(), 36, 1e-6);
  }

  std::printf("-- a larger lead swings wider\n");
  const double tight = planToPose(origin, corner, false, 0.2).length().in();
  const double wide = planToPose(origin, corner, false, 0.8).length().in();
  std::printf("   lead 0.2: %.2f in, lead 0.8: %.2f in\n", tight, wide);
  CHECK(wide > right.length().in() && right.length().in() > tight);

  std::printf("-- curves that fold back, and bad input, give no path\n");
  CHECK(!planToPose(origin, {0, -24, 0}, false, 0.5).valid());
  CHECK(!planToPose(origin, {0, 24, 180 * kDeg}, false, 0.5).valid());
  // Straight ahead, arms longer than the distance send the middle backward.
  CHECK(!planToPose(origin, ahead, false, 1.2).valid());
  CHECK(!planToPose(origin, origin, false, 0.5).valid());
  CHECK(!planToPose(origin, corner, false, 0).valid());
  CHECK(!planToPose(origin, corner, false, NAN).valid());
  CHECK(!planToPose(origin, {NAN, 24, 0}, false, 0.5).valid());

  return mclib::test::summary("pose_path");
}
