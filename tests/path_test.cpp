// mclib
/**
 * @file path_test.cpp
 * @brief Host tests for mclib/path: Path, generateSpline() and PurePursuit.
 *
 * Geometry compiles fine while it is mirrored, so every assertion here is a
 * number. The first block is the frame check: a straight path along +Y with
 * the robot at the origin at heading 0 must produce a goal point dead ahead
 * and exactly zero curvature. If the compass/standard frame handling is
 * transposed that comes out as a hard turn.
 */

#include "mclib/math.hpp"
#include "mclib/path/path.hpp"
#include "mclib/path/pure_pursuit.hpp"
#include "mclib/path/spline.hpp"
#include "mclib/units/units.hpp"
#include "mclib/utils.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using mclib::Pose2D;
using mclib::Vec2;
using mclib::path::Path;
using mclib::path::PathPoint;
using mclib::path::PurePursuit;
using mclib::path::PurePursuitConfig;
using mclib::path::PurePursuitOutput;
using mclib::path::SplineConfig;
using mclib::path::Waypoint;
using mclib::path::approachSpeedLimit;
using mclib::path::curvatureSpeedLimit;
using mclib::path::generateSpline;
using mclib::path::wheelSpeeds;
using mclib::units::QAcceleration;
using mclib::units::QCurvature;
using mclib::units::QLength;
using mclib::units::QVelocity;
using mclib::units::inch;
using mclib::units::inps;
using mclib::units::second;

namespace {

/// @brief A waypoint from plain inch numbers.
Waypoint wp(double x, double y) { return Waypoint{inch * x, inch * y}; }

/// @brief Curvature in 1/inch, the unit a human can sanity-check.
double perInch(QCurvature curvature) { return curvature.raw() * 0.0254; }

// ---------------------------------------------------------------------------
// 1. The frame check.
// ---------------------------------------------------------------------------
void frameCheck() {
  std::printf("-- frame check\n");
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 10.0)});
  CHECK_NEAR(path.length().in(), 10.0, 1e-9);
  // Heading along +Y is compass 0.
  CHECK_NEAR(path.front().heading.deg(), 0.0, 1e-9);

  PurePursuitConfig config;
  config.lookahead = 5.0 * inch;
  PurePursuit follower(path, config);

  const PurePursuitOutput out = follower.update(Pose2D{0.0, 0.0, 0.0});
  std::printf("   goal=(%.6f, %.6f) curvature=%.9f /in xtrack=%.9f in\n",
              out.lookahead_point.x(), out.lookahead_point.y(),
              perInch(out.curvature), out.cross_track_error.in());

  // Straight ahead: x is zero, y is exactly the lookahead.
  CHECK_NEAR(out.lookahead_point.x(), 0.0, 1e-12);
  CHECK_NEAR(out.lookahead_point.y(), 5.0, 1e-12);
  // And the steering command is exactly straight.
  CHECK_EQ(perInch(out.curvature), 0.0);
  CHECK_NEAR(out.cross_track_error.in(), 0.0, 1e-12);
  CHECK(!out.off_path);
  CHECK(!out.at_end);
  CHECK(!out.finished);
  // Both wheels at the same speed on a straight line.
  CHECK_NEAR(out.wheels.left.inps(), out.wheels.right.inps(), 1e-12);

  // The same path driven along +X: the robot must be told to turn right 90 deg
  // worth, not left. Robot at origin heading 0 (+Y), path running to +X.
  Path east = Path::fromWaypoints({wp(0.0, 0.0), wp(10.0, 0.0)});
  PurePursuit east_follower(east, config);
  const PurePursuitOutput east_out = east_follower.update(Pose2D{0.0, 0.0, 0.0});
  std::printf("   east goal=(%.6f, %.6f) curvature=%+.6f /in\n",
              east_out.lookahead_point.x(), east_out.lookahead_point.y(),
              perInch(east_out.curvature));
  CHECK_NEAR(east_out.lookahead_point.x(), 5.0, 1e-12);
  // Goal 5 in to the right, dead abeam: k = 2*x/|p|^2 = 2*5/25 = 0.4 /in,
  // positive because it is to the right. The default max_curvature is 1/6 /in,
  // which is tighter, so the clamp is what actually comes out - assert the
  // clamped value, and the raw one with the clamp off.
  CHECK_NEAR(perInch(east_out.curvature), 1.0 / 6.0, 1e-9);
  PurePursuitConfig unclamped = config;
  unclamped.max_curvature = QCurvature{};
  PurePursuit raw_follower(east, unclamped);
  const PurePursuitOutput raw_out = raw_follower.update(Pose2D{0.0, 0.0, 0.0});
  std::printf("   east unclamped curvature=%+.6f /in\n", perInch(raw_out.curvature));
  CHECK_NEAR(perInch(raw_out.curvature), 0.4, 1e-9);
}

// ---------------------------------------------------------------------------
// 1b. A goal behind the robot must produce a turn, not zero curvature.
// ---------------------------------------------------------------------------
void goalBehind() {
  std::printf("-- goal behind the robot\n");
  // arcRadius() answers +infinity for a target straight ahead AND for one
  // straight behind. Believing the second means commanding zero curvature at
  // full speed away from the path, and the forward-only cursor never recovers.
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 60.0)});
  PurePursuitConfig config;
  config.lookahead = 10.0 * inch;
  config.max_curvature = 1.0 / (6.0 * inch);

  for (double heading_deg : {170.0, 179.0, 180.0, -179.0, -170.0}) {
    PurePursuit follower(path, config);
    const PurePursuitOutput out =
        follower.update(Pose2D{0.0, 10.0, heading_deg * mclib::kPi / 180.0});
    std::printf("   heading %+7.1f deg -> curvature=%+.6f /in v=%.2f L=%.2f R=%.2f\n",
                heading_deg, perInch(out.curvature), out.velocity.inps(),
                out.wheels.left.inps(), out.wheels.right.inps());
    // Never straight: the goal is behind, so the command must be a hard turn.
    CHECK(std::fabs(perInch(out.curvature)) > 0.1);
    CHECK(out.wheels.left.inps() != out.wheels.right.inps());
  }

  // Exactly reversed: the clamp magnitude, and a stable sign rather than a
  // zero or a NaN.
  PurePursuit follower(path, config);
  const PurePursuitOutput reversed = follower.update(Pose2D{0.0, 10.0, mclib::kPi});
  CHECK_NEAR(std::fabs(perInch(reversed.curvature)), 1.0 / 6.0, 1e-9);

  // With the clamp disabled it still turns: the fallback is the same radius,
  // not zero.
  PurePursuitConfig no_clamp = config;
  no_clamp.max_curvature = QCurvature{};
  PurePursuit unclamped_follower(path, no_clamp);
  const PurePursuitOutput out = unclamped_follower.update(Pose2D{0.0, 10.0, mclib::kPi});
  std::printf("   clamp disabled, reversed -> curvature=%+.6f /in\n",
              perInch(out.curvature));
  CHECK_NEAR(std::fabs(perInch(out.curvature)), 1.0 / 6.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 1c. A robot pushed backwards must not fake an end of path.
// ---------------------------------------------------------------------------
void pushedBackwards() {
  std::printf("-- pushed backwards\n");
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 60.0)});
  PurePursuitConfig config;
  config.lookahead = 10.0 * inch;
  PurePursuit follower(path, config);

  const PurePursuitOutput before = follower.update(Pose2D{0.0, 30.0, 0.0});
  std::printf("   y=30 -> goal_y=%.4f at_end=%d\n", before.lookahead_point.y(),
              static_cast<int>(before.at_end));
  CHECK_NEAR(before.lookahead_point.y(), 40.0, 1e-9);
  CHECK(!before.at_end);

  // Shoved back two inches: a bump, a slip, an odometry correction. The
  // lookahead cursor is now ahead of the robot's own circle, and a single-pass
  // search would report at_end with 30 in of path still to drive.
  const PurePursuitOutput after = follower.update(Pose2D{0.0, 28.0, 0.0});
  std::printf("   y=28 -> goal_y=%.4f at_end=%d remaining=%.4f\n",
              after.lookahead_point.y(), static_cast<int>(after.at_end),
              after.remaining.in());
  CHECK(!after.at_end);
  CHECK_NEAR(after.lookahead_point.y(), 38.0, 1e-9);
  CHECK(!after.finished);
}

// ---------------------------------------------------------------------------
// 1d. search_window bounds the cursor even on a single long segment.
// ---------------------------------------------------------------------------
void searchWindowBound() {
  std::printf("-- search window on a long segment\n");
  // One 200 in segment: the per-sample scan has nothing to break on, so the
  // bound has to come from the parameter inside the segment.
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 200.0)});
  PurePursuitConfig config;
  config.lookahead = 10.0 * inch;
  config.search_window = 24.0 * inch;
  PurePursuit follower(path, config);

  follower.update(Pose2D{0.0, 0.0, 0.0});
  const PurePursuitOutput spike = follower.update(Pose2D{0.0, 150.0, 0.0});
  std::printf("   one bad pose at y=150 -> distance_along=%.4f (window 24)\n",
              spike.distance_along.in());
  CHECK_NEAR(spike.distance_along.in(), 24.0, 1e-9);

  // Back to the truth: the cursor is 24 in ahead, not 150, so the robot
  // recovers instead of abandoning the route.
  const PurePursuitOutput recovered = follower.update(Pose2D{0.0, 1.0, 0.0});
  std::printf("   back to y=1 -> distance_along=%.4f off_path=%d goal_y=%.4f\n",
              recovered.distance_along.in(), static_cast<int>(recovered.off_path),
              recovered.lookahead_point.y());
  // The cursor is monotone, so it stays at 24 rather than following the robot
  // back to 1 - and 24 is what search_window promised. Unbounded it would be
  // 150, and the goal 160 in up a path the robot has not driven. The robot is
  // 23 in from its own projection, so off_path is set and the goal is the
  // rejoin point one lookahead past the cursor.
  CHECK_NEAR(recovered.distance_along.in(), 24.0, 1e-9);
  CHECK(recovered.off_path);
  CHECK_NEAR(recovered.lookahead_point.y(), 34.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 1e. Cross-track error is signed against the segment, not a vertex heading.
// ---------------------------------------------------------------------------
void crossTrackOnACorner() {
  std::printf("-- cross-track error at a corner\n");
  // Up to (0,10), then a right angle due east to (10,10). A polyline's heading
  // is per-segment, so an interpolated vertex heading would scale the reported
  // error by cos(the ramp) - and flip its sign on a corner over 90 degrees.
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 10.0), wp(10.0, 10.0)});
  PurePursuitConfig config;
  config.lookahead = 4.0 * inch;

  // Robot 1 in north of the eastbound leg, i.e. 1 in to its LEFT.
  PurePursuit follower(path, config);
  follower.update(Pose2D{0.0, 0.0, 0.0});
  const PurePursuitOutput north = follower.update(Pose2D{5.0, 11.0, mclib::kPi / 2.0});
  std::printf("   robot (5, 11) on an eastbound leg: xtrack=%+.6f in\n",
              north.cross_track_error.in());
  CHECK_NEAR(north.cross_track_error.in(), -1.0, 1e-9);

  // Mirror: 1 in south of it, i.e. 1 in to its right.
  PurePursuit other(path, config);
  other.update(Pose2D{0.0, 0.0, 0.0});
  const PurePursuitOutput south = other.update(Pose2D{5.0, 9.0, mclib::kPi / 2.0});
  std::printf("   robot (5,  9) on an eastbound leg: xtrack=%+.6f in\n",
              south.cross_track_error.in());
  CHECK_NEAR(south.cross_track_error.in(), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 2. Lookahead lands exactly at the lookahead distance.
// ---------------------------------------------------------------------------
void lookaheadDistance() {
  std::printf("-- lookahead distance\n");
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 60.0)});
  PurePursuitConfig config;
  config.lookahead = 12.0 * inch;
  PurePursuit follower(path, config);

  for (double y : {0.0, 5.0, 17.5, 30.0}) {
    const PurePursuitOutput out = follower.update(Pose2D{0.0, y, 0.0});
    const double d = (out.lookahead_point - Vec2{0.0, y}).norm();
    std::printf("   y=%.1f -> goal y=%.6f, |goal-robot|=%.9f\n", y,
                out.lookahead_point.y(), d);
    CHECK_NEAR(d, 12.0, 1e-9);
    CHECK_NEAR(out.lookahead_point.y(), y + 12.0, 1e-9);
  }
}

// ---------------------------------------------------------------------------
// 3. Curvature on a circular arc of known radius.
// ---------------------------------------------------------------------------
void arcCurvature() {
  std::printf("-- arc curvature\n");
  // A clockwise quarter circle of radius 24 in, centred at (24, 0), starting
  // at (0, 0) heading +Y. Clockwise is positive curvature.
  const double radius = 24.0;
  std::vector<Waypoint> waypoints;
  for (int i = 0; i <= 24; ++i) {
    const double phi = (mclib::kPi / 2.0) * (static_cast<double>(i) / 24.0);
    waypoints.push_back(wp(radius - radius * std::cos(phi), radius * std::sin(phi)));
  }
  Path arc = generateSpline(waypoints, SplineConfig{});
  CHECK(arc.valid());
  std::printf("   samples=%zu length=%.4f in (exact %.4f)\n", arc.size(),
              arc.length().in(), radius * mclib::kPi / 2.0);
  CHECK_NEAR(arc.length().in(), radius * mclib::kPi / 2.0, 0.05);

  // Sample the middle of the path, away from the reflected end knots.
  double worst = 0.0;
  for (double t = 0.15; t <= 0.85; t += 0.05) {
    const PathPoint point = arc.atParameter(t);
    worst = std::fmax(worst, std::fabs(perInch(point.curvature) - 1.0 / radius));
  }
  std::printf("   worst |k - 1/R| over t in [0.15, 0.85] = %.6f /in (1/R = %.6f)\n",
              worst, 1.0 / radius);
  CHECK(worst < 2.0e-3);
  // Sign: clockwise means positive.
  CHECK(perInch(arc.atParameter(0.5).curvature) > 0.0);

  // The same arc as a polyline, checked against mclib::arcRadius() through the
  // follower: a robot on the arc facing along it gets ~1/R back.
  PurePursuitConfig config;
  config.lookahead = 6.0 * inch;
  PurePursuit follower(arc, config);
  const PathPoint start = arc.atParameter(0.2);
  const PurePursuitOutput out =
      follower.update(Pose2D{start.x.in(), start.y.in(), start.heading.rad()});
  std::printf("   follower curvature on arc = %.6f /in (1/R = %.6f)\n",
              perInch(out.curvature), 1.0 / radius);
  CHECK_NEAR(perInch(out.curvature), 1.0 / radius, 5.0e-3);
}

// ---------------------------------------------------------------------------
// 4. Lateral offset steers back toward the path, with the right sign.
// ---------------------------------------------------------------------------
void lateralOffsetSign() {
  std::printf("-- lateral offset sign\n");
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 60.0)});
  PurePursuitConfig config;
  config.lookahead = 12.0 * inch;

  // Robot 6 in to the RIGHT of the path (+X), heading 0 (+Y). It must be told
  // to steer LEFT: negative curvature.
  PurePursuit right(path, config);
  const PurePursuitOutput right_out = right.update(Pose2D{6.0, 10.0, 0.0});
  std::printf("   robot +6 in X: xtrack=%+.4f in curvature=%+.6f /in goal=(%.4f, %.4f)\n",
              right_out.cross_track_error.in(), perInch(right_out.curvature),
              right_out.lookahead_point.x(), right_out.lookahead_point.y());
  CHECK_NEAR(right_out.cross_track_error.in(), 6.0, 1e-9);
  CHECK(perInch(right_out.curvature) < 0.0);
  CHECK(right_out.wheels.right.inps() > right_out.wheels.left.inps());

  // Mirror image: 6 in to the LEFT, must steer RIGHT.
  PurePursuit left(path, config);
  const PurePursuitOutput left_out = left.update(Pose2D{-6.0, 10.0, 0.0});
  std::printf("   robot -6 in X: xtrack=%+.4f in curvature=%+.6f /in\n",
              left_out.cross_track_error.in(), perInch(left_out.curvature));
  CHECK_NEAR(left_out.cross_track_error.in(), -6.0, 1e-9);
  CHECK(perInch(left_out.curvature) > 0.0);
  CHECK(left_out.wheels.left.inps() > left_out.wheels.right.inps());

  // Antisymmetric to the last bit.
  CHECK_NEAR(perInch(left_out.curvature), -perInch(right_out.curvature), 1e-12);
}

// ---------------------------------------------------------------------------
// 5. A path that doubles back does not drag the cursor backwards.
// ---------------------------------------------------------------------------
void doubleBack() {
  std::printf("-- double back\n");
  // Out along +Y to 60, hairpin right, back down along x = 4 to y = 0.
  std::vector<Waypoint> waypoints{wp(0.0, 0.0),  wp(0.0, 60.0),
                                  wp(2.0, 62.0), wp(4.0, 60.0),
                                  wp(4.0, 0.0)};
  Path path = Path::fromWaypoints(waypoints);
  PurePursuitConfig config;
  config.lookahead = 10.0 * inch;
  config.search_window = 24.0 * inch;
  PurePursuit follower(path, config);

  // Drive up the outbound leg. The return leg is only 4 in away in field
  // space, so an unguarded nearest-point search would latch onto it.
  double previous = -1.0;
  bool monotone = true;
  for (double y = 0.0; y <= 58.0; y += 2.0) {
    const PurePursuitOutput out = follower.update(Pose2D{0.0, y, 0.0});
    if (out.distance_along.in() < previous - 1e-9) {
      monotone = false;
      std::printf("   BACKWARDS at y=%.1f: %.4f -> %.4f\n", y, previous,
                  out.distance_along.in());
    }
    previous = out.distance_along.in();
    // Arc length along the outbound leg is just y until the hairpin.
    CHECK_NEAR(out.distance_along.in(), y, 1e-6);
    // Only while the hairpin is further away than the lookahead: past that the
    // goal legitimately sits on the return leg, which is *behind* in Y.
    if (y <= 45.0) {
      CHECK(out.lookahead_point.y() > y);
    }
  }
  std::printf("   progress monotone = %s, final distance_along = %.4f in of %.4f\n",
              monotone ? "yes" : "NO", previous, path.length().in());
  CHECK(monotone);

  // And the goal point is never behind on the return leg either.
  double last = previous;
  for (double y = 58.0; y >= 2.0; y -= 2.0) {
    const PurePursuitOutput out = follower.update(Pose2D{4.0, y, mclib::kPi});
    CHECK(out.distance_along.in() >= last - 1e-9);
    last = out.distance_along.in();
  }
  std::printf("   after the hairpin distance_along = %.4f in of %.4f\n", last,
              path.length().in());
  CHECK(last > 60.0);
}

// ---------------------------------------------------------------------------
// 6. End of path and off path.
// ---------------------------------------------------------------------------
void endAndOffPath() {
  std::printf("-- end of path / off path\n");
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 24.0)});
  PurePursuitConfig config;
  config.lookahead = 10.0 * inch;
  config.finish_tolerance = 2.0 * inch;

  // Near the end: the circle hangs off the end, so the goal is the endpoint.
  PurePursuit follower(path, config);
  const PurePursuitOutput near_end = follower.update(Pose2D{0.0, 20.0, 0.0});
  std::printf("   at y=20: at_end=%d goal=(%.4f, %.4f) remaining=%.4f v=%.4f in/s\n",
              static_cast<int>(near_end.at_end), near_end.lookahead_point.x(),
              near_end.lookahead_point.y(), near_end.remaining.in(),
              near_end.velocity.inps());
  CHECK(near_end.at_end);
  CHECK_NEAR(near_end.lookahead_point.y(), 24.0, 1e-12);
  CHECK(!near_end.finished);
  // sqrt(2 * 40 in/s^2 * 4 in) = 17.888 in/s, under the 48 in/s cap.
  CHECK_NEAR(near_end.velocity.inps(), std::sqrt(2.0 * 40.0 * 4.0), 1e-6);

  // Inside the tolerance: finished, zero speed.
  const PurePursuitOutput done = follower.update(Pose2D{0.0, 23.0, 0.0});
  std::printf("   at y=23: finished=%d remaining=%.4f v=%.6f\n",
              static_cast<int>(done.finished), done.remaining.in(), done.velocity.inps());
  CHECK(done.finished);
  CHECK_EQ(done.velocity.inps(), 0.0);
  CHECK_EQ(done.wheels.left.inps(), 0.0);
  CHECK_EQ(done.wheels.right.inps(), 0.0);

  // Off path: 30 in to the right of a path that is only 10 in of lookahead
  // wide. No intersection exists; the goal is one lookahead past the closest
  // point, and off_path says so.
  PurePursuit stray(path, config);
  const PurePursuitOutput off = stray.update(Pose2D{30.0, 4.0, 0.0});
  std::printf("   off path: off_path=%d error=%.4f goal=(%.4f, %.4f) curvature=%+.6f\n",
              static_cast<int>(off.off_path), off.path_error.in(),
              off.lookahead_point.x(), off.lookahead_point.y(), perInch(off.curvature));
  CHECK(off.off_path);
  CHECK_NEAR(off.path_error.in(), 30.0, 1e-9);
  CHECK_NEAR(off.lookahead_point.x(), 0.0, 1e-9);
  CHECK_NEAR(off.lookahead_point.y(), 14.0, 1e-9);
  // Robot is right of the path, so it must be steered left.
  CHECK(perInch(off.curvature) < 0.0);
  CHECK(!off.finished);
}

// ---------------------------------------------------------------------------
// 7. Spline continuity across a segment join.
// ---------------------------------------------------------------------------
void splineContinuity() {
  std::printf("-- spline continuity\n");
  std::vector<Waypoint> waypoints{wp(0.0, 0.0), wp(12.0, 24.0), wp(36.0, 30.0),
                                  wp(48.0, 6.0)};
  SplineConfig config;
  config.spacing = 0.25 * inch;
  Path path = generateSpline(waypoints, config);
  CHECK(path.valid());
  std::printf("   samples=%zu length=%.4f in\n", path.size(), path.length().in());

  // The spline interpolates: every waypoint is on the curve.
  for (const Waypoint& waypoint : waypoints) {
    double best = 1e9;
    for (const PathPoint& point : path.points()) {
      best = std::fmin(best, (point.point() - waypoint.point()).norm());
    }
    CHECK_NEAR(best, 0.0, 1e-6);
  }

  // Sample densely and assert step-to-step position and heading are continuous.
  double worst_gap = 0.0;
  double worst_turn = 0.0;
  const int steps = 4000;
  PathPoint previous = path.atParameter(0.0);
  for (int i = 1; i <= steps; ++i) {
    const PathPoint current = path.atParameter(static_cast<double>(i) / steps);
    worst_gap = std::fmax(worst_gap, (current.point() - previous.point()).norm());
    worst_turn = std::fmax(worst_turn,
                           std::fabs(mclib::wrapAngle(current.heading.rad() -
                                                      previous.heading.rad())));
    previous = current;
  }
  const double step_in = path.length().in() / steps;
  std::printf("   step=%.5f in: worst position gap=%.6f in, worst heading step=%.4f deg\n",
              step_in, worst_gap, worst_turn * 180.0 / mclib::kPi);
  CHECK(worst_gap < step_in * 1.05 + 1e-9);
  // A C1 curve turns by at most (max curvature * step) per step. 2 deg over a
  // 0.02 in step would be a 0.6 in radius - i.e. a corner, not a curve.
  CHECK(worst_turn * 180.0 / mclib::kPi < 2.0);

  // The same, checked right at the interior knots rather than on average.
  for (std::size_t k = 1; k + 1 < waypoints.size(); ++k) {
    // Find the sample nearest this waypoint and compare its neighbours.
    std::size_t best_index = 0;
    double best = 1e9;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double d = (path[i].point() - waypoints[k].point()).norm();
      if (d < best) {
        best = d;
        best_index = i;
      }
    }
    CHECK(best_index >= 1 && best_index + 1 < path.size());
    if (best_index < 1 || best_index + 1 >= path.size()) {
      continue;
    }
    const PathPoint& before = path.at(best_index - 1);
    const PathPoint& after = path.at(best_index + 1);
    const double jump = std::fabs(mclib::wrapAngle(after.heading.rad() - before.heading.rad()));
    // A C1 join turns by curvature * arc length across it and no more. Anything
    // above that is a kink, i.e. a heading discontinuity.
    const double span = (after.distance - before.distance).raw() / 0.0254;
    double local = 0.0;
    for (std::size_t i = best_index - 1; i <= best_index + 1; ++i) {
      local = std::fmax(local, std::fabs(perInch(path.at(i).curvature)));
    }
    const double allowed = local * span * 1.2 + 1e-4;
    std::printf("   knot %zu at (%.1f, %.1f): %.5f rad across %.4f in, curvature allows %.5f\n",
                k, waypoints[k].x.in(), waypoints[k].y.in(), jump, span, allowed);
    CHECK(jump <= allowed);
  }

  // tension = 1 zeroes the tangents, so the Hermite derivative vanishes at
  // every knot. Those samples must fall back to the chord bearing, not report
  // compass 0 - a real direction, pointing along +Y, and wrong.
  SplineConfig taut;
  taut.spacing = 1.0 * inch;
  taut.tension = 1.0;
  Path pulled = generateSpline({wp(0.0, 0.0), wp(0.0, 20.0), wp(20.0, 20.0)}, taut);
  std::printf("   tension=1: first heading %.4f deg, last heading %.4f deg\n",
              pulled.front().heading.deg(), pulled.back().heading.deg());
  CHECK_NEAR(pulled.front().heading.deg(), 0.0, 1e-9);
  CHECK_NEAR(pulled.back().heading.deg(), 90.0, 1e-9);

  // Curvature is finite everywhere: no cusp from the centripetal parameterisation.
  double worst_curvature = 0.0;
  for (const PathPoint& point : path.points()) {
    worst_curvature = std::fmax(worst_curvature, std::fabs(perInch(point.curvature)));
  }
  std::printf("   max |curvature| = %.6f /in (radius %.3f in)\n", worst_curvature,
              1.0 / worst_curvature);
  CHECK(std::isfinite(worst_curvature));
  CHECK(worst_curvature < 1.0);
}

// ---------------------------------------------------------------------------
// 8. Curvature velocity limiting.
// ---------------------------------------------------------------------------
void velocityLimiting() {
  std::printf("-- curvature velocity limiting\n");
  const QVelocity max_v = 48.0 * inps;
  const QAcceleration a_lat = 60.0 * inps / second;

  // Straight: no limit at all.
  CHECK_NEAR(curvatureSpeedLimit(QCurvature{}, max_v, a_lat).inps(), 48.0, 1e-9);

  // A non-positive budget means "no limit" in both directions, so switching the
  // endpoint ramp off does not pin the robot at min_velocity for the whole path.
  CHECK(!std::isfinite(approachSpeedLimit(10.0 * inch, QAcceleration{}).inps()));
  Path straight = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 60.0)});
  PurePursuitConfig no_ramp;
  no_ramp.lookahead = 10.0 * inch;
  no_ramp.max_velocity = 48.0 * inps;
  no_ramp.min_velocity = 6.0 * inps;
  no_ramp.max_decel = QAcceleration{};
  PurePursuit unramped(straight, no_ramp);
  const PurePursuitOutput flat = unramped.update(Pose2D{0.0, 5.0, 0.0});
  std::printf("   max_decel = 0 -> %.4f in/s, not the %.1f in/s floor\n",
              flat.velocity.inps(), no_ramp.min_velocity.inps());
  CHECK_NEAR(flat.velocity.inps(), 48.0, 1e-9);

  // Gentle: R = 48 in -> k = 1/48 /in -> sqrt(60 * 48) = 53.7 in/s, capped at 48.
  const QVelocity gentle = curvatureSpeedLimit(1.0 / (48.0 * inch), max_v, a_lat);
  // Tight: R = 8 in -> sqrt(60 * 8) = 21.909 in/s.
  const QVelocity tight = curvatureSpeedLimit(1.0 / (8.0 * inch), max_v, a_lat);
  // Tighter: R = 2 in -> sqrt(60 * 2) = 10.954 in/s.
  const QVelocity tighter = curvatureSpeedLimit(1.0 / (2.0 * inch), max_v, a_lat);
  std::printf("   R=48in -> %.4f in/s, R=8in -> %.4f in/s, R=2in -> %.4f in/s\n",
              gentle.inps(), tight.inps(), tighter.inps());
  CHECK_NEAR(gentle.inps(), 48.0, 1e-9);
  CHECK_NEAR(tight.inps(), std::sqrt(60.0 * 8.0), 1e-9);
  CHECK_NEAR(tighter.inps(), std::sqrt(60.0 * 2.0), 1e-9);
  CHECK(tight < gentle);
  CHECK(tighter < tight);

  // Through the follower: same start pose, two arcs of different radius.
  auto arcPath = [](double radius) {
    std::vector<Waypoint> waypoints;
    for (int i = 0; i <= 40; ++i) {
      const double phi = (mclib::kPi / 2.0) * (static_cast<double>(i) / 40.0);
      waypoints.push_back(wp(radius - radius * std::cos(phi), radius * std::sin(phi)));
    }
    SplineConfig spline;
    spline.spacing = 0.5 * inch;
    return generateSpline(waypoints, spline);
  };

  PurePursuitConfig config;
  config.lookahead = 6.0 * inch;
  config.max_velocity = 48.0 * inps;
  config.min_velocity = 2.0 * inps;
  config.track_width = 12.0 * inch;

  PurePursuit gentle_follower(arcPath(48.0), config);
  PurePursuit tight_follower(arcPath(10.0), config);
  const PurePursuitOutput gentle_out = gentle_follower.update(Pose2D{0.0, 0.0, 0.0});
  const PurePursuitOutput tight_out = tight_follower.update(Pose2D{0.0, 0.0, 0.0});
  std::printf("   follower on R=48in arc: %.4f in/s; on R=10in arc: %.4f in/s\n",
              gentle_out.velocity.inps(), tight_out.velocity.inps());
  CHECK(tight_out.velocity < gentle_out.velocity);
  // Gentle arc: the curvature limit is sqrt(60 * 48) = 53.7 in/s, so the 48
  // in/s cap wins - and then the outer wheel on a 1/48 /in arc with a 12 in
  // track would want 48 * 1.125 = 54 in/s, so everything scales by 1/1.125.
  CHECK_NEAR(gentle_out.velocity.inps(), 48.0 / 1.125, 0.5);
  CHECK_NEAR(gentle_out.wheels.left.inps(), 48.0, 0.5);
  // Tight arc: sqrt(60 * 10) = 24.49 in/s, and the outer wheel only wants
  // 24.49 * 1.6 = 39.2 in/s, under the cap, so nothing is scaled.
  CHECK_NEAR(tight_out.velocity.inps(), std::sqrt(60.0 * 10.0), 1.0);

  // Wheel split: 24 in/s on a 24 in radius right turn with a 12 in track is
  // 18 / 30 in/s. Positive curvature turns right, so the right wheel is slower.
  const auto split = wheelSpeeds(24.0 * inps, 1.0 / (24.0 * inch), 12.0 * inch);
  std::printf("   wheelSpeeds(24 in/s, R=24in right, track 12in) = L %.4f / R %.4f\n",
              split.left.inps(), split.right.inps());
  CHECK_NEAR(split.left.inps(), 30.0, 1e-9);
  CHECK_NEAR(split.right.inps(), 18.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 9. Path queries.
// ---------------------------------------------------------------------------
void pathQueries() {
  std::printf("-- path queries\n");
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 10.0), wp(10.0, 10.0)});
  CHECK_NEAR(path.length().in(), 20.0, 1e-9);
  CHECK_NEAR(path.atDistance(5.0 * inch).y.in(), 5.0, 1e-9);
  CHECK_NEAR(path.atDistance(15.0 * inch).x.in(), 5.0, 1e-9);
  CHECK_NEAR(path.atParameter(0.5).y.in(), 10.0, 1e-9);
  // Clamped, not extrapolated.
  CHECK_NEAR(path.atDistance(-5.0 * inch).y.in(), 0.0, 1e-9);
  CHECK_NEAR(path.atDistance(500.0 * inch).x.in(), 10.0, 1e-9);
  // Heading is the segment *leaving* each vertex; the last one carries the
  // segment that arrived. The far end runs due east, i.e. compass 90 deg.
  CHECK_NEAR(path.front().heading.deg(), 0.0, 1e-9);
  CHECK_NEAR(path.at(1).heading.deg(), 90.0, 1e-9);
  CHECK_NEAR(path.back().heading.deg(), 90.0, 1e-9);

  // A repeated waypoint is dropped rather than recorded with headingToward()'s
  // zero, which would be a sample on an eastbound path claiming to face +Y.
  Path duped = Path::fromWaypoints({wp(0.0, 0.0), wp(10.0, 0.0), wp(10.0, 0.0),
                                    wp(20.0, 0.0)});
  CHECK_EQ(static_cast<double>(duped.size()), 3.0);
  for (const PathPoint& point : duped.points()) {
    CHECK_NEAR(point.heading.deg(), 90.0, 1e-9);
  }

  // An empty or single-point path is unfollowable and says so instead of
  // crashing.
  Path empty;
  CHECK(!empty.valid());
  CHECK_NEAR(empty.length().in(), 0.0, 1e-12);
  PurePursuit nowhere(empty);
  const PurePursuitOutput out = nowhere.update(Pose2D{1.0, 2.0, 0.0});
  CHECK(out.finished);
  CHECK_EQ(out.velocity.inps(), 0.0);
}

// ---------------------------------------------------------------------------
// 14. Overshooting the end of the path is a stop, not a lap.
// ---------------------------------------------------------------------------
void pastTheEnd() {
  std::printf("-- past the end of the path\n");
  // A 24 in path along +Y, the robot 16 in past its end, still facing +Y. 16 in
  // is more than the 12 in lookahead, so off_path is true and the projection is
  // pinned at the end of the path.
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 24.0)});
  PurePursuitConfig config;  // Defaults: 12 in lookahead, 2 in finish tolerance.
  PurePursuit follower(path, config);

  const PurePursuitOutput first = follower.update(Pose2D{0.0, 40.0, 0.0});
  std::printf("   at (0, 40): off_path=%d past_end=%d remaining=%.4f finished=%d v=%.4f\n",
              static_cast<int>(first.off_path), static_cast<int>(first.past_end),
              first.remaining.in(), static_cast<int>(first.finished),
              first.velocity.inps());
  CHECK(first.off_path);
  CHECK(first.past_end);
  CHECK_NEAR(first.remaining.in(), 0.0, 1e-9);
  CHECK(first.finished);
  CHECK_EQ(first.velocity.inps(), 0.0);
  CHECK_EQ(perInch(first.curvature), 0.0);
  CHECK_EQ(first.wheels.left.inps(), 0.0);
  CHECK_EQ(first.wheels.right.inps(), 0.0);

  // Drive the closed loop the way a chassis task would and count the ticks.
  // Before the fix this ran 627 ticks - 6.3 s of a 15 s autonomous - and
  // wandered out to (14.8, 30.5) on the way. It has to be tick 0 now.
  PurePursuit loop(path, config);
  Pose2D pose{0.0, 40.0, 0.0};
  const double dt = 0.010;
  int ticks = 0;
  double worst_excursion = 0.0;
  for (; ticks < 2000; ++ticks) {
    const PurePursuitOutput out = loop.update(pose);
    if (out.finished) {
      break;
    }
    // Unicycle in the compass frame: theta = 0 is +Y, clockwise positive.
    pose.theta = mclib::wrapAngle(pose.theta + out.turn_rate.raw() * dt);
    pose.x += out.velocity.inps() * std::sin(pose.theta) * dt;
    pose.y += out.velocity.inps() * std::cos(pose.theta) * dt;
    worst_excursion =
        std::fmax(worst_excursion, (Vec2{pose.x, pose.y} - Vec2{0.0, 40.0}).norm());
  }
  std::printf("   closed loop: finished after %d ticks (%.2f s), final (%.4f, %.4f), "
              "worst excursion %.4f in\n",
              ticks, ticks * dt, pose.x, pose.y, worst_excursion);
  CHECK_EQ(static_cast<double>(ticks), 0.0);
  CHECK_NEAR(worst_excursion, 0.0, 1e-12);

  // Being off to the side but not past the end is still not a finish: there is
  // path left to rejoin.
  PurePursuit beside(path, config);
  const PurePursuitOutput abeam = beside.update(Pose2D{30.0, 4.0, 0.0});
  std::printf("   at (30, 4): off_path=%d past_end=%d finished=%d\n",
              static_cast<int>(abeam.off_path), static_cast<int>(abeam.past_end),
              static_cast<int>(abeam.finished));
  CHECK(abeam.off_path);
  CHECK(!abeam.past_end);
  CHECK(!abeam.finished);

  // Nor is being past the end of an *earlier* leg: past_end reads the final leg
  // only, and there is still a whole second leg to drive.
  Path corner = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 24.0), wp(24.0, 24.0)});
  PurePursuit turning(corner, config);
  const PurePursuitOutput mid = turning.update(Pose2D{0.0, 40.0, 0.0});
  std::printf("   past leg 1 of 2: past_end=%d remaining=%.4f finished=%d\n",
              static_cast<int>(mid.past_end), mid.remaining.in(),
              static_cast<int>(mid.finished));
  CHECK(!mid.past_end);
  CHECK(!mid.finished);

  // past_end is a half-plane test on the *final* leg, so on a path whose last
  // leg heads back toward the start the plain geometry is satisfied from the
  // very first tick. The remaining-distance gate is what keeps the flag honest.
  Path loops = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 24.0), wp(10.0, 24.0),
                                    wp(10.0, 0.0)});
  PurePursuit lap(loops, config);
  const PurePursuitOutput start = lap.update(Pose2D{0.0, 0.0, 0.0});
  std::printf("   start of a path that doubles back: past_end=%d remaining=%.4f\n",
              static_cast<int>(start.past_end), start.remaining.in());
  CHECK(!start.past_end);
  CHECK(!start.finished);

  // Finishing off the path is a stop, not an arrival. The caller is told how
  // far off it is, in path_error and off_path, and must gate on those if it
  // cares.
  PurePursuit sideways(path, config);
  const PurePursuitOutput wide = sideways.update(Pose2D{30.0, 25.0, 0.0});
  std::printf("   at (30, 25): past_end=%d finished=%d path_error=%.4f off_path=%d\n",
              static_cast<int>(wide.past_end), static_cast<int>(wide.finished),
              wide.path_error.in(), static_cast<int>(wide.off_path));
  CHECK(wide.past_end);
  CHECK(wide.finished);
  CHECK(wide.off_path);
  CHECK_NEAR(wide.path_error.in(), std::hypot(30.0, 1.0), 1e-9);
}

// ---------------------------------------------------------------------------
// 15. atDistance() heading across a corner.
// ---------------------------------------------------------------------------
void headingAcrossACorner() {
  std::printf("-- heading across a corner\n");
  // North for 24 in, then east for 24 in: a 90 degree corner at (0, 24). The
  // true tangent is compass 0 everywhere on the first leg and compass 90
  // everywhere on the second.
  Path path = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 24.0), wp(24.0, 24.0)});

  double worst = 0.0;
  double worst_at = 0.0;
  for (double d = 0.0; d <= 48.0; d += 0.25) {
    const double truth = (d < 24.0) ? 0.0 : 90.0;
    const double reported = path.atDistance(inch * d).heading.deg();
    const double error = radToDeg(
        std::fabs(mclib::wrapAngle(degToRad(reported - truth))));
    if (error > worst) {
      worst = error;
      worst_at = d;
    }
  }
  std::printf("   90 deg corner at 24 in: worst |heading - tangent| = %.4f deg at d = %.2f in\n",
              worst, worst_at);
  // Before the fix the heading ramped from 0 to 90 across the whole first leg,
  // so the error was 45 deg at the middle of that leg and 89.06 deg just short
  // of the corner.
  CHECK(worst < 1e-9);

  // Either side of the corner, spelled out.
  CHECK_NEAR(path.atDistance(inch * 1.0).heading.deg(), 0.0, 1e-9);
  CHECK_NEAR(path.atDistance(inch * 12.0).heading.deg(), 0.0, 1e-9);
  CHECK_NEAR(path.atDistance(inch * 23.9).heading.deg(), 0.0, 1e-9);
  CHECK_NEAR(path.atDistance(inch * 24.0).heading.deg(), 90.0, 1e-9);
  CHECK_NEAR(path.atDistance(inch * 24.1).heading.deg(), 90.0, 1e-9);
  CHECK_NEAR(path.atDistance(inch * 47.0).heading.deg(), 90.0, 1e-9);

  // A 135 degree corner, to show the reported heading is the leg's own bearing
  // whatever the corner angle is, not a blend of the two.
  Path sharp = Path::fromWaypoints({wp(0.0, 0.0), wp(0.0, 24.0), wp(-24.0, 0.0)});
  const double leg2 = std::hypot(24.0, 24.0);
  const double tangent2 =
      radToDeg(mclib::headingToward(Vec2{0.0, 24.0}, Vec2{-24.0, 0.0}));
  std::printf("   135 deg corner: leg 2 tangent = %.4f deg, reported at 24.1 in = %.4f deg\n",
              tangent2, sharp.atDistance(inch * 24.1).heading.deg());
  CHECK_NEAR(sharp.atDistance(inch * 12.0).heading.deg(), 0.0, 1e-9);
  CHECK_NEAR(sharp.atDistance(inch * (24.0 + leg2 * 0.5)).heading.deg(), tangent2, 1e-9);

  // A spline still gets a smoothly ramped heading: its samples carry real
  // curvature, so the turn is spread along the arc rather than parked on a
  // vertex. Checked against the true tangent of a circle.
  const double radius = 30.0;
  std::vector<Waypoint> arc_points;
  for (int i = 0; i <= 12; ++i) {
    const double angle = static_cast<double>(i) / 12.0 * (mclib::kPi / 2.0);
    arc_points.push_back(wp(radius * std::sin(angle), radius * (1.0 - std::cos(angle))));
  }
  SplineConfig spline_config;
  spline_config.spacing = 0.5 * inch;
  Path arc = generateSpline(arc_points, spline_config);
  double arc_worst = 0.0;
  for (double t = 0.05; t <= 0.95; t += 0.01) {
    const PathPoint point = arc.atParameter(t);
    // Quarter circle of radius `radius` centred on (0, radius), swept from
    // (0, 0) eastward. A point at sweep angle `a` sits at
    // `centre + (R sin a, -R cos a)`, and the compass tangent there is 90 - a.
    const double sweep = std::atan2(point.x.in(), radius - point.y.in());
    const double truth = mclib::kPi / 2.0 - sweep;
    arc_worst = std::fmax(arc_worst,
                          radToDeg(std::fabs(
                              mclib::wrapAngle(point.heading.rad() - truth))));
  }
  std::printf("   spline arc: worst |heading - true tangent| over t in [0.05, 0.95] = %.6f deg\n",
              arc_worst);
  CHECK(arc_worst < 0.2);
}

}  // namespace

int main() {
  frameCheck();
  goalBehind();
  pushedBackwards();
  searchWindowBound();
  crossTrackOnACorner();
  lookaheadDistance();
  arcCurvature();
  lateralOffsetSign();
  doubleBack();
  endAndOffPath();
  splineContinuity();
  velocityLimiting();
  pathQueries();
  pastTheEnd();
  headingAcrossACorner();
  return mclib::test::summary("path");
}
