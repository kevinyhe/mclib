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
  // Goal 5 in to the right, dead abeam: arc radius is 5/2 * ... -> k = 2*x/|p|^2
  // = 2*5/25 = 0.4 /in. Positive because it is to the right.
  CHECK(perInch(east_out.curvature) > 0.0);
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
  // Heading at the far end is +X, i.e. compass 90 deg.
  CHECK_NEAR(path.back().heading.deg(), 90.0, 1e-9);

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

}  // namespace

int main() {
  frameCheck();
  lookaheadDistance();
  arcCurvature();
  lateralOffsetSign();
  doubleBack();
  endAndOffPath();
  splineContinuity();
  velocityLimiting();
  pathQueries();
  return mclib::test::summary("path");
}
