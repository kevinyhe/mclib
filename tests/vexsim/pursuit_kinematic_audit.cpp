// Independent audit: actual mclib follower, exact sample-held wheel kinematics.
// This deliberately does not use vexsim, mclib odometry, arcRadius(),
// fieldToRobot(), turnRate(), or motion-controller implementations as its oracle.
// Example build from mclib:
// g++ -std=gnu++20 -O1 -g -Wall -Wextra -DMCLIB_HOST_BUILD -Iinclude -Itests
//     tests/vexsim/pursuit_kinematic_audit.cpp src/mclib/math.cpp
//     src/mclib/path/path.cpp src/mclib/path/pure_pursuit.cpp
//     src/mclib/path/spline.cpp -pthread -o /tmp/pursuit_kinematic_audit

#include "mclib/path/path.hpp"
#include "mclib/path/pure_pursuit.hpp"
#include "mclib/path/spline.hpp"
#include "test_assert.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace {

using mclib::Pose2D;
using mclib::path::Path;
using mclib::path::PathPoint;
using mclib::path::PurePursuit;
using mclib::path::PurePursuitConfig;
using mclib::path::PurePursuitOutput;
using mclib::path::Waypoint;
using mclib::units::inch;
using mclib::units::inps;
using mclib::units::second;
constexpr double pi = 3.141592653589793238462643383279502884;

Waypoint wp(double x, double y) { return {x * inch, y * inch}; }

PathPoint point(double x, double y, double heading = 0.0, double curvature = 0.0) {
  PathPoint p;
  p.x = x * inch;
  p.y = y * inch;
  p.heading = mclib::units::QAngle::fromBase(heading);
  p.curvature = curvature / inch;
  return p;
}

double curvatureInches(const PurePursuitOutput& out) {
  return out.curvature.raw() * 0.0254;
}

double separation(const Pose2D& a, const Pose2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Integrate dx/dt = v sin(theta), dy/dt = v cos(theta),
// dtheta/dt = (left - right) / track exactly for constant wheel speeds.
Pose2D exactStep(Pose2D p, double left, double right, double track, double dt) {
  const double speed = (left + right) * 0.5;
  const double turn = (left - right) * dt / track;
  const double half = turn * 0.5;
  const double sinc = std::fabs(half) < 1e-9
                          ? 1.0 - half * half / 6.0
                          : std::sin(half) / half;
  const double travel = speed * dt * sinc;
  p.x += travel * std::sin(p.theta + half);
  p.y += travel * std::cos(p.theta + half);
  p.theta += turn;
  return p;
}

// A separate RK4 integration verifies the exact oracle without using its
// midpoint/sinc formula. Wheel commands remain constant during each sample.
Pose2D numericalStep(Pose2D p, double left, double right, double track, double dt) {
  const double speed = 0.5 * (left + right);
  const double omega = (left - right) / track;
  constexpr int subdivisions = 20;
  const double h = dt / subdivisions;
  for (int i = 0; i < subdivisions; ++i) {
    const double first = p.theta;
    const double middle = first + omega * h * 0.5;
    const double last = first + omega * h;
    p.x += speed * h * (std::sin(first) + 4 * std::sin(middle) + std::sin(last)) / 6;
    p.y += speed * h * (std::cos(first) + 4 * std::cos(middle) + std::cos(last)) / 6;
    p.theta = last;
  }
  return p;
}

void checkOutput(const PurePursuitOutput& out, const PurePursuitConfig& config) {
  CHECK(std::isfinite(out.lookahead_point.x()) && std::isfinite(out.lookahead_point.y()));
  CHECK(std::isfinite(out.velocity.inps()) && std::isfinite(out.turn_rate.raw()));
  CHECK(std::isfinite(out.curvature.raw()) && std::isfinite(out.cross_track_error.in()));
  CHECK(std::isfinite(out.distance_along.in()) && std::isfinite(out.path_error.in()));
  const double left = out.wheels.left.inps();
  const double right = out.wheels.right.inps();
  CHECK(std::isfinite(left) && std::isfinite(right));
  CHECK(std::fabs(left) <= config.max_velocity.inps() + 1e-10);
  CHECK(std::fabs(right) <= config.max_velocity.inps() + 1e-10);
  CHECK_NEAR((left + right) * 0.5, out.velocity.inps(), 1e-10);
  CHECK_NEAR((left - right) / config.track_width.in(), out.turn_rate.raw(), 1e-10);
  if (out.finished) {
    CHECK_NEAR(left, 0.0, 1e-12);
    CHECK_NEAR(right, 0.0, 1e-12);
    CHECK_NEAR(curvatureInches(out), 0.0, 1e-12);
  }
}

void exactOracleAndSignedWheelMapping() {
  double worst = 0.0;
  for (double left : {-48.0, -6.0, 0.0, 6.0, 48.0}) {
    for (double right : {-48.0, -6.0, 0.0, 6.0, 48.0}) {
      for (double theta : {-2.4, 0.0, 0.9, 3.1}) {
        const Pose2D start{17.0, -29.0, theta};
        const auto exact = exactStep(start, left, right, 12.0, 0.01);
        const auto numerical = numericalStep(start, left, right, 12.0, 0.01);
        worst = std::max(worst, separation(exact, numerical));
        CHECK_NEAR(exact.x, numerical.x, 1e-11);
        CHECK_NEAR(exact.y, numerical.y, 1e-11);
        CHECK_NEAR(exact.theta, numerical.theta, 1e-12);
      }
    }
  }
  for (double speed : {-40.0, -6.0, 0.0, 6.0, 40.0}) {
    for (double k : {-0.25, -0.05, 0.0, 0.05, 0.25}) {
      const auto wheels = mclib::path::wheelSpeeds(speed * inps, k / inch, 12.0 * inch);
      CHECK_NEAR((wheels.left.inps() + wheels.right.inps()) / 2, speed, 1e-12);
      CHECK_NEAR((wheels.left.inps() - wheels.right.inps()) / 12, speed * k, 1e-12);
    }
  }
  std::printf("ORACLE exact_vs_RK4_max_error_in=%.12g signed_wheel_cases=25\n", worst);
}

void independentGoalGeometry() {
  double worst = 0.0;
  int cases = 0;
  for (double theta : {-2.1, -0.6, 0.0, 0.9, 2.7}) {
    for (double local_x : {-16.0, -5.0, -0.5, 0.0, 0.5, 5.0, 16.0}) {
      for (double local_y : {3.0, 10.0, 24.0}) {
        const Pose2D pose{13.0, -7.0, theta};
        const double goal_x = pose.x + local_x * std::cos(theta) + local_y * std::sin(theta);
        const double goal_y = pose.y - local_x * std::sin(theta) + local_y * std::cos(theta);
        const Path path = Path::fromWaypoints({wp(pose.x, pose.y), wp(goal_x, goal_y)});
        PurePursuitConfig config;
        config.lookahead = 100.0 * inch;
        config.max_curvature = 0.0 / inch;
        PurePursuit follower(path, config);
        const auto out = follower.update(pose);
        CHECK(!out.finished);
        CHECK_NEAR(out.lookahead_point.x(), goal_x, 1e-10);
        CHECK_NEAR(out.lookahead_point.y(), goal_y, 1e-10);
        const double truth = 2 * local_x / (local_x * local_x + local_y * local_y);
        worst = std::max(worst, std::fabs(curvatureInches(out) - truth));
        CHECK_NEAR(curvatureInches(out), truth, 1e-11);
        checkOutput(out, config);
        ++cases;
      }
    }
  }
  std::printf("GEOMETRY cases=%d max_curvature_error_per_in=%.12g\n", cases, worst);
}

PurePursuitConfig trackingConfig() {
  PurePursuitConfig config;
  config.lookahead = 8.0 * inch;
  config.max_velocity = 36.0 * inps;
  config.min_velocity = 2.0 * inps;
  config.finish_tolerance = 0.5 * inch;
  return config;
}

struct RunResult {
  Pose2D final{};
  double seconds = 0.0;
  double endpoint_error = 0.0;
  double max_path_error = 0.0;
  double max_radial_error = 0.0;
  bool finished = false;
};

RunResult follow(const char* name, const Path& path, Pose2D pose,
                 PurePursuitConfig config, double dt = 0.01,
                 bool circular = false, double center_x = 0.0, double radius = 0.0) {
  PurePursuit follower(path, config);
  double last_progress = 0.0;
  RunResult result;
  for (int tick = 0; tick < static_cast<int>(30.0 / dt); ++tick) {
    const auto out = follower.update(pose);
    checkOutput(out, config);
    CHECK(out.distance_along.in() >= last_progress - 1e-9);
    CHECK(out.distance_along.in() <= path.length().in() + 1e-9);
    CHECK_NEAR(out.remaining.in() + out.distance_along.in(), path.length().in(), 1e-8);
    CHECK(out.velocity.inps() >= -1e-12);  // This API has no reverse-path mode.
    last_progress = out.distance_along.in();
    result.seconds = tick * dt;
    result.max_path_error = std::max(result.max_path_error, out.path_error.in());
    if (circular) {
      result.max_radial_error = std::max(result.max_radial_error,
                                       std::fabs(std::hypot(pose.x - center_x, pose.y) - radius));
    }
    if (out.finished) {
      result.finished = true;
      break;
    }
    pose = exactStep(pose, out.wheels.left.inps(), out.wheels.right.inps(),
                     config.track_width.in(), dt);
  }
  result.final = pose;
  result.endpoint_error = std::hypot(pose.x - path.back().x.in(),
                                     pose.y - path.back().y.in());
  std::printf("TRACK %-24s dt=%.4f finish=%d t=%.3f endpoint_in=%.6f "
              "max_path_in=%.6f radial_in=%.6f final=(%.6f,%.6f,%.6f)\n",
              name, dt, result.finished, result.seconds, result.endpoint_error,
              result.max_path_error, result.max_radial_error,
              pose.x, pose.y, pose.theta);
  CHECK(result.finished);
  return result;
}

Path circle(double radius, double sign) {
  std::vector<PathPoint> samples;
  for (int i = 0; i <= 720; ++i) {
    const double angle = pi * 0.5 * i / 720;
    samples.push_back(point(sign * radius * (1 - std::cos(angle)),
                            radius * std::sin(angle), sign * angle, sign / radius));
  }
  return Path(samples);
}

Path sinusoid() {
  std::vector<PathPoint> samples;
  constexpr double amplitude = 12.0;
  constexpr double length = 96.0;
  for (int i = 0; i <= 960; ++i) {
    const double t = static_cast<double>(i) / 960;
    const double dx = amplitude * 2 * pi * std::cos(2 * pi * t);
    const double ddx = -amplitude * 4 * pi * pi * std::sin(2 * pi * t);
    const double curvature = length * ddx / std::pow(dx * dx + length * length, 1.5);
    samples.push_back(point(amplitude * std::sin(2 * pi * t), length * t,
                            std::atan2(dx, length), curvature));
  }
  return Path(samples);
}

void closedLoopTrajectories() {
  const auto config = trackingConfig();
  const Path straight = Path::fromWaypoints({wp(0, 0), wp(0, 96)});
  auto result = follow("straight_north", straight, {0, 0, 0}, config);
  CHECK(result.endpoint_error <= 0.51);
  CHECK(result.max_path_error <= 1e-8);
  const auto fine = follow("straight_north_fine", straight, {0, 0, 0}, config, 0.005);
  CHECK(separation(result.final, fine.final) <= 0.1);

  const Path east = Path::fromWaypoints({wp(7, -9), wp(103, -9)});
  result = follow("straight_east", east, {7, -9, pi / 2}, config);
  CHECK(result.endpoint_error <= 0.51);
  CHECK(result.max_path_error <= 1e-8);

  for (double sign : {-1.0, 1.0}) {
    const Path arc = circle(24.0, sign);
    result = follow(sign > 0 ? "quarter_circle_right" : "quarter_circle_left",
                    arc, {0, 0, 0}, config, 0.01, true, sign * 24.0, 24.0);
    CHECK(result.endpoint_error <= 0.51);
    CHECK(result.max_radial_error <= 0.03);
    const auto refined = follow(sign > 0 ? "quarter_right_fine" : "quarter_left_fine",
                                arc, {0, 0, 0}, config, 0.005, true, sign * 24.0, 24.0);
    CHECK(separation(result.final, refined.final) <= 0.1);
  }

  const Path bend = Path::fromWaypoints({wp(0, 0), wp(0, 48), wp(48, 48)});
  result = follow("L_bend_polyline", bend, {0, 0, 0}, config);
  CHECK(result.endpoint_error <= 0.75);
  // A finite-lookahead follower deliberately rounds a polyline corner.
  CHECK(result.max_path_error <= config.lookahead.in());

  const Path smooth_s = sinusoid();
  result = follow("S_bend_analytic", smooth_s, {0, 0, std::atan2(24 * pi, 96)}, config);
  CHECK(result.endpoint_error <= 0.75);
  const auto refined_s = follow("S_bend_fine", smooth_s,
                                {0, 0, std::atan2(24 * pi, 96)}, config, 0.005);
  CHECK(separation(result.final, refined_s.final) <= 0.1);

  const Path spline = mclib::path::generateSpline(
      {wp(0, 0), wp(12, 24), wp(-12, 48), wp(0, 72)});
  result = follow("S_bend_library_spline", spline, spline.front().pose(), config);
  CHECK(result.endpoint_error <= 0.75);

  result = follow("off_path_right_18in", straight, {18, 0, 0}, config);
  CHECK(result.endpoint_error <= 0.75);
  result = follow("off_path_left_18in", straight, {-18, 0, 0}, config);
  CHECK(result.endpoint_error <= 0.75);
  result = follow("before_start_20in", straight, {0, -20, 0}, config);
  CHECK(result.endpoint_error <= 0.75);
  result = follow("backward_facing_start", straight, {0, 0, pi}, config);
  CHECK(result.endpoint_error <= 0.75);
}

void progressResetAndLookahead() {
  const Path path = Path::fromWaypoints({wp(0, 0), wp(0, 96)});
  PurePursuit follower(path);
  double prior = 0.0;
  for (double y : {0.0, 5.0, 15.0, 30.0, 28.0, 5.0, 150.0, 6.0}) {
    const auto out = follower.update({0, y, 0});
    CHECK(out.distance_along.in() >= prior - 1e-10);
    CHECK(out.distance_along.in() <= prior + 24.0 + 1e-10);
    prior = out.distance_along.in();
  }
  follower.reset();
  CHECK_NEAR(follower.progress().in(), 0, 1e-12);
  const auto after_reset = follower.update({0, 0, 0});
  CHECK_NEAR(after_reset.distance_along.in(), 0, 1e-12);
  CHECK_NEAR(after_reset.lookahead_point.y(), 12, 1e-12);
  follower.setPath(Path::fromWaypoints({wp(0, 0), wp(96, 0)}));
  CHECK_NEAR(follower.progress().in(), 0, 1e-12);
  const auto after_replace = follower.update({0, 0, pi / 2});
  CHECK_NEAR(after_replace.lookahead_point.x(), 12, 1e-12);
  CHECK_NEAR(after_replace.lookahead_point.y(), 0, 1e-12);

  const Path hairpin = Path::fromWaypoints({wp(0, 0), wp(0, 48), wp(8, 48), wp(8, 0)});
  PurePursuit turn(hairpin);
  const auto first = turn.update({0, 0, 0});
  CHECK_NEAR(first.distance_along.in(), 0, 1e-12);
  CHECK_NEAR(first.lookahead_point.x(), 0, 1e-12);
  CHECK_NEAR(first.lookahead_point.y(), 12, 1e-12);
  CHECK(!first.past_end && !first.finished);

  PurePursuit clamped(path);
  const auto behind = clamped.update({0, 0, pi});
  CHECK_NEAR(std::fabs(curvatureInches(behind)), 1.0 / 6.0, 1e-12);
  CHECK(!behind.finished);
  // Explicitly documented: crossing the last-leg end plane can finish far
  // from the endpoint. Record the distinction; do not assert fake arrival.
  const Path short_path = Path::fromWaypoints({wp(0, 0), wp(0, 24)});
  PurePursuit past(short_path);
  const auto away = past.update({30, 25, 0});
  checkOutput(away, PurePursuitConfig{});
  CHECK(away.finished && away.past_end && away.off_path);
  std::printf("CONTRACT past_end_finished_with_endpoint_error_in=%.6f\n", std::hypot(30.0, 1.0));
}

void degeneratesAndDuplicates() {
  const std::vector<Path> invalid{
      Path{}, Path::fromWaypoints({wp(0, 0)}),
      Path::fromWaypoints({wp(0, 0), wp(0, 0), wp(0, 0)})};
  for (const auto& path : invalid) {
    CHECK(!path.valid());
    PurePursuit follower(path);
    const auto out = follower.update({30, -30, 1.0});
    checkOutput(out, PurePursuitConfig{});
    CHECK(out.finished);
  }

  // The public raw-sample constructor says callers only need positions.
  // A sample list with no nonzero segment has no path to follow.
  Path repeated(std::vector<PathPoint>{point(0, 0), point(0, 0), point(0, 0)});
  PurePursuit zero_length(repeated);
  const auto empty_route = zero_length.update({30, -30, 0});
  std::printf("DEGENERATE raw_duplicate_only valid=%d length_in=%.6f finished=%d "
              "left_inps=%.6f right_inps=%.6f\n", repeated.valid(), repeated.length().in(),
              empty_route.finished, empty_route.wheels.left.inps(), empty_route.wheels.right.inps());
  CHECK(!repeated.valid());
  CHECK(empty_route.finished);
  CHECK_NEAR(empty_route.velocity.inps(), 0, 1e-12);

  Path duplicate_start(std::vector<PathPoint>{point(0, 0), point(0, 0), point(24, 0)});
  const Path clean = Path::fromWaypoints({wp(0, 0), wp(24, 0)});
  PurePursuit with_duplicate(duplicate_start), without_duplicate(clean);
  const auto actual = with_duplicate.update({0, 3, pi / 2});
  const auto reference = without_duplicate.update({0, 3, pi / 2});
  // North of an eastbound segment is 3 inches left of the path, regardless of
  // a zero-length initial sample. Compare with independent compass geometry.
  std::printf("DEGENERATE duplicate_initial_segment cross_track_in=%.6f clean_in=%.6f expected=-3\n",
              actual.cross_track_error.in(), reference.cross_track_error.in());
  CHECK_NEAR(actual.cross_track_error.in(), -3, 1e-10);
  CHECK_NEAR(reference.cross_track_error.in(), -3, 1e-10);
  CHECK_NEAR(actual.lookahead_point.x(), reference.lookahead_point.x(), 1e-10);
  CHECK_NEAR(actual.lookahead_point.y(), reference.lookahead_point.y(), 1e-10);
}

}  // namespace

int main() {
  exactOracleAndSignedWheelMapping();
  independentGoalGeometry();
  closedLoopTrajectories();
  progressResetAndLookahead();
  degeneratesAndDuplicates();
  return mclib::test::summary("independent pure pursuit kinematic audit");
}
