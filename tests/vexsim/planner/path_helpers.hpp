// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @file path_helpers.hpp
 * @brief Drive a path drawn in the path planner with mclib's planner and
 *        RAMSETE follower.
 *
 * The planner draws a path as cubic Bezier sections, each with its own speed
 * limit and direction. This file bakes the sections into `mclib::path::Path`
 * samples, plans one `Trajectory` per run of sections that share a speed limit
 * and a direction, and follows the plans one after another.
 *
 * The simulator runs this file, and Export C++ copies it into the generated
 * autonomous, so the robot plans exactly what the simulator drove.
 *
 * Where the speed limit changes, the speed carries across: each plan ends at
 * the speed the next one can start at. Where the direction flips, the robot
 * stops.
 *
 * A path that keeps moving into the next path starts and ends at the speeds
 * the planner worked out for the join, and its last plan leaves the drive
 * powered (`exit` false) instead of stopping and settling.
 */

// BEGIN EXPORTED HELPERS
#include "mclib/control/motion.hpp"
#include "mclib/control/profile.hpp"
#include "mclib/control/ramsete.hpp"
#include "mclib/path/path.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/time.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace path_planner {

struct Point {
  double x;
  double y;
};

/// @brief One cubic Bezier section in field inches. The robot passes through
///        `start` and `end`; the two handles shape the curve between them.
struct Section {
  Point start;
  Point start_handle;
  Point end_handle;
  Point end;
  double max_speed_ips;
  bool reversed;
};

/// @brief Limits shared by every section of a path.
struct Limits {
  double max_acceleration_ips2;
  double max_lateral_acceleration_ips2;
  double track_width_in;
};

inline constexpr double kSampleSpacingIn = 0.5;
inline constexpr int kMinimumSamplesPerSection = 8;
inline constexpr double kSmallestTangent = 1e-6;

namespace detail {

inline Point bezierPoint(const Section& s, double t) {
  const double u = 1.0 - t;
  const double a = u * u * u, b = 3 * u * u * t, c = 3 * u * t * t, d = t * t * t;
  return {a * s.start.x + b * s.start_handle.x + c * s.end_handle.x + d * s.end.x,
          a * s.start.y + b * s.start_handle.y + c * s.end_handle.y + d * s.end.y};
}

inline Point bezierTangent(const Section& s, double t) {
  const double u = 1.0 - t;
  const double a = 3 * u * u, b = 6 * u * t, c = 3 * t * t;
  return {a * (s.start_handle.x - s.start.x) + b * (s.end_handle.x - s.start_handle.x) +
              c * (s.end.x - s.end_handle.x),
          a * (s.start_handle.y - s.start.y) + b * (s.end_handle.y - s.start_handle.y) +
              c * (s.end.y - s.end_handle.y)};
}

inline Point bezierBend(const Section& s, double t) {
  const double u = 1.0 - t;
  return {6 * u * (s.end_handle.x - 2 * s.start_handle.x + s.start.x) +
              6 * t * (s.end.x - 2 * s.end_handle.x + s.start_handle.x),
          6 * u * (s.end_handle.y - 2 * s.start_handle.y + s.start.y) +
              6 * t * (s.end.y - 2 * s.end_handle.y + s.start_handle.y)};
}

inline int sampleCount(const Section& s) {
  const double polygon = std::hypot(s.start_handle.x - s.start.x, s.start_handle.y - s.start.y) +
                         std::hypot(s.end_handle.x - s.start_handle.x, s.end_handle.y - s.start_handle.y) +
                         std::hypot(s.end.x - s.end_handle.x, s.end.y - s.end_handle.y);
  return std::max(kMinimumSamplesPerSection,
                  static_cast<int>(std::ceil(polygon / kSampleSpacingIn)));
}

// Heading is the direction of travel along the curve; curvature is positive
// when the curve bends clockwise, as in mclib/path/path.hpp. False when the
// curve stops or doubles back, where neither is defined: the tangent then
// vanishes or turns more than 90 deg between two samples half an inch apart.
inline bool appendSection(std::vector<mclib::path::PathPoint>& points, const Section& s) {
  using namespace mclib::units;
  const int count = sampleCount(s);
  Point previous_tangent = bezierTangent(s, 0.0);
  for (int i = points.empty() ? 0 : 1; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    const Point position = bezierPoint(s, t);
    const Point tangent = bezierTangent(s, t);
    const Point bend = bezierBend(s, t);
    const double speed = std::hypot(tangent.x, tangent.y);
    if (speed < kSmallestTangent) return false;
    if (tangent.x * previous_tangent.x + tangent.y * previous_tangent.y <= 0) return false;
    previous_tangent = tangent;
    const double curvature_per_in = (tangent.y * bend.x - tangent.x * bend.y) / (speed * speed * speed);
    mclib::path::PathPoint point;
    point.x = position.x * inch;
    point.y = position.y * inch;
    point.heading = QAngle::fromBase(std::atan2(tangent.x, tangent.y));
    point.curvature = QCurvature::fromBase(curvature_per_in / (1.0 * inch).raw());
    points.push_back(point);
  }
  return true;
}

struct Part {
  mclib::path::Path path;
  double max_speed_ips;
  bool reversed;
};

inline std::vector<Part> bakeParts(const std::vector<Section>& sections) {
  std::vector<Part> parts;
  std::vector<mclib::path::PathPoint> points;
  for (std::size_t i = 0; i < sections.size(); ++i) {
    if (!appendSection(points, sections[i])) return {};
    const bool part_ends = i + 1 == sections.size() ||
                           sections[i + 1].reversed != sections[i].reversed ||
                           sections[i + 1].max_speed_ips != sections[i].max_speed_ips;
    if (part_ends) {
      parts.push_back({mclib::path::Path(points), sections[i].max_speed_ips, sections[i].reversed});
      points.clear();
    }
  }
  return parts;
}

inline mclib::path::TrajectoryConstraints constraintsFor(const Part& part, const Limits& limits,
                                                         double start_ips, double end_ips) {
  using namespace mclib::units;
  mclib::path::TrajectoryConstraints constraints;
  constraints.max_velocity = part.max_speed_ips * inps;
  constraints.max_acceleration = limits.max_acceleration_ips2 * mclib::control::inps2;
  constraints.max_lateral_acceleration = limits.max_lateral_acceleration_ips2 * mclib::control::inps2;
  constraints.track_width = limits.track_width_in * inch;
  constraints.start_velocity = start_ips * inps;
  constraints.end_velocity = end_ips * inps;
  constraints.reversed = part.reversed;
  return constraints;
}

}  // namespace detail

/**
 * @brief Plan @p sections as one trajectory per run of sections with the
 *        same speed limit and direction.
 * @param start_ips Speed at the first point, 0 from a stop. Lowered to what
 *                  the path allows there.
 * @param end_ips   Speed at the last point, 0 to stop. Lowered to what the
 *                  path can reach.
 * @return The plans in driving order, or nothing when a section stops or
 *         doubles back on itself or a limit is not positive.
 */
inline std::vector<mclib::path::Trajectory> planSections(const std::vector<Section>& sections,
                                                         const Limits& limits, double start_ips = 0.0,
                                                         double end_ips = 0.0) {
  const std::vector<detail::Part> parts = detail::bakeParts(sections);
  if (parts.empty()) return {};
  const std::size_t count = parts.size();
  const auto keepsDirection = [&](std::size_t i) {
    return i + 1 < count && parts[i + 1].reversed == parts[i].reversed;
  };
  // Last to first: the fastest speed each part can start at and still slow
  // down for everything after it. The part before ends at that speed.
  std::vector<double> part_end_ips(count, 0.0);
  part_end_ips.back() = end_ips;
  for (std::size_t i = count; i-- > 1;) {
    const mclib::path::Trajectory fastest = mclib::path::Trajectory::generate(
        parts[i].path, detail::constraintsFor(parts[i], limits, parts[i].max_speed_ips, part_end_ips[i]));
    if (fastest.empty()) return {};
    if (keepsDirection(i - 1)) part_end_ips[i - 1] = std::fabs(fastest.states().front().velocity.inps());
  }
  std::vector<mclib::path::Trajectory> plans;
  double part_start_ips = start_ips;
  for (std::size_t i = 0; i < count; ++i) {
    plans.push_back(mclib::path::Trajectory::generate(
        parts[i].path, detail::constraintsFor(parts[i], limits, part_start_ips, part_end_ips[i])));
    if (plans.back().empty()) return {};
    part_start_ips = keepsDirection(i) ? std::fabs(plans.back().states().back().velocity.inps()) : 0.0;
  }
  return plans;
}

/**
 * @brief Follow @p plans from planSections() with followTrajectory(), all
 *        within @p time_limit.
 * @param exit True stops and settles at the end of the last plan. False
 *             leaves the drive powered for the next motion.
 */
inline mclib::control::MotionResult followSections(const std::vector<mclib::path::Trajectory>& plans,
                                                   const mclib::control::RamseteConfig& config,
                                                   mclib::units::QTime time_limit,
                                                   mclib::units::QVoltage max_output, bool exit = true) {
  using mclib::control::MotionResult;
  if (plans.empty()) return MotionResult::InvalidValue;
  const std::uint32_t start_ms = mclib::time::millis();
  for (std::size_t i = 0; i < plans.size(); ++i) {
    const mclib::units::QTime remaining =
        time_limit - static_cast<double>(mclib::time::msSince(start_ms)) * mclib::units::millisecond;
    if (!(remaining.raw() > 0)) return MotionResult::TimedOut;
    const bool settle = exit && i + 1 == plans.size();
    const MotionResult result = ::followTrajectory(plans[i], config, remaining, settle, max_output);
    if (result != MotionResult::Reached) return result;
  }
  return MotionResult::Reached;
}

}  // namespace path_planner
// END EXPORTED HELPERS
