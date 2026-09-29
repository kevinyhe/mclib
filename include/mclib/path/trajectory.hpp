// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/math.hpp"
#include "mclib/path/path.hpp"
#include "mclib/units/units.hpp"

#include <vector>

/**
 * @file trajectory.hpp
 * @brief Speeds and times planned along a `Path` before the robot moves.
 *
 * Pure pursuit picks its speed as it drives. A follower that tracks a target
 * pose over time (`Ramsete`) needs the whole plan up front: where the robot
 * should be at each moment, how fast, and how hard it should be turning.
 * `Trajectory::generate()` builds that plan from a path and a set of limits.
 *
 * ## How the speeds are chosen
 *
 * The path is resampled every `TrajectoryConstraints::spacing`. Each sample
 * gets a speed cap from three limits: `max_velocity`, the cornering limit
 * `sqrt(max_lateral_acceleration / |curvature|)`, and the outer-wheel limit
 * `max_velocity / (1 + |curvature| * track_width / 2)`. A forward pass then
 * limits how fast the speed can rise between samples (`max_acceleration`), and
 * a backward pass limits how fast it can fall (`max_deceleration`). Time comes
 * from the speeds, assuming constant acceleration between samples.
 *
 * Use a spline path from `generateSpline()`. A polyline from
 * `Path::fromWaypoints()` has zero curvature and a heading jump at every
 * corner, so the plan would drive straight into each corner at full speed.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace path {

using units::QAcceleration;
using units::QAngularVelocity;
using units::QTime;
using units::QVelocity;

/**
 * @brief Limits a trajectory is planned against.
 *
 * A zero `max_velocity` or `max_acceleration` gives an empty trajectory.
 */
struct TrajectoryConstraints {
  /// @brief Top speed of the robot's centre.
  QVelocity max_velocity{};
  /// @brief How fast the speed may rise.
  QAcceleration max_acceleration{};
  /// @brief How fast the speed may fall. 0 means the same as
  ///        `max_acceleration`.
  QAcceleration max_deceleration{};
  /// @brief Cornering budget: speed on a curve is capped at
  ///        `sqrt(max_lateral_acceleration / |curvature|)`. 0 means no
  ///        cornering cap.
  QAcceleration max_lateral_acceleration{};
  /// @brief Wheel spacing for a tank drive. When set, the outer wheel on a
  ///        curve is kept at or below `max_velocity`. 0 skips this cap.
  QLength track_width{};
  /// @brief Speed at the start of the path, e.g. when chaining. Capped by the
  ///        limits at the first sample.
  QVelocity start_velocity{};
  /// @brief Speed at the end of the path. 0 stops on the last point.
  QVelocity end_velocity{};
  /// @brief Drive the path backward: the back of the robot leads, speeds are
  ///        negative and the body heading is the path heading + 180 deg.
  bool reversed = false;
  /// @brief Distance between planned samples. Smaller follows the limits
  ///        more closely and costs more memory. 0 or negative uses 0.5 in.
  QLength spacing = 0.5 * units::inch;
};

/**
 * @brief Where the robot should be, and how it should be moving, at one time.
 */
struct TrajectoryState {
  /// @brief Time from the start of the trajectory.
  QTime time{};
  /// @brief Arc length from the start of the path. Always positive, even when
  ///        reversed.
  QLength distance{};
  /// @brief Field X.
  QLength x{};
  /// @brief Field Y.
  QLength y{};
  /// @brief Compass heading of the robot body. Equals the path heading, plus
  ///        180 deg when reversed.
  QAngle heading{};
  /// @brief Signed speed of the robot's centre. Negative when reversed.
  QVelocity velocity{};
  /// @brief Signed acceleration, in the same direction convention as
  ///        `velocity`.
  QAcceleration acceleration{};
  /// @brief Curvature of the path relative to the body. Positive turns
  ///        clockwise when driving forward. Reversed trajectories flip the
  ///        sign so that `velocity * curvature` is always the turn rate.
  units::QCurvature curvature{};

  /// @brief Clockwise-positive turn rate: `velocity * curvature`.
  QAngularVelocity angularVelocity() const {
    return units::turnRate(velocity, curvature);
  }
  /// @brief `(x, y, heading)` in inches and radians.
  Pose2D pose() const { return Pose2D{x.in(), y.in(), heading.rad()}; }
};

/**
 * @brief A time-stamped plan along a path.
 *
 * Build one with `generate()`, then call `sample()` each control tick.
 */
class Trajectory {
 public:
  Trajectory() = default;

  /**
   * @brief Plan speeds and times along @p path.
   * @return An empty trajectory when the path is not `valid()` or a required
   *         limit is zero, negative or not finite.
   */
  static Trajectory generate(const Path& path,
                             const TrajectoryConstraints& constraints);

  /// @brief True when there is nothing to follow.
  bool empty() const { return m_states.empty(); }
  /// @brief Time to drive the whole path. Zero when empty.
  QTime duration() const;
  /// @brief Path length. Zero when empty.
  QLength length() const;
  /// @brief The planned samples, one per `spacing` of path.
  const std::vector<TrajectoryState>& states() const { return m_states; }

  /**
   * @brief The planned state at time @p t.
   *
   * Between samples the speed follows constant acceleration and the position
   * is interpolated along the path. Before the start this returns the first
   * state; after the end, the last state. Empty trajectories return a
   * default state.
   */
  TrajectoryState sample(QTime t) const;

 private:
  std::vector<TrajectoryState> m_states;
};

}  // namespace path
}  // namespace mclib
