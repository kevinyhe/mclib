// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/feedforward.hpp"
#include "mclib/math.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/units/units.hpp"

/**
 * @file holonomic_follower.hpp
 * @brief One step of trajectory following for an X-drive or mecanum drive.
 *
 * A holonomic drive can move in any direction while facing any way, so the
 * direction of travel and the heading are two separate targets:
 *
 * - **Where to be** comes from the trajectory. Its speed along the path
 *   tangent is the feedforward; a proportional term on the x and y position
 *   error, in the field frame, corrects drift.
 * - **Which way to face** is a separate heading and turn rate, with its own
 *   proportional correction.
 *
 * The field-frame velocity is rotated into the robot frame with the odometry
 * heading, then turned into forward, strafe and turn volts by
 * `HolonomicFeedforward`. Those three go through `holonomic::mix()` like any
 * other holonomic command.
 *
 * Plan the trajectory with `reversed = false`. On a holonomic drive the
 * trajectory only sets the path; which way the robot faces is up to the
 * heading target.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace control {

using QVoltagePerAngularVelocity =
    decltype(units::QVoltage{} / units::QAngularVelocity{});

/// @brief Feedforward for turning in place: `kS * sgn(w) + kV * w`.
struct TurnFeedforwardGains {
  units::QVoltage kS{};
  QVoltagePerAngularVelocity kV{};
};

/**
 * @brief Volts for a robot-frame velocity on a holonomic drive.
 *
 * Measure each axis on its own, as in docs/motion.md: drive straight forward
 * for `forward`, strafe for `strafe` (mecanum rollers slip, so its `kV` is
 * larger), and spin in place for `turn`. Acceleration terms are not used.
 */
struct HolonomicFeedforward {
  FeedforwardGains forward{};
  FeedforwardGains strafe{};
  TurnFeedforwardGains turn{};
};

/// @brief Tuning for `holonomicFollowStep()`.
struct HolonomicFollowerConfig {
  /// @brief Extra speed per inch of position error, 1/s. 3 closes a 1 in
  ///        error at 3 in/s.
  double translation_kp = 3.0;
  /// @brief Extra turn rate per radian of heading error, 1/s.
  double heading_kp = 4.0;
  HolonomicFeedforward feedforward{};
};

/// @brief Robot-frame volts, before `holonomic::mix()`.
struct HolonomicVolts {
  double forward = 0.0;  ///< Toward the robot's front.
  double strafe = 0.0;   ///< Toward the robot's right.
  double turn = 0.0;     ///< Clockwise.
};

/// @brief What the robot should be doing at one moment.
struct HolonomicTarget {
  /// @brief Position and speed along the path.
  path::TrajectoryState state{};
  /// @brief Which way to face.
  units::QAngle heading{};
  /// @brief Planned clockwise turn rate toward @ref heading.
  units::QAngularVelocity heading_rate{};
};

/**
 * @brief The heading target at time @p t for a turn from @p start to
 *        @p end spread evenly over @p duration, the short way round.
 *
 * `heading_rate` is constant during the turn and zero outside it. Zero
 * @p duration jumps straight to @p end.
 */
HolonomicTarget holonomicTarget(const path::TrajectoryState& state,
                                units::QAngle start, units::QAngle end,
                                units::QTime duration, units::QTime t);

/**
 * @brief Robot-frame volts that steer from @p pose toward @p target.
 * @param pose Odometry pose: inches, radians, compass frame.
 */
HolonomicVolts holonomicFollowStep(const Pose2D& pose,
                                   const HolonomicTarget& target,
                                   const HolonomicFollowerConfig& config);

}  // namespace control
}  // namespace mclib
