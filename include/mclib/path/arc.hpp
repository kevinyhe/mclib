// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/math.hpp"
#include "mclib/path/path.hpp"
#include "mclib/units/units.hpp"

/**
 * @file arc.hpp
 * @brief The constant-radius arc `curveCircle()` drives, as a baked `Path`.
 *
 * Same geometry and sign rules as `curveCircle()`:
 *
 * - The heading turns from the start heading to @p end_heading the short
 *   way round.
 * - The radius sign puts the centre on the robot's right (+) or left (-),
 *   in its own frame at the start.
 * - Driving forward along a circle whose centre is on the right turns the
 *   robot clockwise. So the robot drives forward when the radius sign and
 *   the turn direction agree, and backward when they don't.
 *
 * The path's headings are the direction of travel, so for a backward arc
 * plan the trajectory with `reversed = true`; `ArcPlan::reversed` says which.
 *
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace path {

/// @brief An arc and which way the robot travels along it.
struct ArcPlan {
  Path path;
  /// @brief True when the robot backs along the arc.
  bool reversed = false;
};

/**
 * @brief Bake the arc from @p start to @p end_heading.
 * @param start Robot pose: inches, radians, compass frame.
 * @param end_heading Absolute compass heading to finish on.
 * @param signed_radius Radius to the robot's centre. Positive: centre on the
 *        right.
 * @param spacing Distance between samples.
 * @return An invalid path (`valid()` false) for a zero radius or no turn.
 */
ArcPlan planArc(const Pose2D& start, units::QAngle end_heading, units::QLength signed_radius,
                units::QLength spacing = 0.5 * units::inch);

}  // namespace path
}  // namespace mclib
