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
 * @file pose_path.hpp
 * @brief The curve `moveToPose()` drives: from the robot's pose to a target
 * pose, leaving along the start heading and arriving along the target heading.
 *
 * It is a cubic Bezier. Its two inner control points sit along the start and
 * end headings, each `lead` times the straight-line distance from the ends.
 * A `lead` of about 0.39 turns a 90 degree corner into a near-circular arc.
 * Smaller values take the corner tighter and run straighter in between;
 * larger values swing wider. A curve that folds back on itself, where the
 * robot would have to stop and reverse, is refused: a target behind the
 * start heading, one to arrive at against the start heading, or a target
 * straight ahead with a `lead` of 1 or more.
 *
 * The path's headings are the direction of travel, so for a robot that backs
 * along it plan the trajectory with `reversed = true`.
 * Host-only: no PROS headers.
 */

namespace mclib {
namespace path {

/**
 * @brief Bake the curve from @p start to @p end.
 * @param start Robot pose: inches, radians, compass frame.
 * @param end Target pose. `end.theta` is the body heading to arrive on.
 * @param reversed True when the robot backs along the curve: it travels
 *        opposite its body heading at both ends.
 * @param lead Length of each control arm over the straight-line distance.
 * @param spacing Distance between samples, about.
 * @return An invalid path (`valid()` false) when @p start and @p end are at
 *         the same point, or a value is not finite, or @p lead is not
 *         positive.
 */
Path planToPose(const Pose2D& start, const Pose2D& end, bool reversed, double lead,
                units::QLength spacing = 0.5 * units::inch);

}  // namespace path
}  // namespace mclib
