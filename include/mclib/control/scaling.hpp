// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @file scaling.hpp
 * @brief Ratio-preserving floor and cap on a left/right voltage pair.
 *
 * Both values are **volts**, and both are read and written in place. They stay
 * `double` rather than `QVoltage` for the same reason the rest of the motion
 * inner loop does: these run inside a 10 ms control loop whose numbers are what
 * the robot was tuned on, and a round trip through SI base units is not
 * bit-exact. See the rationale at the top of `control/motion_math.hpp`.
 */

/**
 * @brief Raise the smaller-magnitude side to @p min_output, scaling the other
 *        to keep the ratio.
 *
 * A floor on one wheel alone would change the commanded curvature; scaling both
 * keeps the arc the controller asked for and only makes it faster. Both sides
 * must already have the same sign and be non-zero for this to do anything.
 *
 * @param left_output  Left voltage, in and out.
 * @param right_output Right voltage, in and out.
 * @param min_output   The floor, volts - the stiction floor, i.e. the smallest
 *                     voltage that actually turns the wheels. The default and
 *                     its audit note live on the `min_output` global in
 *                     `config.hpp`; read the value there rather than assuming
 *                     one, and check it before enabling the floor.
 */
void scaleToMin(double& left_output, double& right_output, double min_output);

/**
 * @brief Cap the larger-magnitude side at @p max_output, scaling the other to
 *        keep the ratio.
 *
 * @param left_output  Left voltage, in and out.
 * @param right_output Right voltage, in and out.
 * @param max_output   The cap, volts.
 *
 * @note Equal magnitudes are capped, in every sign combination. Ties go to the
 *       left side. This used to be wrong: the negative-overflow branches tested
 *       one side strictly greater than the other where the positive ones used
 *       `>=`, so a tie fell through all four branches and nothing was capped -
 *       `(-13, -13)` came back unchanged, which is exactly straight backwards
 *       at full command.
 */
void scaleToMax(double& left_output, double& right_output, double max_output);
