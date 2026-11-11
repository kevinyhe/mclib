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
 * keeps the arc the controller asked for and only makes it faster. It does
 * nothing unless the smaller side is non-zero and under the floor.
 *
 * @param left_output  Left voltage, in and out.
 * @param right_output Right voltage, in and out.
 * @param min_output   The floor, volts - the stiction floor, i.e. the smallest
 *                     voltage that actually turns the wheels. The default
 *                     lives on `MotionConfig::min_voltage` in
 *                     `control/motion_config.hpp`; read the value there rather
 *                     than assuming one, and check it before enabling the floor.
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

/**
 * @brief Shrink the base outputs so that, with @p correction added to the left
 *        and taken from the right, neither side passes @p max_output.
 *
 * The correction is kept whole and the base keeps its ratio. When the
 * correction alone passes @p max_output, the base goes to zero; cap the sum
 * with scaleToMax() afterward.
 *
 * @param left_base  Left voltage before the correction, in and out.
 * @param right_base Right voltage before the correction, in and out.
 * @param correction Volts added to the left side and taken from the right.
 * @param max_output The cap, volts.
 */
void fitBaseUnderCorrection(double& left_base, double& right_base, double correction, double max_output);
