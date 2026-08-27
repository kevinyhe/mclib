// mclib
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
 * @param min_output   The floor, volts. See the audit note on the `min_output`
 *                     global in `config.hpp`: the default is 10 V of a 12 V rail.
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
 * @warning Does not cap a pair whose magnitudes are **equal and negative**:
 *          every branch below tests one side strictly greater than the other,
 *          and the two that would catch the negative case use `>` where the
 *          positive ones use `>=`. `(-13, -13)` comes back unchanged. Left as
 *          it is deliberately - `tests/scaling_test.cpp` pins the behaviour
 *          with `knownBug()` - because the drive clamps to the rail anyway and
 *          changing it moves tuned autonomous numbers.
 */
void scaleToMax(double& left_output, double& right_output, double max_output);
