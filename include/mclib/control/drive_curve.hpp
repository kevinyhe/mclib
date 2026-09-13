// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @file drive_curve.hpp
 * @brief Shaping a joystick reading before it reaches the drive.
 *
 * A raw stick is a poor drive command. It never quite rests at zero, the
 * robot needs a few percent of power before it moves at all, and a driver
 * wants fine control near the centre and full power at the edge. Four
 * corrections, applied in this order:
 *
 * 1. **Deadzone.** Readings inside `deadzone` become 0. What is left is
 *    rescaled so the output rises from 0 at the edge of the deadzone instead
 *    of jumping - `shapeDriveInput(deadzone)` is 0 and
 *    `shapeDriveInput(1)` is 1.
 * 2. **Curve.** The exponential curve used by 5225A and LemLib:
 *
 *        out = (e^(-g/10) + e^((|x| - 1) * g / 10) * (1 - e^(-g/10))) * x
 *
 *    `g` is `gain`. At `g = 0` this is `out = x`. For larger `g` the slope
 *    near the centre is `e^(-g/10)` - at `g = 10` a quarter-stick gives 0.17
 *    instead of 0.25 - while `out(1)` stays exactly 1. The curve is odd in
 *    `x` and strictly increasing, so it never reverses the driver.
 * 3. **Minimum output.** If `min_output` is non-zero, the curved value is
 *    rescaled from `[0, 1]` onto `[min_output, 1]`, so any stick outside the
 *    deadzone commands at least enough power to move. Zero stays zero.
 * 4. **Clamp** to `[-1, 1]`.
 *
 * `shapeDriveInput()` is those four steps with no state. `DriveCurve` adds
 * the fifth - a slew limit on how fast the output may change between
 * calls - which needs the previous output and so needs an object.
 *
 * Pure `double` arithmetic, no PROS: built into the host tests and covered by
 * `tests/drive_curve_test.cpp`.
 */

namespace mclib {
namespace control {

/// @brief Parameters for shapeDriveInput() and DriveCurve.
struct DriveCurveConfig {
  /// @brief |input| below this is 0. The rest is rescaled so the output is
  ///        continuous at the edge. Values outside [0, 1) are treated as 0.
  double deadzone = 0.05;
  /// @brief Exponential curve strength. 0 is linear; 10 is a common start.
  ///        Negative values are treated as 0.
  double gain = 0.0;
  /// @brief Smallest non-zero |output|, to overcome stiction. 0 = off.
  ///        Values outside [0, 1) are treated as 0.
  double min_output = 0.0;
  /// @brief Largest change in output per DriveCurve::apply() call. 0 = off.
  ///        Ignored by shapeDriveInput(), which has no memory.
  double slew_per_tick = 0.0;
};

/**
 * @brief Deadzone, curve, minimum output and clamp, with no state.
 *
 * @param input  Stick reading as a fraction, nominally -1..1. Anything
 *               outside is clamped first; NaN becomes 0.
 * @param config Shaping parameters. `slew_per_tick` is ignored here.
 * @return A drive fraction in [-1, 1] with the same sign as @p input.
 */
double shapeDriveInput(double input, const DriveCurveConfig& config);

/**
 * @brief shapeDriveInput() plus a slew limit.
 *
 * One instance per stick axis. Each apply() moves the output from where it
 * was last time toward the newly shaped value by at most
 * `config.slew_per_tick`. Call reset() when the drive command starts, so
 * a stick held before enable does not ramp from a stale value.
 */
class DriveCurve {
 public:
  explicit DriveCurve(DriveCurveConfig config = {});

  /// @brief Shape @p input and slew from the previous output toward it.
  double apply(double input);

  /// @brief Forget the previous output; the next apply() slews from 0.
  void reset();

  const DriveCurveConfig& config() const;
  void setConfig(const DriveCurveConfig& config);

 private:
  DriveCurveConfig m_config;
  double m_last_output = 0.0;
};

}  // namespace control
}  // namespace mclib
