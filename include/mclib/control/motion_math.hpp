// mclib
#pragma once

#include <cmath>

/**
 * @file motion_math.hpp
 * @brief The pure arithmetic of the motion routines, lifted out of the loops.
 *
 * `control/motion.cpp` includes `api.h`, so nothing in it can be exercised on
 * a host machine. Everything in this header is the part of those routines that
 * has no sensors, no motors and no clock in it: given the same numbers in, it
 * gives the same numbers out. It is compiled into the host test build
 * (`HOST_TEST_SRC`) and frozen by a golden table in `tests/motion_math_test.cpp`.
 *
 * ## Why these take `double` and not `QVoltage`
 *
 * The public motion API in `control/motion.hpp` is typed. This layer under it
 * is deliberately not, for two reasons:
 *
 * 1. It is the inner loop. Converting a value into SI base units and back out
 *    again is not a bit-exact round trip (`(24_in).in()` is
 *    23.999999999999996), so a typed inner loop would perturb every number the
 *    robot was tuned on. Unwrapping once at the entry of each routine and doing
 *    the arithmetic on the same doubles as before is what makes the migration
 *    provably behaviour-preserving.
 * 2. Some of it genuinely has no dimension to give. `max_slew_fwd` is volts per
 *    10 ms tick, `chase_power` is an empirical fudge factor, and
 *    `sqrt(chase_power * getRadius(...) * 9.8)` mixes volts, inches and m/s^2
 *    on purpose (see `config.hpp`). Those are documented raw-`double` escapes,
 *    not oversights.
 *
 * Every parameter below still says its unit in its doc comment, which is the
 * part that was missing before.
 */

namespace mclib {
namespace control {

/**
 * @brief The four tuned slew rates plus the two chaining flags, as motion sees them.
 *
 * Each rate is a **voltage step per nominal 10 ms tick** - `max_slew_accel_fwd`
 * and friends from `config.cpp`, which default to 1, i.e. 1 V per tick.
 */
struct SlewConfig {
  double accel_fwd = 0.0;  ///< Volts per tick, ramping up while driving forward.
  double decel_fwd = 0.0;  ///< Volts per tick, ramping down while driving forward.
  double accel_rev = 0.0;  ///< Volts per tick, ramping up while driving backward.
  double decel_rev = 0.0;  ///< Volts per tick, ramping down while driving backward.
  bool dir_change_start = true;  ///< The chain before this motion reverses direction.
  bool dir_change_end = true;    ///< The chain after this motion reverses direction.
  /**
   * @brief The "do not limit" rate used on chained segments, volts per tick.
   *
   * 24 is twice the voltage rail, so a limit of 24 V per 10 ms tick can never
   * bind. The literal 24 appears six times in the original motion.cpp; it is
   * named here rather than made tunable, because changing it would change
   * nothing until it dropped below 12.
   */
  double chain_slew = 24.0;
};

/// @brief The slew rates and min-speed gating a routine runs one motion with.
struct SlewPlan {
  double max_slew_fwd = 0.0;  ///< Volts per tick the output may rise by.
  double max_slew_rev = 0.0;  ///< Volts per tick the output may fall by.
  bool apply_min_speed_floor = false;  ///< Whether scaleToMin() runs at all.
};

/**
 * @brief Pick the slew rates and min-speed gating for one motion.
 *
 * The same 25-line block appeared verbatim in `driveTo`, `curveCircle`,
 * `moveToPoint` and `boomerang`. Reproduced here exactly, branch for branch.
 *
 * @param config              Tuned rates and the chaining flags.
 * @param drive_direction     +1 forward, -1 backward.
 * @param exit                True when the motion stops at the end. A motion
 *                            that stops keeps its tuned rates; a chained one
 *                            (`exit == false`) may drop the limiter entirely.
 * @param min_speed_requested True when the caller passed a non-negative
 *                            `min_speed`, i.e. asked for a floor explicitly.
 * @param min_speed_output    The resolved floor, volts. See minSpeedOutput().
 */
SlewPlan planSlew(const SlewConfig& config,
                  int drive_direction,
                  bool exit,
                  bool min_speed_requested,
                  double min_speed_output);

/**
 * @brief Resolve the caller's `min_speed` against the `min_output` default.
 *
 * @param min_speed  The caller's request, volts. Negative means "use the default".
 * @param min_output The `min_output` global, volts - the stiction floor,
 *                   1.5 V of a 12 V rail by default. It was 10 V, i.e. 83% of
 *                   the rail as a minimum; see the note in config.hpp.
 * @return The floor to apply, volts, never negative.
 */
double minSpeedOutput(double min_speed, double min_output);

/**
 * @brief Push a single-sided output out to at least the floor, keeping its sign.
 *
 * Used by the turning routines, which drive one scalar rather than a left/right
 * pair. The pair form is `scaleToMin()` in `control/scaling.hpp`.
 *
 * @param output           The PID output, volts.
 * @param min_speed_output The floor, volts. Zero or less disables the floor.
 */
double applyMinSpeedFloor(double output, double min_speed_output);

/// @brief Clamp @p value into [-@p max_output, +@p max_output]. Both volts.
double clampSymmetric(double value, double max_output);

/**
 * @brief One tick of a first-order rate limit on a voltage demand.
 *
 * @param desired      Where the controller wants to be, volts.
 * @param previous     Where the output was last tick, volts.
 * @param accel_limit  Largest permitted rise, volts per nominal 10 ms tick.
 * @param decel_limit  Largest permitted fall, volts per nominal 10 ms tick.
 * @param loop_dt_ms   Actual tick length, ms. Scales both limits; zero or less
 *                     is treated as the nominal tick.
 */
double applySlewLimit(double desired,
                      double previous,
                      double accel_limit,
                      double decel_limit,
                      double loop_dt_ms);

/**
 * @brief Rate-limit a left/right voltage pair against last tick's pair.
 *
 * @param left_output  In/out, volts.
 * @param right_output In/out, volts.
 * @param prev_left    Last tick's left output, volts.
 * @param prev_right   Last tick's right output, volts.
 * @param max_slew_fwd Largest permitted rise, volts per tick.
 * @param max_slew_rev Largest permitted fall, volts per tick.
 * @param limit_decel  False leaves the falling edge unlimited, which is what
 *                     `moveToPoint` does when it stops at the end so the PID
 *                     can brake into the target.
 */
void applySlewClamp(double& left_output,
                    double& right_output,
                    double prev_left,
                    double prev_right,
                    double max_slew_fwd,
                    double max_slew_rev,
                    bool limit_decel);

/**
 * @brief Trade forward drive away for heading correction, then split the pair.
 *
 * When `|drive| + |correction|` exceeds the voltage cap, the drive term is the
 * one that gives way, so a sharp turn still turns instead of saturating into a
 * straight line. Identical in `moveToPoint` and `boomerang`.
 *
 * The drive term is floored at zero: it gives up all of its forward voltage
 * and stops there, leaving a point turn. It used to be reduced without a
 * floor, so once `|correction| > max_output` it crossed zero and the robot
 * drove **backwards**. `boomerang()` passes the uncapped heading-PID output,
 * so that was reachable: drive 50 V, correction 37.5 V, cap 12 V gave a drive
 * term of -25.5 V and, after the mix and `scaleToMax`, roughly (+2.3, -12) -
 * a backwards point turn. It is now (+12, -12).
 *
 * @param left_output  In: the common drive term, volts. Out: the left voltage.
 * @param right_output Out only; whatever comes in is overwritten.
 * @param correction   Heading PID output, volts.
 * @param max_output   Voltage cap.
 * @param overturn     False disables the trade entirely.
 */
void applyOverturnAndMix(double& left_output,
                         double& right_output,
                         double correction,
                         double max_output,
                         bool overturn);

/**
 * @brief The decel rate the post-motion ramp uses to actually reach 0 V.
 *
 * The tuned rates default to 1 V per tick, which from 12 V is 120 ms; the
 * `max_output / 30` floor keeps the ramp inside its 500 ms safety cap even if
 * someone tunes the rates far slower.
 *
 * @return Volts per nominal tick.
 */
double exitDecel(double max_slew_fwd, double max_slew_rev, double max_output);

/**
 * @brief `boomerang()`'s slip-speed cap: how fast it may drive round an arc of
 *        the given radius before the wheels are expected to break traction.
 *
 * @param chase_power_gain The `chase_power` global. Unitless, empirical.
 * @param radius_in        Arc radius in inches, from `mclib::arcRadius()`.
 *                         **Signed** input is fine - only the magnitude
 *                         matters, since a left-hand arc slips at the same
 *                         speed as the mirror-image right-hand one.
 *                         `+/-infinity` means a straight line.
 * @return A cap in volts, to compare against the drive output. `+infinity`
 *         means "do not limit".
 *
 * ## Units
 *
 * `sqrt(chase_power * radius_in * 9.8)` mixes four systems: `chase_power` is a
 * unitless fudge factor, the radius is inches, 9.8 is g in m/s^2, and the
 * result is compared against volts. That is not fixed here and should not be:
 * the whole expression is an empirical constant that happens to be written
 * like a physical one, and the boomerang tuning was fitted to its shape. Only
 * the radius source changed - from the frame-transposed `getRadius()` to
 * `mclib::arcRadius()`.
 *
 * ## The two degenerate cases this exists to handle
 *
 * 1. **Straight line.** `arcRadius()` returns `+infinity` for a target dead
 *    ahead or dead behind, which is right: no turn, so nothing to slip. That
 *    flows through to `+infinity` here and no comparison against it is ever
 *    true, so the caller's clamp correctly does not fire.
 * 2. **`chase_power <= 0`.** `0 * infinity` is NaN, and *every* comparison
 *    against NaN is false, so on a straight line the limiter would silently
 *    vanish instead of clamping - the exact opposite of what a zero gain asks
 *    for. A non-positive gain means "allow no speed at all", and that is what
 *    it returns, at every radius including infinite. It stops the robot, which
 *    is loud; the NaN was silent.
 */
inline double slipSpeedLimit(double chase_power_gain, double radius_in) {
  if (!(chase_power_gain > 0.0)) {
    return 0.0;
  }
  return std::sqrt(chase_power_gain * std::fabs(radius_in) * 9.8);
}

}  // namespace control
}  // namespace mclib
