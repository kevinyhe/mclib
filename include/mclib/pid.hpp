// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include <cmath>
#include <limits>

namespace mclib {
namespace pid_detail {

/**
 * @brief Absolute value for either a plain double or a units Quantity.
 *
 * @details Written against the operators every PID value type has to provide
 * anyway (comparison against a zero-initialised value, unary minus) so the
 * controller body reads the same for `double` and for `QLength`.
 */
template <typename T>
constexpr T absValue(T value) {
  return value < T{} ? -value : value;
}

/**
 * @brief -1, 0 or +1, matching the sign() the controller has always used.
 */
template <typename T>
constexpr int signOf(T value) {
  if (value > T{}) {
    return 1;
  }
  if (value < T{}) {
    return -1;
  }
  return 0;
}

/**
 * @brief Bridge between a value type and the raw double behind it.
 *
 * @details Only needed to spell "infinity" in whatever type the caller chose.
 * A Quantity carries an explicit constructor, so it needs fromBase(); a double
 * is its own raw value.
 */
template <typename T>
struct RawAccess {
  static constexpr T from(double raw) { return T::fromBase(raw); }
  static constexpr double get(T value) { return value.raw(); }
};

/// @brief Specialisation for the dimensionless PID, where raw is the value.
template <>
struct RawAccess<double> {
  static constexpr double from(double raw) { return raw; }
  static constexpr double get(double value) { return value; }
};

/**
 * @brief The constructor defaults that only make sense for the dimensionless
 * controller.
 *
 * @details PID's historical defaults are bare numbers - a small tolerance of 1,
 * a big one of 3, an integral clamp of 500 - and they mean whatever the call
 * site's units happen to be. There is no honest way to carry that over to a
 * typed instantiation: "1" would become 1 metre for a QLength loop and 1 radian
 * (57 degrees) for a QAngle loop, which would latch `arrived` on the first tick
 * of almost any real motion and kill the integral at the same time.
 *
 * So a typed controller starts at zero instead: tolerances of 0 never latch and
 * never gate the integral, and an integral_max of 0 means unbounded. That is a
 * controller that visibly refuses to finish until the caller sets its
 * tolerances, rather than one that silently finishes immediately.
 */
template <typename T>
struct LegacyDefault {
  static constexpr T value(double) { return T{}; }
};

/// @brief The dimensionless controller keeps the historical numbers exactly.
template <>
struct LegacyDefault<double> {
  static constexpr double value(double legacy) { return legacy; }
};

}  // namespace pid_detail
}  // namespace mclib

/**
 * @brief Discrete PID controller with an optional arrival latch.
 *
 * @details Call setTarget() once, then update(input) every loop iteration; it
 * returns the output for that tick. The controller layers two behaviours on top
 * of the plain sum of P, I and D:
 *
 * **Arrival detection (`arrive`, default true).** Two independent settle
 * windows run at once. The "small" one needs |error| <= small_error_tolerance
 * held for small_error_duration milliseconds; the "big" one needs |error| <=
 * big_error_tolerance held for big_error_duration milliseconds. Both also
 * require the per-tick change in error to be <= derivative_tolerance (infinite
 * by default, so that check is off until you set it). Whichever window fills
 * first latches `arrived` true. The latch is sticky: only reset() clears it.
 * setArrive(false) disables arrival detection entirely, so `arrived` never
 * becomes true and update() keeps controlling forever.
 *
 * **Output after arrival (`hold_output`, default false).** With hold_output
 * false - the historical behaviour, kept so motion routines that spin until the
 * output reaches zero still terminate - update() returns a hard 0 on every tick
 * once `arrived` is latched, until reset(). That means a settled positional
 * mechanism has zero holding torque and sags under gravity. Call
 * setHoldOutput(true) and update() keeps computing P + I + D after arrival;
 * `arrived` and targetArrived() still latch exactly the same way, so a caller
 * can use arrival as a "motion finished" signal while the loop keeps holding
 * the setpoint.
 *
 * Three useful configurations:
 *  - setArrive(true), hold_output false (default): move, then stop dead. Right
 *    for a chassis motion that ends when the PID reports arrival.
 *  - setArrive(true), setHoldOutput(true): report arrival but keep driving.
 *    Right for a lift or an arm that would otherwise sag.
 *  - setArrive(false): never latch, never zero. Right for velocity control,
 *    where "arrived" is meaningless.
 *
 * @note What hold_output gives you inside the settle band is a P (plus D)
 * hold, not a zero-error hold. Arrival requires |error| <=
 * small_error_tolerance, and update() zeroes sum_error over exactly that same
 * band, so the integral is 0 on every held tick. A loaded arm settles wherever
 * kp * error balances the load, up to small_error_tolerance of steady droop.
 * To integrate that out, lower small_error_tolerance (which also tightens
 * arrival) or set it to 0 and end the motion on something other than
 * targetArrived().
 *
 * @warning The default small_error_tolerance of 1 also gates the integral:
 * update() zeroes sum_error whenever |error| <= small_error_tolerance (and
 * whenever sum_error's sign disagrees with the error's). For any loop that
 * lives inside an error of 1 - velocity control in rpm, or a position loop in
 * revolutions - that silently disables ki completely. Call
 * setSmallBigErrorTolerance(0, 0) (and setArrive(false), since the same
 * tolerances drive arrival) for those loops.
 *
 * @warning Those defaults of 1, 3 and 500 are bare numbers in whatever units
 * the call site uses, so they only exist for the dimensionless `PID` alias. A
 * typed instantiation starts with both tolerances and integral_max at 0 - it
 * never latches `arrived` and never clamps the integral until you call
 * setSmallBigErrorTolerance() and setIntegralMax() yourself. Carrying "1" over
 * would have meant 1 metre for a QLength loop and 1 radian for a QAngle one,
 * which would report arrival on the first tick of nearly every real motion.
 *
 * **Timing (`use_dt`, default false).** By default the D and I terms use raw
 * per-call deltas: `kd * (error - previous_error)` and `ki * sum(error)`, with
 * no division or multiplication by elapsed time at all. That is the historical
 * behaviour and every tuned constant in this repository was fitted against it,
 * so it stays the default. setUseDt(true) switches to true rates - see that
 * method's warning, because it changes what the gains mean.
 *
 * @tparam Input The process-value / setpoint type. `double` for the historical
 *   unit-agnostic controller; a units Quantity such as QLength or QAngle to
 *   have the compiler check the setpoint.
 * @tparam Output The type update() returns, e.g. `double` or QVoltage.
 *
 * @details The three gains all have type Gain = Output / Input. For the
 * dimensionless `PID` alias that is plain `double`, exactly as before. For
 * `PIDController<QLength, QVoltage>` it is volts per metre, so handing kp a
 * bare number or a voltage is a compile error rather than a silently wrong
 * loop.
 */
template <typename Input, typename Output>
class PIDController {
 public:
  /**
   * @brief The type of kp, ki and kd: output units per input unit.
   */
  using Gain = decltype(Output{} / Input{});

  /**
   * @brief Construct a PID with its three gains.
   */
  PIDController(Gain new_kp, Gain new_ki, Gain new_kd);

  /**
   * @brief Replace the three gains. Does not touch accumulated state.
   */
  void setCoefficient(Gain new_kp, Gain new_ki, Gain new_kd);

  /**
   * @brief Set the setpoint update() drives towards.
   *
   * @details Does not clear the arrival latch or the integral; call reset()
   * first when starting a new motion.
   */
  void setTarget(Input new_target);

  /**
   * @brief Set the two error bands used for arrival detection and for the
   * integral cutoff.
   *
   * @details See the class warning: small_error_tolerance also zeroes
   * sum_error, so passing (0, 0) is the way to keep ki alive on a loop whose
   * errors are small. Defaults are 1 and 3 for the dimensionless `PID` and 0
   * and 0 for a typed instantiation, which has to be told its own bands.
   */
  void setSmallBigErrorTolerance(Input new_small_error_tolerance,
                                 Input new_big_error_tolerance);

  /**
   * @brief Clamp the integral term's contribution to +/- this magnitude.
   *
   * @details 0 disables the clamp. Defaults to 500 for the dimensionless `PID`
   * and to 0 - unbounded - for a typed instantiation.
   */
  void setIntegralMax(Output new_integral_max);

  /**
   * @brief Only accumulate the integral once |error| is below this value.
   *
   * @details 0 disables the gate, so the integral accumulates at any error.
   */
  void setIntegralRange(Input new_integral_range);

  /**
   * @brief Drop the accumulated integral without touching anything else.
   */
  void clearSumError();

  /**
   * @brief Cap on the per-tick change in error allowed while a settle window
   * counts down.
   *
   * @details Infinite by default, which means the check never blocks arrival.
   * This is always the raw per-tick change in error, never a rate, so
   * setUseDt() does not move this threshold.
   */
  void setDerivativeTolerance(Input new_derivative_tolerance);

  /**
   * @brief How long, in milliseconds, each error band must hold before
   * `arrived` latches.
   */
  void setSmallBigErrorDuration(double new_small_error_duration,
                                double new_big_error_duration);

  /**
   * @brief Enable or disable arrival detection. Default true.
   *
   * @details With false, `arrived` never latches, so targetArrived() stays
   * false and update() never zeroes its output. This only stops new latches;
   * it does not clear one that already happened. Call reset() as well if the
   * controller may already have arrived.
   */
  void setArrive(bool new_arrive);

  /**
   * @brief Keep producing output after arrival instead of returning zero.
   *
   * @details Default false, which preserves the original behaviour: once
   * `arrived` latches, update() returns 0 until reset(). Pass true for any
   * mechanism that has to keep pushing against gravity after it gets there -
   * arrival is still detected and reported, the output simply keeps being
   * computed. See the class @note about what the held output actually is.
   */
  void setHoldOutput(bool new_hold_output);

  /// When false, arrival describes the current dwell, not a historical latch.
  /// Use with hold output when multiple axes must be settled simultaneously.
  void setLatchArrival(bool latch) { latch_arrival = latch; arrived = false; }

  /**
   * @brief Compute D and I as true rates against elapsed time. Default false.
   *
   * @warning **Flipping this on changes what your gains mean. Retune.** With
   * use_dt false (the default, and what every tuned constant in this
   * repository was fitted against) the terms are
   * `derivative = kd * (error - previous_error)` and
   * `integral = ki * sum(error)` - raw per-call deltas, correct only if the
   * caller happens to tick at a fixed period. With use_dt true they become
   * `derivative = kd * (error - previous_error) / dt_seconds` and
   * `integral = ki * sum(error * dt_seconds)`.
   *
   * At the usual 10 ms tick that multiplies the D term by 100 and divides the
   * I term by 100. Nothing warns you at compile time or at run time, so a
   * controller flipped to rate mode with its old gains will be violently
   * overdamped and have essentially no integral.
   *
   * @details dt is elapsed seconds as a plain number, so the gains keep the
   * same type in both modes and the class stays dimensionally consistent under
   * either setting. dt is measured from mclib::time::now() unless the caller
   * passes one to update(), and is always clamped to setDtRange() so a
   * double-call in the same millisecond cannot divide by zero and a long pause
   * cannot dump a huge integral step into the loop.
   *
   * The arrival criteria themselves are untouched: the settle windows and
   * derivative_tolerance still test the raw per-tick change in error, not a
   * rate, so the thresholds mean the same thing in both modes. That is not a
   * promise that a motion ends on the same tick - rate mode changes the output,
   * the output moves the robot, and the error trajectory moves with it.
   */
  void setUseDt(bool new_use_dt);

  /**
   * @brief Bounds applied to the measured or supplied dt in rate mode.
   *
   * @details Defaults are 1 ms and 100 ms. The lower bound is what makes
   * `dt == 0` safe: two update() calls inside the same millisecond, which the
   * scheduler really does produce, would otherwise divide the derivative by
   * zero. A dt that is zero, negative or NaN is raised to the lower bound. The
   * upper bound stops the first tick after a long pause - a disabled period, a
   * blocking call - from being treated as one enormous timestep.
   *
   * An inverted range (max below min) is collapsed to min rather than left to
   * clamp everything to the floor. Ignored entirely while use_dt is false.
   */
  void setDtRange(QTime new_min_dt, QTime new_max_dt);

  /**
   * @brief Whether rate mode is on.
   */
  bool usingDt() const;

  /**
   * @brief The dt, in seconds, the last update() actually used.
   *
   * @details 0 while use_dt is false, since no dt is measured or applied then.
   * Otherwise the clamped value, which is what makes a clamp visible to a
   * test or a tuning readout.
   */
  double getLastDt() const;

  /**
   * @brief Clear the arrival latch, the integral, and the settle timers.
   *
   * @details Call this before each new motion. The gains, tolerances,
   * durations, target, `arrive`, `hold_output`, `use_dt` and the dt range are
   * all left alone.
   */
  void reset();

  /**
   * @brief Whether a settle window has latched. Sticky until reset().
   */
  bool targetArrived();

  /**
   * @brief The ki gain. Not the integral term, which is ki * sum_error.
   */
  Gain getI();

  /**
   * @brief The value update() returned last tick.
   */
  Output getOutput();

  /**
   * @brief Run one control tick against the current process value and return
   * the output.
   *
   * @details In rate mode dt is measured from mclib::time::now() since the
   * previous update().
   */
  Output update(Input input);

  /**
   * @brief Run one control tick with a caller-supplied timestep.
   *
   * @details For a loop that already knows its period, or a test that needs
   * dt to be exact. @p dt is ignored while use_dt is false, because nothing in
   * the default numerics reads a timestep at all. It is clamped to
   * setDtRange() exactly like a measured dt.
   */
  Output update(Input input, QTime dt);

 private:
  static int sign(Input number);

  Output updateInternal(Input input, QTime dt, bool dt_supplied);

  Input target;
  bool arrived;
  bool arrive;
  bool hold_output;
  bool latch_arrival = true;
  Input small_error_tolerance;
  Input big_error_tolerance;
  double small_error_duration;
  double big_error_duration;
  double small_check_time;
  double big_check_time;
  bool first_time;
  Gain kp;
  Gain ki;
  Gain kd;
  Input integral_range;
  Output integral_max;
  Input derivative_tolerance;
  Input error_tolerance;
  Input current_error;
  Input previous_error;
  Input sum_error;
  Output proportional;
  Output integral;
  Output derivative;
  Output output;
  bool use_dt;
  double min_dt_seconds;
  double max_dt_seconds;
  double last_dt_seconds;
  QTime last_update_time;
  bool last_time_valid;
};

// ---------------------------------------------------------------------------
// Definitions. In the header because the class is a template; the
// PIDController<double, double> instantiation still lives in src/mclib/pid.cpp
// (see the extern template below), so pid.cpp stays a real translation unit
// with real symbols in the archive.
// ---------------------------------------------------------------------------

template <typename Input, typename Output>
PIDController<Input, Output>::PIDController(Gain new_kp, Gain new_ki, Gain new_kd)
    : target{}, arrived(false), arrive(true), hold_output(false),
      small_error_tolerance(mclib::pid_detail::LegacyDefault<Input>::value(1)),
      big_error_tolerance(mclib::pid_detail::LegacyDefault<Input>::value(3)),
      small_error_duration(100), big_error_duration(500), small_check_time(0),
      big_check_time(0), first_time(true), kp(new_kp), ki(new_ki), kd(new_kd),
      integral_range{},
      integral_max(mclib::pid_detail::LegacyDefault<Output>::value(500)),
      derivative_tolerance(mclib::pid_detail::RawAccess<Input>::from(
          std::numeric_limits<double>::infinity())),
      error_tolerance{}, current_error{}, previous_error{}, sum_error{},
      proportional{}, integral{}, derivative{}, output{}, use_dt(false),
      min_dt_seconds(0.001), max_dt_seconds(0.1), last_dt_seconds(0),
      last_update_time{}, last_time_valid(false) {}

template <typename Input, typename Output>
void PIDController<Input, Output>::setCoefficient(Gain new_kp, Gain new_ki, Gain new_kd) {
  kp = new_kp;
  ki = new_ki;
  kd = new_kd;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setTarget(Input new_target) {
  target = new_target;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setSmallBigErrorTolerance(Input new_small_error_tolerance,
                                                             Input new_big_error_tolerance) {
  small_error_tolerance = new_small_error_tolerance;
  big_error_tolerance = new_big_error_tolerance;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setIntegralMax(Output new_integral_max) {
  integral_max = new_integral_max;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setIntegralRange(Input new_integral_range) {
  integral_range = new_integral_range;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::clearSumError() {
  sum_error = Input{};
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setDerivativeTolerance(Input new_derivative_tolerance) {
  derivative_tolerance = new_derivative_tolerance;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setSmallBigErrorDuration(double new_small_error_duration,
                                                            double new_big_error_duration) {
  small_error_duration = new_small_error_duration;
  big_error_duration = new_big_error_duration;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setArrive(bool new_arrive) {
  arrive = new_arrive;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setHoldOutput(bool new_hold_output) {
  hold_output = new_hold_output;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setUseDt(bool new_use_dt) {
  use_dt = new_use_dt;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::setDtRange(QTime new_min_dt, QTime new_max_dt) {
  min_dt_seconds = new_min_dt.raw();
  max_dt_seconds = new_max_dt.raw();
  // An inverted range would otherwise clamp every dt to the floor and never
  // look at the ceiling at all, which reads as a working range that is silently
  // 100x wrong. Collapse it to a single value instead.
  if (max_dt_seconds < min_dt_seconds) {
    max_dt_seconds = min_dt_seconds;
  }
}

template <typename Input, typename Output>
bool PIDController<Input, Output>::usingDt() const {
  return use_dt;
}

template <typename Input, typename Output>
double PIDController<Input, Output>::getLastDt() const {
  return last_dt_seconds;
}

template <typename Input, typename Output>
void PIDController<Input, Output>::reset() {
  arrived = false;
  first_time = true;
  small_check_time = 0;
  big_check_time = 0;
  current_error = Input{};
  previous_error = Input{};
  sum_error = Input{};
  proportional = Output{};
  integral = Output{};
  derivative = Output{};
  output = Output{};
  last_dt_seconds = 0;
  last_update_time = QTime{};
  last_time_valid = false;
}

template <typename Input, typename Output>
bool PIDController<Input, Output>::targetArrived() {
  return arrived;
}

template <typename Input, typename Output>
typename PIDController<Input, Output>::Gain PIDController<Input, Output>::getI() {
  return ki;
}

template <typename Input, typename Output>
Output PIDController<Input, Output>::getOutput() {
  return output;
}

template <typename Input, typename Output>
int PIDController<Input, Output>::sign(Input number) {
  return mclib::pid_detail::signOf(number);
}

template <typename Input, typename Output>
Output PIDController<Input, Output>::update(Input input) {
  return updateInternal(input, QTime{}, false);
}

template <typename Input, typename Output>
Output PIDController<Input, Output>::update(Input input, QTime dt) {
  return updateInternal(input, dt, true);
}

template <typename Input, typename Output>
Output PIDController<Input, Output>::updateInternal(Input input, QTime dt, bool dt_supplied) {
  using mclib::pid_detail::absValue;

  current_error = target - input;
  if (!std::isfinite(mclib::pid_detail::RawAccess<Input>::get(current_error))) {
    reset();
    return Output{};
  }

  // Rate mode only. With use_dt false nothing below reads a timestep, the
  // clock is never sampled, and the arithmetic is bit-for-bit what it was
  // before this flag existed.
  double dt_seconds = 0.0;
  if (use_dt) {
    // The clock is stamped on every rate-mode tick, including one that was
    // handed its own dt. Stamping only the measured path would leave a stale
    // timestamp behind a run of update(input, dt) calls, and the next measured
    // tick would then charge the whole run to itself and hit the ceiling.
    const QTime stamp = mclib::time::now();
    QTime measured = dt;
    if (!dt_supplied) {
      // last_time_valid, not first_time: rate mode can be switched on partway
      // through a motion, and there is no usable previous stamp until the first
      // rate-mode tick has taken one.
      measured = last_time_valid ? stamp - last_update_time : QTime{};
    }
    last_update_time = stamp;
    last_time_valid = true;

    dt_seconds = measured.raw();
    // Written as a negated comparison so a zero, a negative dt from a clock
    // that went backwards, and a NaN all land on the lower bound.
    if (!(dt_seconds > min_dt_seconds)) {
      dt_seconds = min_dt_seconds;
    }
    if (dt_seconds > max_dt_seconds) {
      dt_seconds = max_dt_seconds;
    }
  }
  last_dt_seconds = dt_seconds;

  if (first_time) {
    first_time = false;
    previous_error = current_error;
    sum_error = Input{};
    small_check_time = mclib::time::millis();
    big_check_time = mclib::time::millis();
  }

  proportional = kp * current_error;
  const Input error_delta = current_error - previous_error;
  derivative = use_dt ? kd * error_delta / dt_seconds : kd * error_delta;
  previous_error = current_error;

  if (absValue(current_error) >= integral_range && integral_range != Input{}) {
    sum_error = Input{};
  } else {
    sum_error += use_dt ? current_error * dt_seconds : current_error;
    if (absValue(sum_error) * ki > integral_max && integral_max != Output{}) {
      sum_error = sign(sum_error) * integral_max / ki;
    }
  }

  if (sign(sum_error) != sign(current_error) ||
      absValue(current_error) <= small_error_tolerance) {
    sum_error = Input{};
  }

  integral = ki * sum_error;

  if (!latch_arrival) arrived = false;

  // Use raw error change for settling checks so kD tuning does not affect
  // arrival detection thresholds. This stays a raw per-tick delta in rate mode
  // too, so the thresholds mean the same thing under either setting.
  if (arrive && absValue(current_error) <= small_error_tolerance &&
      absValue(error_delta) <= derivative_tolerance) {
    if (mclib::time::millis() - small_check_time >= small_error_duration) {
      arrived = true;
    }
  } else {
    small_check_time = mclib::time::millis();
  }

  if (arrive && absValue(current_error) <= big_error_tolerance &&
      absValue(error_delta) <= derivative_tolerance) {
    if (mclib::time::millis() - big_check_time >= big_error_duration) {
      arrived = true;
    }
  } else {
    big_check_time = mclib::time::millis();
  }

  // Arrival latches by default; setLatchArrival(false) tracks current settling.
  // hold_output only decides whether the
  // loop keeps driving afterwards; it defaults to false so callers that wait
  // for the output to fall to zero to end a motion keep working.
  if (arrived && !hold_output) {
    output = Output{};
    return output;
  }

  output = proportional + integral + derivative;
  if (!std::isfinite(mclib::pid_detail::RawAccess<Output>::get(output))) {
    reset();
    return Output{};
  }
  return output;
}

/**
 * @brief The historical unit-agnostic PID: doubles in, doubles out.
 *
 * @details Every existing caller uses this. It is the dimensionless
 * instantiation of PIDController, and its arithmetic is bit-identical to the
 * pre-template class - `tests/pid_test.cpp` pins a recorded output trace to
 * prove it. `kp` still means whatever the call site's units imply, which is why
 * the same class serves driveTo (volts per inch) and turnToAngle (volts per
 * degree). Reach for PIDController<QLength, QVoltage> and friends to have the
 * compiler check that.
 */
using PID = PIDController<double, double>;

// Defined once, in src/mclib/pid.cpp. Every caller then links against that one
// copy rather than emitting its own, which keeps pid.cpp a real archive member
// that the linker has to pull in.
extern template class PIDController<double, double>;
