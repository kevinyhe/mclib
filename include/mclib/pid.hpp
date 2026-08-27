// mclib
#pragma once

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
 */
class PID {
public:
  /**
   * @brief Construct a PID with its three gains.
   */
  PID(double new_kp, double new_ki, double new_kd);

  /**
   * @brief Replace the three gains. Does not touch accumulated state.
   */
  void setCoefficient(double new_kp, double new_ki, double new_kd);

  /**
   * @brief Set the setpoint update() drives towards.
   *
   * @details Does not clear the arrival latch or the integral; call reset()
   * first when starting a new motion.
   */
  void setTarget(double new_target);

  /**
   * @brief Set the two error bands used for arrival detection and for the
   * integral cutoff.
   *
   * @details See the class warning: small_error_tolerance also zeroes
   * sum_error, so passing (0, 0) is the way to keep ki alive on a loop whose
   * errors are small.
   */
  void setSmallBigErrorTolerance(double new_small_error_tolerance,
                                 double new_big_error_tolerance);

  /**
   * @brief Clamp the integral term's contribution to +/- this magnitude.
   *
   * @details 0 disables the clamp.
   */
  void setIntegralMax(double new_integral_max);

  /**
   * @brief Only accumulate the integral once |error| is below this value.
   *
   * @details 0 disables the gate, so the integral accumulates at any error.
   */
  void setIntegralRange(double new_integral_range);

  /**
   * @brief Drop the accumulated integral without touching anything else.
   */
  void clearSumError();

  /**
   * @brief Cap on the per-tick change in error allowed while a settle window
   * counts down.
   *
   * @details Infinite by default, which means the check never blocks arrival.
   */
  void setDerivativeTolerance(double new_derivative_tolerance);

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

  /**
   * @brief Clear the arrival latch, the integral, and the settle timers.
   *
   * @details Call this before each new motion. The gains, tolerances,
   * durations, target, `arrive` and `hold_output` are all left alone.
   */
  void reset();

  /**
   * @brief Whether a settle window has latched. Sticky until reset().
   */
  bool targetArrived();

  /**
   * @brief The ki gain. Not the integral term, which is ki * sum_error.
   */
  double getI();

  /**
   * @brief The value update() returned last tick.
   */
  double getOutput();

  /**
   * @brief Run one control tick against the current process value and return
   * the output.
   */
  double update(double input);

private:
  static int sign(double number);

  double target;
  bool arrived;
  bool arrive;
  bool hold_output;
  double small_error_tolerance;
  double big_error_tolerance;
  double small_error_duration;
  double big_error_duration;
  double small_check_time;
  double big_check_time;
  bool first_time;
  double kp;
  double ki;
  double kd;
  double integral_range;
  double integral_max;
  double derivative_tolerance;
  double error_tolerance;
  double current_error;
  double previous_error;
  double sum_error;
  double proportional;
  double integral;
  double derivative;
  double output;
};
