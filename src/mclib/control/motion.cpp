// mclib
#include "mclib/control/motion.hpp"

#include "api.h"
#include "mclib/config.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion_math.hpp"
#include "mclib/control/scaling.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/math.hpp"
#include "mclib/pid.hpp"
#include "mclib/units/units.hpp"
#include "mclib/utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * @file motion.cpp
 * @brief Implementation of the typed motion API in motion.hpp.
 *
 * Each routine unwraps its typed parameters into `double` locals on its first
 * few lines and then runs the arithmetic it always ran, on the same doubles,
 * in the same order. That is deliberate: these loops are what this robot's
 * autonomous was tuned against, and converting a value into SI base units and
 * back out again is not a bit-exact round trip. Typing the API without typing
 * the inner loop is what makes the conversion provably behaviour-preserving.
 *
 * The pure parts of that arithmetic - slew planning, rate limiting, output
 * mixing - live in `control/motion_math.hpp`, which has no PROS dependency and
 * is exercised on the host by `tests/motion_math_test.cpp`.
 */
namespace {
using mclib::control::applyMinSpeedFloor;
using mclib::control::applyOverturnAndMix;
using mclib::control::applySlewClamp;
using mclib::control::applySlewLimit;
using mclib::control::clampSymmetric;
using mclib::control::exitDecel;
using mclib::control::minSpeedOutput;
using mclib::control::planSlew;
using mclib::control::slipSpeedLimit;
using mclib::control::SlewConfig;
using mclib::control::SlewPlan;

/**
 * @brief Encoder degrees to inches rolled, from the one drive geometry.
 *
 * Replaces the seven copies of `deg * wheel_distance_in / 360.0` that used to
 * be open-coded in the loops below, each remembering the `/ 360.0` for itself
 * and each reading a mutable global that no longer exists.
 * `mclib::config::robot_drive_geometry` is now the only description of this
 * drive base; `DriveGeometry::encoderToDistance()` is the same formula, just
 * written once and typed.
 *
 * It is not bit-identical to what it replaces: the typed path goes through
 * metres, which costs up to 2 ulp - 4.6e-16 relative, or 4e-15 in over a ten
 * inch drive. Pinned by tests/geometry_test.cpp.
 */
double encoderDegreesToInches(double deg) {
  return mclib::config::robot_drive_geometry
      .encoderToDistance(deg * mclib::units::degree)
      .in();
}

/// @brief Half the track width in inches - the radius each wheel turns about
///        when the robot spins in place. Was `distance_between_wheels / 2`.
double halfTrackWidthIn() {
  return mclib::config::robot_drive_geometry.turnRadius().in();
}

/// @brief The tuned slew rates and chaining flags, as they stand in config.cpp.
SlewConfig slewConfig() {
  const mclib::config::SlewRates rates = mclib::config::slewRates();
  SlewConfig config{};
  config.accel_fwd = rates.accel_fwd;
  config.decel_fwd = rates.decel_fwd;
  config.accel_rev = rates.accel_rev;
  config.decel_rev = rates.decel_rev;
  config.dir_change_start = dir_change_start;
  config.dir_change_end = dir_change_end;
  return config;
}

/**
 * @brief Send a left/right voltage pair, in volts, to the drive.
 *
 * The one place the inner loop's bare doubles meet the typed actuator
 * boundary. `driveChassis()` takes QVoltage; everything above it here is
 * volts-as-double by design (see the file comment).
 */
void driveVolts(double left, double right) {
  driveChassis(left * mclib::units::volt, right * mclib::units::volt);
}

void holdLeftSide() {
  left_chassis.setBrakeMode(mclib::device::BrakeMode::Hold);
  left_chassis.brake();
}

void holdRightSide() {
  right_chassis.setBrakeMode(mclib::device::BrakeMode::Hold);
  right_chassis.brake();
}

/// @brief The shared state that replaced the bare globals in state.hpp.
mclib::control::RobotState& state() {
  return mclib::control::robotState();
}

/**
 * @brief True once someone has asked the running motion routine to stop.
 *
 * Every loop below tests this next to its timeout. That is what lets
 * AsyncControlCommand cancel a motion by asking instead of by calling
 * pros::Task::remove() on a task that might be mid-store.
 */
bool cancelled() {
  return mclib::control::cancelRequested(
      mclib::control::CancelToken::Motion);
}

/**
 * @brief The heading to hold after a routine that aimed at @p commanded_deg.
 *
 * A routine that ran to completion reached its target, so that is the heading
 * to hold. A cancelled one was stopped short, and publishing the commanded
 * angle would have correctHeading() actively drive toward a heading the robot
 * never got to. Report where it actually is.
 */
double settledHeadingDeg(double commanded_deg);

/**
 * @brief True once someone has asked correctHeading() to stop.
 *
 * A separate token from cancelled(). correctHeading() runs *alongside* the
 * motions - it gates on isTurning() so it can - so cancelling a motion must
 * not take the heading hold down with it.
 */
bool headingCorrectionCancelled() {
  return mclib::control::cancelRequested(
      mclib::control::CancelToken::HeadingCorrection);
}

double settledHeadingDeg(double commanded_deg) {
  return cancelled() ? getInertialHeading() : commanded_deg;
}
}  // namespace
void turnToAngle(QAngle turn_angle_target, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  // Unwrap once, here. Everything below is the arithmetic this routine always
  // ran, on the same doubles; see the file comment.
  double turn_angle = turn_angle_target.deg();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // Brake mode helps dissipate momentum near the setpoint and reduces hunting.
  stopChassis(mclib::device::BrakeMode::Brake);
  state().setTurning(true);
  const double threshold = 1;
  PID pid(turn_kp, turn_ki, turn_kd);

  turn_angle = normalizeTarget(turn_angle);
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);
  pid.setIntegralRange(3);
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  const double start_time = pros::millis();
  double output = 0;
  const double min_speed_output = minSpeedOutput(min_speed, min_output);

  if (!exit && getInertialHeading() < turn_angle)
  {
    // early exit path drives until the inertial pose passes the commanded heading allowing chained moves
    while (getInertialHeading() < turn_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      output = pid.update(getInertialHeading());
      // keep a minimum turning speed so the robot doesn't stall mid-chain
      output = applyMinSpeedFloor(output, min_speed_output);
      // clamp keeps pid output within symmetric voltage rails so differential
      // drive math stays bounded
      output = clampSymmetric(output, max_output);
      driveVolts(output, -output);
      pros::delay(10);
    }
  }
  else if (!exit && getInertialHeading() > turn_angle)
  {
    while (getInertialHeading() > turn_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      output = pid.update(getInertialHeading());
      output = applyMinSpeedFloor(output, min_speed_output);
      output = clampSymmetric(output, max_output);
      driveVolts(output, -output);
      pros::delay(10);
    }
  }
  else
  {
    while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      output = clampSymmetric(pid.update(getInertialHeading()), max_output);
      driveVolts(output, -output);
      pros::delay(10);
    }
  }

  if (exit)
  {
    stopChassis(mclib::device::BrakeMode::Hold);
  }
  state().setCorrectAngleDeg(settledHeadingDeg(turn_angle));
  state().setTurning(false);
}

void driveTo(QLength distance, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  double distance_in = distance.in();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // Store initial encoder values
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  const double min_speed_output = minSpeedOutput(min_speed, min_output);
  const SlewPlan slew =
      planSlew(slewConfig(), drive_direction, exit, min_speed >= 0, min_speed_output);
  const double max_slew_fwd = slew.max_slew_fwd;
  const double max_slew_rev = slew.max_slew_rev;
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  // flip target so pid always works with a positive distance scalar regardless of command direction
  distance_in = distance_in * drive_direction;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Configure PID controllers
  pid_distance.setTarget(distance_in);
  pid_distance.setIntegralMax(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(state().correctAngleDeg()));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_distance = 0, current_angle = 0;
  // Local mirror of the shared slew baseline: read once here, published once
  // at the end, so the loop below never touches the lock.
  double prev_left_output = state().prevLeftOutput();
  double prev_right_output = state().prevRightOutput();

  // Main PID loop for driving straight
  while ((((!pid_distance.targetArrived()) && pros::millis() - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && pros::millis() - start_time <= time_limit_msec)) && !cancelled())
  {
    // integrate wheel travel by converting encoder degrees into linear inches and averaging both treads
    current_distance = (fabs(encoderDegreesToInches(getLeftRotationDegree() - start_left)) + fabs(encoderDegreesToInches(getRightRotationDegree() - start_right))) / 2;
    current_angle = getInertialHeading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = pid_heading.update(current_angle);

    // Minimum Output Check
    if (apply_min_speed_floor && min_speed_output > 0)
    {
      scaleToMin(left_output, right_output, min_speed_output);
    }
    if (!exit)
    {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    // heading pid adds and subtracts to create differential voltage that cancels yaw error
    left_output += correction_output;
    right_output -= correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    // slew limits act as a discrete first order filter on voltage demand to reduce jerk
    applySlewClamp(left_output,
                   right_output,
                   prev_left_output,
                   prev_right_output,
                   max_slew_fwd,
                   max_slew_rev,
                   true);
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveVolts(left_output, right_output);
    pros::delay(10);
  }
  if (exit)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    // within the timeout (stops from max_output in ~300 ms).
    const double exit_decel = exitDecel(max_slew_fwd, max_slew_rev, max_output);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && !cancelled())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      driveVolts(prev_left_output, prev_right_output);
      pros::delay(10);
    }
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(mclib::device::BrakeMode::Hold);
  }
  // Publish the slew baseline so the next motion picks up where this one left
  // off (zero, on the exit path above).
  state().setPrevOutputs(prev_left_output, prev_right_output);
  state().setTurning(false);
}

void curveCircle(QAngle result_angle_target, QLength center_radius, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage, bool reverse)
{
  double result_angle_deg = result_angle_target.deg();
  // Signed: the sign picks the curve direction, the magnitude is the radius.
  const double center_radius_in = center_radius.in();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // Store initial encoder values for both sides
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // Normalize the target angle to be within +/-180 degrees of the current heading
  result_angle_deg = normalizeTarget(result_angle_deg);
  // convert delta heading into radians so arc length math can use radius times angle
  const double entry_angle_deg = state().correctAngleDeg();
  result_angle = (result_angle_deg - entry_angle_deg) * 3.14159265359 / 180;

  // Calculate arc lengths for inner and outer wheels
  // inner and outer tread travel differ by wheel base offset so compute each arc explicitly
  const double half_track_width_in = halfTrackWidthIn();
  in_arc = fabs((fabs(center_radius_in) - half_track_width_in) * result_angle);
  out_arc = fabs((fabs(center_radius_in) + half_track_width_in) * result_angle);
  ratio = in_arc / out_arc;

  stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);
  double threshold = 0.5;

  // Determine curve and drive direction
  int curve_direction = center_radius_in > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - entry_angle_deg) > 0) || (curve_direction == -1 && (result_angle_deg - entry_angle_deg) < 0))
  {
    drive_direction = 1;
  }
  else
  {
    drive_direction = -1;
  }

  if (reverse)
  {
    drive_direction = -1;
  }

  // Slew rate and minimum speed logic for chaining. curveCircle never applies
  // the rate limit itself - it only reads apply_min_speed_floor out of the
  // plan - but the gating depends on the whole block, so it is computed whole.
  const double min_speed_output = minSpeedOutput(min_speed, min_output);
  const SlewPlan slew =
      planSlew(slewConfig(), drive_direction, exit, min_speed >= 0, min_speed_output);
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  // Initialize PID controllers for arc distance and heading correction
  PID pid_out = PID(distance_kp, distance_ki, distance_kd);
  PID pid_turn = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_out.setTarget(out_arc);
  pid_out.setIntegralMax(0);
  pid_out.setIntegralRange(5);
  pid_out.setSmallBigErrorTolerance(0.3, 0.9);
  pid_out.setSmallBigErrorDuration(50, 250);
  pid_out.setDerivativeTolerance(threshold * 4.5);

  pid_turn.setTarget(0);
  pid_turn.setIntegralMax(0);
  pid_turn.setIntegralRange(1);
  pid_turn.setSmallBigErrorTolerance(0, 0);
  pid_turn.setSmallBigErrorDuration(0, 0);
  pid_turn.setDerivativeTolerance(0);
  pid_turn.setArrive(false);

  double start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;

  // Main control loop for each curve/exit configuration
  if (curve_direction == -1 && exit == true)
  {
    // Left curve, stop at end
    while (!pid_out.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_right = fabs(encoderDegreesToInches(getRightRotationDegree() - start_right));
      // interpolate instantaneous heading by mapping right wheel progress onto desired arc fraction
      real_angle = current_right / out_arc * (result_angle_deg - entry_angle_deg) + entry_angle_deg;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      // Enforce minimum output if chaining
      if (apply_min_speed_floor && min_speed_output > 0)
      {
        scaleToMin(left_output, right_output, min_speed_output);
      }

      // Apply heading correction
      left_output += correction_output;
      right_output -= correction_output;

      // Enforce maximum output
      scaleToMax(left_output, right_output, max_output);

      driveVolts(left_output, right_output);
      pros::delay(10);
    }
  }
  else if (curve_direction == 1 && exit == true)
  {
    // Right curve, stop at end
    while (!pid_out.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_left = fabs(encoderDegreesToInches(getLeftRotationDegree() - start_left));
      real_angle = current_left / out_arc * (result_angle_deg - entry_angle_deg) + entry_angle_deg;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if (apply_min_speed_floor && min_speed_output > 0)
      {
        scaleToMin(left_output, right_output, min_speed_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveVolts(left_output, right_output);
      pros::delay(10);
    }
  }
  else if (curve_direction == -1 && exit == false)
  {
    // Left curve, chaining (do not stop at end)
    while (current_right < out_arc && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_right = fabs(encoderDegreesToInches(getRightRotationDegree() - start_right));
      real_angle = current_right / out_arc * (result_angle_deg - entry_angle_deg) + entry_angle_deg;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if (apply_min_speed_floor && min_speed_output > 0)
      {
        scaleToMin(left_output, right_output, min_speed_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveVolts(left_output, right_output);
      pros::delay(10);
    }
  }
  else
  {
    // Right curve, chaining (do not stop at end)
    while (current_left < out_arc && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_left = fabs(encoderDegreesToInches(getLeftRotationDegree() - start_left));
      real_angle = current_left / out_arc * (result_angle_deg - entry_angle_deg) + entry_angle_deg;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if (apply_min_speed_floor && min_speed_output > 0)
      {
        scaleToMin(left_output, right_output, min_speed_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveVolts(left_output, right_output);
      pros::delay(10);
    }
  }
  // Stop the chassis if required
  if (exit == true)
  {
    stopChassis(mclib::device::BrakeMode::Hold);
  }
  // Update the global heading
  state().setCorrectAngleDeg(settledHeadingDeg(result_angle_deg));
  state().setTurning(false);
}

void curveCircleReverse(QAngle result_angle, QLength center_radius, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  curveCircle(result_angle, center_radius, time_limit, exit, max_voltage, min_voltage, true);
}

void swing(QAngle swing_angle_target, double drive_direction, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  double swing_angle = swing_angle_target.deg();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before starting swing
  state().setTurning(true);                      // Set turning state
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd); // Initialize PID for turning

  swing_angle = normalizeTarget(swing_angle); // Normalize target angle
  pid.setTarget(swing_angle);                 // Set PID target
  pid.setIntegralMax(0);
  pid.setIntegralRange(5);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Start the PID loop
  double start_time = pros::millis();
  double output;
  const double min_speed_output = minSpeedOutput(min_speed, min_output);
  const double entry_angle_deg = state().correctAngleDeg();
  double current_heading = entry_angle_deg;
  int choice = 1;

  // choice encodes which tread stays locked so swing math can reuse one code path per quadrant
  if (swing_angle - entry_angle_deg < 0 && drive_direction == 1)
  {
    choice = 1;
  }
  else if (swing_angle - entry_angle_deg > 0 && drive_direction == 1)
  {
    choice = 2;
  }
  else if (swing_angle - entry_angle_deg < 0 && drive_direction == -1)
  {
    choice = 3;
  }
  else
  {
    choice = 4;
  }

  // Swing logic for each case, chaining (exit == false)
  if (choice == 1 && exit == false)
  {
    // Swing left, forward
    while (current_heading > swing_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Clamp output
      output = applyMinSpeedFloor(output, min_speed_output);
      output = clampSymmetric(output, max_output);

      holdLeftSide(); // Hold left, swing right
      right_chassis.setVoltage(output * drive_direction);
      pros::delay(10);
    }
  }
  else if (choice == 2 && exit == false)
  {
    // Swing right, forward
    while (current_heading < swing_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Clamp output
      output = applyMinSpeedFloor(output, min_speed_output);
      output = clampSymmetric(output, max_output);

      left_chassis.setVoltage(output * drive_direction);
      holdRightSide(); // Hold right, swing left
      pros::delay(10);
    }
  }
  else if (choice == 3 && exit == false)
  {
    // Swing left, backward
    while (current_heading > swing_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Clamp output
      output = applyMinSpeedFloor(output, min_speed_output);
      output = clampSymmetric(output, max_output);

      left_chassis.setVoltage(output * drive_direction);
      holdRightSide();
      pros::delay(10);
    }
  }
  else
  {
    // Swing right, backward
    while (current_heading < swing_angle && pros::millis() - start_time <= time_limit_msec && exit == false && !cancelled())
    {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Clamp output
      output = applyMinSpeedFloor(output, min_speed_output);
      output = clampSymmetric(output, max_output);

      holdLeftSide();
      right_chassis.setVoltage(output * drive_direction);
      pros::delay(10);
    }
  }

  // PID loop for exit == true (stop at end)
  while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && exit == true && !cancelled())
  {
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Clamp output
    output = clampSymmetric(output, max_output);

    // Apply output to correct side based on swing direction
    switch (choice)
    {
    case 1:
      holdLeftSide();
      right_chassis.setVoltage(-output * drive_direction);
      break;
    case 2:
      left_chassis.setVoltage(output * drive_direction);
      holdRightSide();
      break;
    case 3:
      left_chassis.setVoltage(-output * drive_direction);
      holdRightSide();
      break;
    case 4:
      holdLeftSide();
      right_chassis.setVoltage(output * drive_direction);
      break;
    }
    pros::delay(10);
  }
  if (exit == true)
  {
    stopChassis(mclib::device::BrakeMode::Hold); // Stop chassis at end if required
  }
  state().setCorrectAngleDeg(settledHeadingDeg(swing_angle)); // Update shared heading
  state().setTurning(false);          // Reset turning state
}

void correctHeading()
{
  double output = 0;
  PID pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid.setTarget(state().correctAngleDeg()); // Set PID target to current heading
  pid.setIntegralRange(fabs(state().correctAngleDeg()) / 2.5);

  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);

  // feed equal magnitude opposite sign voltages
  // this cancels drift while driving straight
  while (heading_correction && !headingCorrectionCancelled())
  {
    pid.setTarget(state().correctAngleDeg());
    if (!state().isTurning())
    {
      output = pid.update(getInertialHeading());
      driveVolts(output, -output); // Apply correction to chassis
    }
    pros::delay(10);
  }
}

void wallReset(QLength reset_x, QLength reset_y, QAngle reset_heading,
               QVoltage drive_power, QTime time_limit,
               QCurrent current_threshold, QAngularVelocity velocity_threshold)
{
  const double reset_x_in = reset_x.in();
  const double reset_y_in = reset_y.in();
  // NaN when the caller passed keep_current_heading. Kept as a double so the
  // isnan() test below is the same test it always was.
  const double reset_heading_deg = reset_heading.deg();
  const double drive_power_volts = drive_power.volts();
  const double time_limit_msec = time_limit.ms();
  // The V5 motor reports current in mA and speed in RPM; both thresholds are
  // compared against those raw readings.
  const double current_threshold_ma = current_threshold.mA();
  const double velocity_threshold_rpm = velocity_threshold.rpm();

  uint32_t start_time = pros::millis();
  int stall_count = 0;
  constexpr int stall_cycles_needed = 5; // 50 ms of sustained stall

  // Drive into the wall
  driveVolts(drive_power_volts, drive_power_volts);

  // Give the robot a moment to start moving before checking stall
  pros::delay(200);

  while (pros::millis() - start_time <= time_limit_msec && !cancelled())
  {
    // Average current across all motors on both sides (mA)
    auto left_currents = left_chassis.getCurrentDraws();
    auto right_currents = right_chassis.getCurrentDraws();
    double total_current = 0;
    int motor_count = 0;
    for (auto c : left_currents)
    {
      total_current += c;
      motor_count++;
    }
    for (auto c : right_currents)
    {
      total_current += c;
      motor_count++;
    }
    double avg_current = (motor_count > 0) ? total_current / motor_count : 0;

    // Average velocity across all motors (RPM)
    auto left_vel = left_chassis.getActualVelocities();
    auto right_vel = right_chassis.getActualVelocities();
    double total_vel = 0;
    int vel_count = 0;
    for (auto v : left_vel)
    {
      total_vel += fabs(v);
      vel_count++;
    }
    for (auto v : right_vel)
    {
      total_vel += fabs(v);
      vel_count++;
    }
    double avg_vel = (vel_count > 0) ? total_vel / vel_count : 0;

    // Stall detection: high current, low velocity
    if (avg_current > current_threshold_ma && avg_vel < velocity_threshold_rpm)
    {
      stall_count++;
    }
    else
    {
      stall_count = 0;
    }

    if (stall_count >= stall_cycles_needed)
    {
      break; // Wall detected
    }

    pros::delay(10);
  }

  // Stop motors
  stopChassis(mclib::device::BrakeMode::Brake);

  // Reset heading first if a valid value was provided, so the pose below is
  // built from the IMU value we are actually going to keep.
  if (!std::isnan(reset_heading_deg))
  {
    inertial_sensor.setRotationDeg(reset_heading_deg);
    state().setCorrectAngleDeg(reset_heading_deg);
  }
  else
  {
    state().setCorrectAngleDeg(getInertialHeading());
  }

  // Reset position to known coordinates. This goes through the odometry, not
  // just the pose, so the next odometry tick re-seeds its encoder baseline
  // instead of integrating a delta across the teleport. A non-finite heading
  // leaves the odometry's heading alone.
  mclib::control::resetOdometry(
      mclib::Pose2D{reset_x_in, reset_y_in, degToRad(getInertialHeading())});
}

void turnToPoint(QLength x, QLength y, int direction, QTime time_limit, QVoltage min_voltage)
{
  const double x_in = x.in();
  const double y_in = y.in();
  const double time_limit_msec = time_limit.ms();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before turning
  state().setTurning(true);                      // Set turning state
  double threshold = 1, add = 0;
  if (direction == -1)
  {
    add = 180; // Add 180 degrees if turning to face backward
  }
  // One locked read: x and y always come from the same odometry tick.
  mclib::Pose2D pose = state().pose();
  // Calculate target angle using atan2 and normalize
  double turn_angle = normalizeTarget(radToDeg(atan2(x_in - pose.x, y_in - pose.y))) + add;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  pid.setTarget(turn_angle); // Set PID target
  pid.setIntegralMax(0);
  pid.setIntegralRange(3);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);

  double start_time = pros::millis();
  const double min_speed_output = minSpeedOutput(min_speed, min_output);
  while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
  {
    pose = state().pose();
    pid.setTarget(normalizeTarget(radToDeg(atan2(x_in - pose.x, y_in - pose.y))) + add);
    double output = pid.update(getInertialHeading());
    output = applyMinSpeedFloor(output, min_speed_output);
    driveVolts(output, -output);
    pros::delay(10);
  }
  stopChassis(mclib::device::BrakeMode::Hold); // Stop at end
  state().setCorrectAngleDeg(getInertialHeading());  // Update shared heading
  state().setTurning(false);                    // Reset turning state
}

void moveToPoint(QLength x, QLength y, int dir, QTime time_limit, bool exit, QVoltage max_voltage, bool overturn, QVoltage min_voltage)
{
  const double x_in = x.in();
  const double y_in = y.in();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before moving
  state().setTurning(true);                      // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  const double min_speed_output = minSpeedOutput(min_speed, min_output);
  const SlewPlan slew =
      planSlew(slewConfig(), dir, exit, min_speed >= 0, min_speed_output);
  const double max_slew_fwd = slew.max_slew_fwd;
  const double max_slew_rev = slew.max_slew_rev;
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // One locked read: x_in and y_in always come from the same odometry tick.
  mclib::Pose2D pose = state().pose();
  // Set PID targets for distance and heading
  pid_distance.setTarget(hypot(x_in - pose.x, y_in - pose.y));
  pid_distance.setIntegralMax(0);
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x_in - pose.x, y_in - pose.y)) + add));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);

  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis
  double start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0;
  // Local mirror of the shared slew baseline; see driveTo().
  //
  // This used to be a pair of locals initialised to zero, so moveToPoint()
  // slew-limited from 0 V on every call while driveTo() and boomerang()
  // carried the baseline across motions - three routines, two contracts, and
  // the odd one out was the one that shadowed names that had been globals.
  // All three read and publish the shared baseline now.
  //
  // The consequence, stated plainly because it is a behaviour change and not a
  // type change: chained straight after a motion that left the drive at full
  // voltage in the OTHER direction, the accel limit now has to walk the output
  // across zero at max_slew_fwd per tick before this motion moves the right
  // way - about 180 ms at the tuned 1 V/tick. boomerang() has always behaved
  // this way; moveToPoint() now does too. Reversing direction between motions
  // is what dir_change_start / dir_change_end describe, and planSlew() only
  // consults them on a chained (`exit == false`) motion, so a reversal into an
  // `exit == true` moveToPoint() gets the slow crossing. Give the reversing
  // motion `.withoutExit()`, or let it stop first.
  double prev_left_output = state().prevLeftOutput();
  double prev_right_output = state().prevRightOutput();
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0;
  bool ch = true;

  // Main PID loop for moving to point
  while (pros::millis() - start_time <= time_limit_msec && !cancelled())
  {
    // Continuously update targets as robot moves
    pose = state().pose();
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x_in - pose.x, y_in - pose.y)) + add));
    pid_distance.setTarget(hypot(x_in - pose.x, y_in - pose.y));
    current_angle = getInertialHeading();
    // Calculate drive output based on heading and distance
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x_in - pose.x, y_in - pose.y) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((pose.y - y_in) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (pose.x - x_in) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if (perpendicular_line && !prev_perpendicular_line)
    {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Only apply heading correction if far from target
    if (hypot(x_in - pose.x, y_in - pose.y) > 8 && ch == true)
    {
      correction_output = pid_heading.update(current_angle);
      // Cap correction so it can't overwhelm the forward drive and cause a pivot
      double max_correction = fabs(left_output) * 0.75;
      if (fabs(correction_output) > max_correction)
      {
        correction_output = (correction_output > 0) ? max_correction : -max_correction;
      }
    }
    else
    {
      correction_output = 0;
      ch = false;
    }

    // Minimum Output Check
    if (apply_min_speed_floor && min_speed_output > 0)
    {
      scaleToMin(left_output, right_output, min_speed_output);
    }

    // Overturn logic for sharp turns
    applyOverturnAndMix(left_output, right_output, correction_output, max_output, overturn);

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    // When exit=true, skip decel slew so PID can brake before the target.
    // Only limit acceleration to prevent wheel slip on startup.
    applySlewClamp(left_output,
                   right_output,
                   prev_left_output,
                   prev_right_output,
                   max_slew_fwd,
                   max_slew_rev,
                   !exit);
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveVolts(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  if (exit == true)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    const double exit_decel = exitDecel(max_slew_fwd, max_slew_rev, max_output);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && !cancelled())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      driveVolts(prev_left_output, prev_right_output);
      pros::delay(10);
    }
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(mclib::device::BrakeMode::Hold); // Stop at end if required
  }
  // Publish the slew baseline so the next motion picks up where this one left
  // off (zero, on the exit path above). Same contract as driveTo/boomerang.
  state().setPrevOutputs(prev_left_output, prev_right_output);
  state().setCorrectAngleDeg(getInertialHeading()); // Update shared heading
  state().setTurning(false);                   // Reset turning state
}

void boomerang(QLength x, QLength y, int dir, QAngle final_heading, double dlead, QTime time_limit, bool exit, QVoltage max_voltage, bool overturn, QVoltage min_voltage)
{
  const double x_in = x.in();
  const double y_in = y.in();
  // The final heading, degrees. `dlead` is genuinely dimensionless.
  const double a = final_heading.deg();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before moving
  state().setTurning(true);                      // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  const double min_speed_output = minSpeedOutput(min_speed, min_output);
  const SlewPlan slew =
      planSlew(slewConfig(), dir, exit, min_speed >= 0, min_speed_output);
  const double max_slew_fwd = slew.max_slew_fwd;
  const double max_slew_rev = slew.max_slew_rev;
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // One locked read: x_in and y_in always come from the same odometry tick.
  mclib::Pose2D pose = state().pose();
  // Compute initial carrot so the PID starts with a real nonzero target
  double init_hyp = hypot(pose.x - x_in, pose.y - y_in);
  double init_carrot_x = x_in - init_hyp * sin(degToRad(a + add)) * dlead;
  double init_carrot_y = y_in - init_hyp * cos(degToRad(a + add)) * dlead;
  pid_distance.setTarget(hypot(init_carrot_x - pose.x, init_carrot_y - pose.y) * dir);
  pid_distance.setIntegralMax(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x_in - pose.x, y_in - pose.y))));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0;
  // Local mirror of the shared slew baseline; see driveTo().
  double prev_left_output = state().prevLeftOutput();
  double prev_right_output = state().prevRightOutput();
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // Main PID loop for boomerang path
  while (pros::millis() - start_time <= time_limit_msec && !cancelled())
  {
    pose = state().pose();
    hypotenuse = hypot(pose.x - x_in, pose.y - y_in); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x_in - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y_in - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - pose.x, carrot_y - pose.y) * dir);
    current_angle = getInertialHeading();
    // Calculate drive output based on carrot point
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - pose.x, carrot_y - pose.y) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((pose.y - y_in) * -cos(degToRad(normalizeTarget(a))) <= (pose.x - x_in) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
    if (perpendicular_line && !prev_perpendicular_line)
    {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if (apply_min_speed_floor && min_speed_output > 0)
    {
      scaleToMin(left_output, right_output, min_speed_output);
    }

    // Heading correction logic based on distance to carrot/target
    if (hypot(carrot_x - pose.x, carrot_y - pose.y) > 8)
    {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(carrot_x - pose.x, carrot_y - pose.y)) + add));
      correction_output = pid_heading.update(current_angle);
    }
    else if (hypot(x_in - pose.x, y_in - pose.y) > 6)
    {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x_in - pose.x, y_in - pose.y)) + add));
      correction_output = pid_heading.update(current_angle);
    }
    else
    {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if (exit && hypot(x_in - pose.x, y_in - pose.y) < 5 && pros::millis() - start_time > 200)
      {
        break;
      }
    }

    // Limit slip speed for smoother curves.
    //
    // The radius comes from mclib::arcRadius(), the compass-frame arc through
    // the carrot. It used to come from getRadius() in utils.hpp, which is
    // frame-transposed: it puts the target's Y offset where its robot-frame
    // lateral offset belongs, so for a robot at the origin heading 0 with the
    // carrot 10 in dead ahead it returns 5 instead of infinity - a hard speed
    // cap on a straight line. arcRadius() returns infinity there, and the
    // clamp below correctly does not fire.
    //
    // Both are signed - positive curves right, negative left - and the old
    // expression took sqrt() of that directly, so every left-hand arc produced
    // NaN and dropped the limiter. slipSpeedLimit() takes the magnitude: a
    // left arc slips at the same speed as its mirror image.
    //
    // Raw doubles on purpose, and inexpressible in units: chase_power is a
    // unitless fudge factor, the radius is inches, 9.8 is g in m/s^2, and the
    // result is compared against volts. Four unit systems in one expression,
    // deliberately preserved - see slipSpeedLimit() in motion_math.hpp and the
    // note on chase_power in config.hpp.
    //
    // current_angle, not pose.theta: motion.cpp steers on raw IMU degrees and
    // this has to be the same frame as the heading PID above it.
    slip_speed = slipSpeedLimit(
        chase_power,
        mclib::arcRadius(mclib::Pose2D{pose.x, pose.y, degToRad(current_angle)},
                         mclib::Vec2{carrot_x, carrot_y}));
    if (left_output > slip_speed)
    {
      left_output = slip_speed;
    }
    else if (left_output < -slip_speed)
    {
      left_output = -slip_speed;
    }

    // Overturn logic for sharp turns
    applyOverturnAndMix(left_output, right_output, correction_output, max_output, overturn);

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    applySlewClamp(left_output,
                   right_output,
                   prev_left_output,
                   prev_right_output,
                   max_slew_fwd,
                   max_slew_rev,
                   true);
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveVolts(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  if (exit)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    const double exit_decel = exitDecel(max_slew_fwd, max_slew_rev, max_output);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && !cancelled())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      driveVolts(prev_left_output, prev_right_output);
      pros::delay(10);
    }
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(mclib::device::BrakeMode::Hold); // Stop at end if required
  }
  // Publish the slew baseline so the next motion picks up where this one left
  // off (zero, on the exit path above).
  state().setPrevOutputs(prev_left_output, prev_right_output);
  state().setCorrectAngleDeg(settledHeadingDeg(a));  // Update shared heading
  state().setTurning(false); // Reset turning state
}
