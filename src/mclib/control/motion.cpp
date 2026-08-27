// mclib
#include "mclib/control/motion.hpp"

#include "api.h"
#include "mclib/config.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/scaling.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/math.hpp"
#include "mclib/pid.hpp"
#include "mclib/utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
namespace {
double applySlewLimit(double desired,
                      double previous,
                      double accel_limit,
                      double decel_limit,
                      double loop_dt_ms) {
  constexpr double nominal_loop_ms = 10.0;
  const double dt_scale = loop_dt_ms <= 0 ? 1.0 : loop_dt_ms / nominal_loop_ms;
  const double max_increase = accel_limit * dt_scale;
  const double max_decrease = decel_limit * dt_scale;
  const double delta = desired - previous;

  if (delta > max_increase) {
    return previous + max_increase;
  }
  if (delta < -max_decrease) {
    return previous - max_decrease;
  }
  return desired;
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
void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output, double min_speed)
{
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
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);

  // clamp keeps pid output within symmetric voltage rails so differential drive math stays bounded
  auto clampOutput = [&](double value)
  {
    if (value > max_output)
      return max_output;
    if (value < -max_output)
      return -max_output;
    return value;
  };

  if (!exit && getInertialHeading() < turn_angle)
  {
    // early exit path drives until the inertial pose passes the commanded heading allowing chained moves
    while (getInertialHeading() < turn_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      output = pid.update(getInertialHeading());
      // keep a minimum turning speed so the robot doesn't stall mid-chain
      if (min_speed_output > 0 && fabs(output) < min_speed_output)
        output = (output >= 0 ? min_speed_output : -min_speed_output);
      output = clampOutput(output);
      driveChassis(output, -output);
      pros::delay(10);
    }
  }
  else if (!exit && getInertialHeading() > turn_angle)
  {
    while (getInertialHeading() > turn_angle && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      output = pid.update(getInertialHeading());
      if (min_speed_output > 0 && fabs(output) < min_speed_output)
        output = (output >= 0 ? min_speed_output : -min_speed_output);
      output = clampOutput(output);
      driveChassis(output, -output);
      pros::delay(10);
    }
  }
  else
  {
    while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      output = clampOutput(pid.update(getInertialHeading()));
      driveChassis(output, -output);
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

void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output, double min_speed)
{
  // Store initial encoder values
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);
  bool apply_min_speed_floor = (min_speed >= 0 && min_speed_output > 0);
  if (!exit)
  {
    // Adjust slew rates and min speed for chaining
    if (!dir_change_start && dir_change_end)
    {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if (dir_change_start && !dir_change_end)
    {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      apply_min_speed_floor = true;
    }
    if (!dir_change_start && !dir_change_end)
    {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      apply_min_speed_floor = true;
    }
  }

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
    current_distance = (fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in) + fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in)) / 2;
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
    if (prev_left_output - left_output > max_slew_rev)
    {
      left_output = prev_left_output - max_slew_rev;
    }
    if (prev_right_output - right_output > max_slew_rev)
    {
      right_output = prev_right_output - max_slew_rev;
    }
    if (left_output - prev_left_output > max_slew_fwd)
    {
      left_output = prev_left_output + max_slew_fwd;
    }
    if (right_output - prev_right_output > max_slew_fwd)
    {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    pros::delay(10);
  }
  if (exit)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    // within the timeout (stops from max_output in ~300 ms).
    const double exit_decel = fmax(fmax(max_slew_fwd, max_slew_rev), max_output / 30.0);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && !cancelled())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      driveChassis(prev_left_output, prev_right_output);
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

void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output, double min_speed, bool reverse)
{
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
  in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distance_between_wheels / 2)) * result_angle);
  ratio = in_arc / out_arc;

  stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);
  double threshold = 0.5;

  // Determine curve and drive direction
  int curve_direction = center_radius > 0 ? 1 : -1;
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

  // Slew rate and minimum speed logic for chaining
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);
  bool apply_min_speed_floor = (min_speed >= 0 && min_speed_output > 0);
  if (!exit)
  {
    if (!dir_change_start && dir_change_end)
    {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if (dir_change_start && !dir_change_end)
    {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      apply_min_speed_floor = true;
    }
    if (!dir_change_start && !dir_change_end)
    {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      apply_min_speed_floor = true;
    }
  }

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
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
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

      driveChassis(left_output, right_output);
      pros::delay(10);
    }
  }
  else if (curve_direction == 1 && exit == true)
  {
    // Right curve, stop at end
    while (!pid_out.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
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

      driveChassis(left_output, right_output);
      pros::delay(10);
    }
  }
  else if (curve_direction == -1 && exit == false)
  {
    // Left curve, chaining (do not stop at end)
    while (current_right < out_arc && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
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

      driveChassis(left_output, right_output);
      pros::delay(10);
    }
  }
  else
  {
    // Right curve, chaining (do not stop at end)
    while (current_left < out_arc && pros::millis() - start_time <= time_limit_msec && !cancelled())
    {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
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

      driveChassis(left_output, right_output);
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

void curveCircleReverse(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output, double min_speed)
{
  curveCircle(result_angle_deg, center_radius, time_limit_msec, exit, max_output, min_speed, true);
}

void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output, double min_speed)
{
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
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);
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
      if (min_speed_output > 0 && fabs(output) < min_speed_output)
        output = (output >= 0 ? min_speed_output : -min_speed_output);
      if (output > max_output)
        output = max_output;
      else if (output < -max_output)
        output = -max_output;

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
      if (min_speed_output > 0 && fabs(output) < min_speed_output)
        output = (output >= 0 ? min_speed_output : -min_speed_output);
      if (output > max_output)
        output = max_output;
      else if (output < -max_output)
        output = -max_output;

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
      if (min_speed_output > 0 && fabs(output) < min_speed_output)
        output = (output >= 0 ? min_speed_output : -min_speed_output);
      if (output > max_output)
        output = max_output;
      else if (output < -max_output)
        output = -max_output;

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
      if (min_speed_output > 0 && fabs(output) < min_speed_output)
        output = (output >= 0 ? min_speed_output : -min_speed_output);
      if (output > max_output)
        output = max_output;
      else if (output < -max_output)
        output = -max_output;

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
    if (output > max_output)
      output = max_output;
    else if (output < -max_output)
      output = -max_output;

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
      driveChassis(output, -output); // Apply correction to chassis
    }
    pros::delay(10);
  }
}

void wallReset(double reset_x, double reset_y, double reset_heading,
               double drive_power, double time_limit_msec,
               double current_threshold, double velocity_threshold)
{
  uint32_t start_time = pros::millis();
  int stall_count = 0;
  constexpr int stall_cycles_needed = 5; // 50 ms of sustained stall

  // Drive into the wall
  driveChassis(drive_power, drive_power);

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
    if (avg_current > current_threshold && avg_vel < velocity_threshold)
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
  if (!std::isnan(reset_heading))
  {
    inertial_sensor.setRotationDeg(reset_heading);
    state().setCorrectAngleDeg(reset_heading);
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
      mclib::Pose2D{reset_x, reset_y, degToRad(getInertialHeading())});
}

void turnToPoint(double x, double y, int direction, double time_limit_msec, double min_speed)
{
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
  double turn_angle = normalizeTarget(radToDeg(atan2(x - pose.x, y - pose.y))) + add;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  pid.setTarget(turn_angle); // Set PID target
  pid.setIntegralMax(0);
  pid.setIntegralRange(3);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);

  double start_time = pros::millis();
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);
  while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && !cancelled())
  {
    pose = state().pose();
    pid.setTarget(normalizeTarget(radToDeg(atan2(x - pose.x, y - pose.y))) + add);
    double output = pid.update(getInertialHeading());
    if (min_speed_output > 0 && fabs(output) < min_speed_output)
    {
      output = (output >= 0 ? min_speed_output : -min_speed_output);
    }
    driveChassis(output, -output);
    pros::delay(10);
  }
  stopChassis(mclib::device::BrakeMode::Hold); // Stop at end
  state().setCorrectAngleDeg(getInertialHeading());  // Update shared heading
  state().setTurning(false);                    // Reset turning state
}

void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn, double min_speed)
{
  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before moving
  state().setTurning(true);                      // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);
  bool apply_min_speed_floor = (min_speed >= 0 && min_speed_output > 0);
  if (!exit)
  {
    // Adjust slew rates and min speed for chaining
    if (!dir_change_start && dir_change_end)
    {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if (dir_change_start && !dir_change_end)
    {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      apply_min_speed_floor = true;
    }
    if (!dir_change_start && !dir_change_end)
    {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      apply_min_speed_floor = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // One locked read: x and y always come from the same odometry tick.
  mclib::Pose2D pose = state().pose();
  // Set PID targets for distance and heading
  pid_distance.setTarget(hypot(x - pose.x, y - pose.y));
  pid_distance.setIntegralMax(0);
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - pose.x, y - pose.y)) + add));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);

  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis
  double start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  // Main PID loop for moving to point
  while (pros::millis() - start_time <= time_limit_msec && !cancelled())
  {
    // Continuously update targets as robot moves
    pose = state().pose();
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - pose.x, y - pose.y)) + add));
    pid_distance.setTarget(hypot(x - pose.x, y - pose.y));
    current_angle = getInertialHeading();
    // Calculate drive output based on heading and distance
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x - pose.x, y - pose.y) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((pose.y - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (pose.x - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if (perpendicular_line && !prev_perpendicular_line)
    {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Only apply heading correction if far from target
    if (hypot(x - pose.x, y - pose.y) > 8 && ch == true)
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
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if (overturn_value > 0 && overturn)
    {
      if (left_output > 0)
      {
        left_output -= overturn_value;
      }
      else
      {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    // When exit=true, skip decel slew so PID can brake before the target.
    // Only limit acceleration to prevent wheel slip on startup.
    if (!exit)
    {
      if (prev_left_output - left_output > max_slew_rev)
      {
        left_output = prev_left_output - max_slew_rev;
      }
      if (prev_right_output - right_output > max_slew_rev)
      {
        right_output = prev_right_output - max_slew_rev;
      }
    }
    if (left_output - prev_left_output > max_slew_fwd)
    {
      left_output = prev_left_output + max_slew_fwd;
    }
    if (right_output - prev_right_output > max_slew_fwd)
    {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  if (exit == true)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    const double exit_decel = fmax(fmax(max_slew_fwd, max_slew_rev), max_output / 30.0);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && !cancelled())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      driveChassis(prev_left_output, prev_right_output);
      pros::delay(10);
    }
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(mclib::device::BrakeMode::Hold); // Stop at end if required
    // Zero the shared slew baseline so the next driveTo starts cleanly
    state().setPrevOutputs(0.0, 0.0);
  }
  state().setCorrectAngleDeg(getInertialHeading()); // Update shared heading
  state().setTurning(false);                   // Reset turning state
}

void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn, double min_speed)
{
  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before moving
  state().setTurning(true);                      // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  const double min_speed_output = fmax(0.0, min_speed < 0 ? min_output : min_speed);
  bool apply_min_speed_floor = (min_speed >= 0 && min_speed_output > 0);
  if (!exit)
  {
    // Adjust slew rates and min speed for chaining
    if (!dir_change_start && dir_change_end)
    {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if (dir_change_start && !dir_change_end)
    {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      apply_min_speed_floor = true;
    }
    if (!dir_change_start && !dir_change_end)
    {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      apply_min_speed_floor = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // One locked read: x and y always come from the same odometry tick.
  mclib::Pose2D pose = state().pose();
  // Compute initial carrot so the PID starts with a real nonzero target
  double init_hyp = hypot(pose.x - x, pose.y - y);
  double init_carrot_x = x - init_hyp * sin(degToRad(a + add)) * dlead;
  double init_carrot_y = y - init_hyp * cos(degToRad(a + add)) * dlead;
  pid_distance.setTarget(hypot(init_carrot_x - pose.x, init_carrot_y - pose.y) * dir);
  pid_distance.setIntegralMax(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - pose.x, y - pose.y))));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
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
    hypotenuse = hypot(pose.x - x, pose.y - y); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - pose.x, carrot_y - pose.y) * dir);
    current_angle = getInertialHeading();
    // Calculate drive output based on carrot point
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - pose.x, carrot_y - pose.y) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((pose.y - y) * -cos(degToRad(normalizeTarget(a))) <= (pose.x - x) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
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
    else if (hypot(x - pose.x, y - pose.y) > 6)
    {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - pose.x, y - pose.y)) + add));
      correction_output = pid_heading.update(current_angle);
    }
    else
    {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if (exit && hypot(x - pose.x, y - pose.y) < 5 && pros::millis() - start_time > 200)
      {
        break;
      }
    }

    // Limit slip speed for smoother curves
    slip_speed = sqrt(chase_power * getRadius(pose.x, pose.y, carrot_x, carrot_y, current_angle) * 9.8);
    if (left_output > slip_speed)
    {
      left_output = slip_speed;
    }
    else if (left_output < -slip_speed)
    {
      left_output = -slip_speed;
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if (overturn_value > 0 && overturn)
    {
      if (left_output > 0)
      {
        left_output -= overturn_value;
      }
      else
      {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if (prev_left_output - left_output > max_slew_rev)
    {
      left_output = prev_left_output - max_slew_rev;
    }
    if (prev_right_output - right_output > max_slew_rev)
    {
      right_output = prev_right_output - max_slew_rev;
    }
    if (left_output - prev_left_output > max_slew_fwd)
    {
      left_output = prev_left_output + max_slew_fwd;
    }
    if (right_output - prev_right_output > max_slew_fwd)
    {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  if (exit)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    const double exit_decel = fmax(fmax(max_slew_fwd, max_slew_rev), max_output / 30.0);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && !cancelled())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      driveChassis(prev_left_output, prev_right_output);
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
