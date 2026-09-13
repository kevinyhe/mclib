// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/motion.hpp"

#include "api.h"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion_config.hpp"
#include "mclib/control/motion_math.hpp"
#include "mclib/control/scaling.hpp"
#include "mclib/control/swing_math.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/math.hpp"
#include "mclib/pid.hpp"
#include "mclib/units/units.hpp"
#include "mclib/utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>

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
using mclib::control::swingChoice;
using mclib::control::SwingCommand;
using mclib::control::swingCommand;

using mclib::control::MotionConfig;
using mclib::control::MotionPhase;

/// @brief The drivetrain every routine below runs on. Bound by
///        ChassisController's constructor or control::bindDrive().
mclib::control::DriveHardware& drive() {
  return *mclib::control::boundDrive();
}

/// @brief The active tuning. Read once at the top of each routine.
const MotionConfig& cfg() {
  return mclib::control::motionConfig();
}

/**
 * @brief Encoder degrees to inches rolled, from the bound drive's geometry.
 *
 * Replaces the seven copies of `deg * wheel_distance_in / 360.0` that used to
 * be open-coded in the loops below. `DriveGeometry::encoderToDistance()` is
 * the same formula, written once and typed.
 */
double encoderDegreesToInches(double deg) {
  return drive().driveGeometry().encoderToDistance(deg * mclib::units::degree).in();
}

/// @brief Half the track width in inches - the radius each wheel turns about
///        when the robot spins in place.
double halfTrackWidthIn() {
  return drive().driveGeometry().turnRadius().in();
}

/// @brief The tuned slew rates and chaining flags from the active config.
SlewConfig slewConfig() {
  const MotionConfig& c = cfg();
  SlewConfig config{};
  config.accel_fwd = c.slew.accel_fwd;
  config.decel_fwd = c.slew.decel_fwd;
  config.accel_rev = c.slew.accel_rev;
  config.decel_rev = c.slew.decel_rev;
  config.dir_change_start = c.dir_change_start;
  config.dir_change_end = c.dir_change_end;
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
  drive().brakeSide(true, mclib::device::BrakeMode::Hold);
}

void holdRightSide() {
  drive().brakeSide(false, mclib::device::BrakeMode::Hold);
}

/// @brief A PID over one gain set from the active config.
PID makePid(const mclib::PIDGains& gains) {
  return PID(gains.kp, gains.ki, gains.kd);
}

/// @brief Apply one exit rule from the active config.
void applyExit(PID& pid, const mclib::PIDExit& exit) {
  pid.setSmallBigErrorTolerance(exit.small_error, exit.big_error);
  pid.setSmallBigErrorDuration(exit.small_duration.ms(), exit.big_duration.ms());
  pid.setDerivativeTolerance(exit.derivative);
}

/// @brief The stiction floor in volts, or 0 when disabled.
double minOutput() {
  return cfg().min_voltage.volts();
}

/// @brief The shared state that replaced the bare globals in state.hpp.
mclib::control::RobotState& state() {
  return mclib::control::robotState();
}

// Observation only: never reads sensors, changes PID call order, or holds the
// state lock while invoking the PID, hardware, or simulator callbacks.
class MotionObservation {
 public:
  explicit MotionObservation(MotionPhase phase) {
    value.phase = phase;
    state().setMotionTelemetry(value);
  }
  ~MotionObservation() { state().setMotionTelemetry({}); }
  void target(PID& pid, double target) {
    value.target_heading_deg = target;
    pid.setTarget(target);
  }
  double heading(PID& pid, double input) {
    value.heading_error_deg = value.target_heading_deg - input;
    return pid.update(input);
  }
  void publish(MotionPhase phase, double drive, double yaw,
               bool slew_limited, bool voltage_limited) {
    value.phase = phase;
    value.drive_volts = drive;
    value.yaw_volts = yaw;
    value.slew_limited = slew_limited;
    value.voltage_limited = voltage_limited;
    state().setMotionTelemetry(value);
  }
  void decelerate(double left, double right) {
    // No new PID/sensor sample is taken in the existing exit ramp.
    value = {};
    publish(MotionPhase::Decelerate, 0, 0, left != 0 || right != 0,
            fabs(left) > 12 || fabs(right) > 12);
  }
  mclib::control::MotionTelemetry value{};
};

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

// Every blocking motion owns this guard, including chained (exit=false)
// motions. Only a successful exit may leave voltage for the next segment.
// Unsigned elapsed time also handles the PROS millisecond clock wrapping.
class MotionSafety {
 public:
  MotionSafety(QTime limit, std::initializer_list<double> inputs)
      : start_(pros::millis()), limit_(limit.ms()) {
    failed_ = !std::isfinite(limit_) || limit_ <= 0;
    for (double value : inputs) failed_ |= !std::isfinite(value);
  }
  bool running() {
    const auto pose = state().pose();
    failed_ |= cancelled() || pros::competition::is_disabled() ||
        static_cast<uint32_t>(pros::millis() - start_) >= limit_ ||
        !std::isfinite(getInertialHeading()) ||
        !std::isfinite(getLeftRotationDegree()) ||
        !std::isfinite(getRightRotationDegree()) ||
        !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.theta);
    return !failed_;
  }
  double heading() { return checked(getInertialHeading()); }
  double left() { return checked(getLeftRotationDegree()); }
  double right() { return checked(getRightRotationDegree()); }
  mclib::Pose2D pose() {
    const auto value = state().pose();
    checked(value.x); checked(value.y); checked(value.theta);
    return value;
  }
  double normalize(double angle) {
    const double current = heading();
    double delta = std::fmod(angle - current, 360.0);
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    return checked(current + delta);
  }
  void write(double left, double right) {
    checked(left); checked(right);
    if (running()) driveVolts(left, right);
    else { driveVolts(0, 0); stopChassis(mclib::device::BrakeMode::Hold); }
  }
  void writeSide(bool left, double voltage) {
    checked(voltage);
    if (running()) drive().setSideVoltage(left, voltage);
    else { driveVolts(0, 0); stopChassis(mclib::device::BrakeMode::Hold); }
  }
  void setCorrectHeading(double commanded) {
    const double value = running() ? commanded : heading();
    if (std::isfinite(value)) state().setCorrectAngleDeg(value);
  }
  ~MotionSafety() {
    if (!running()) {
      driveVolts(0, 0);
      stopChassis(mclib::device::BrakeMode::Hold);
      state().setPrevOutputs(0, 0);
      const double heading = getInertialHeading();
      if (std::isfinite(heading)) state().setCorrectAngleDeg(heading);
    }
    state().setTurning(false);
  }
 private:
  double checked(double value) {
    failed_ |= !std::isfinite(value);
    return value;
  }
  uint32_t start_;
  double limit_;
  bool failed_ = false;
};
}  // namespace
void turnToAngle(QAngle turn_angle_target, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  MotionObservation telemetry(MotionPhase::Turn);
  if (mclib::control::requireDrive("turnToAngle") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {turn_angle_target.deg(), max_voltage.volts(), min_voltage.volts()});
  if (!safety.running()) return;
  // Unwrap once, here. Everything below is the arithmetic this routine always
  // ran, on the same doubles; see the file comment.
  double turn_angle = turn_angle_target.deg();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // Brake mode helps dissipate momentum near the setpoint and reduces hunting.
  stopChassis(mclib::device::BrakeMode::Brake);
  state().setTurning(true);
  PID pid = makePid(cfg().turn_pid);

  turn_angle = safety.normalize(turn_angle);
  telemetry.target(pid, turn_angle);
  pid.setIntegralMax(0);
  pid.setIntegralRange(3);
  applyExit(pid, cfg().turn_exit);

  const uint32_t start_time = pros::millis();
  double output = 0;
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());

  if (!exit && safety.heading() < turn_angle)
  {
    // early exit path drives until the inertial pose passes the commanded heading allowing chained moves
    while (safety.heading() < turn_angle && pros::millis() - start_time <= time_limit_msec && safety.running())
    {
      output = telemetry.heading(pid, safety.heading());
      // keep a minimum turning speed so the robot doesn't stall mid-chain
      output = applyMinSpeedFloor(output, min_speed_output);
      // clamp keeps pid output within symmetric voltage rails so differential
      // drive math stays bounded
      const double requested = output;
      output = clampSymmetric(output, max_output);
      telemetry.publish(MotionPhase::Turn, 0, requested, false,
                        requested != output || fabs(output) > 12);
      safety.write(output, -output);
      pros::delay(10);
    }
  }
  else if (!exit && safety.heading() > turn_angle)
  {
    while (safety.heading() > turn_angle && pros::millis() - start_time <= time_limit_msec && safety.running())
    {
      output = telemetry.heading(pid, safety.heading());
      output = applyMinSpeedFloor(output, min_speed_output);
      const double requested = output;
      output = clampSymmetric(output, max_output);
      telemetry.publish(MotionPhase::Turn, 0, requested, false,
                        requested != output || fabs(output) > 12);
      safety.write(output, -output);
      pros::delay(10);
    }
  }
  else
  {
    while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && safety.running())
    {
      const double requested = telemetry.heading(pid, safety.heading());
      output = clampSymmetric(requested, max_output);
      telemetry.publish(MotionPhase::Turn, 0, requested, false,
                        requested != output || fabs(output) > 12);
      safety.write(output, -output);
      pros::delay(10);
    }
  }

  if (exit)
  {
    stopChassis(mclib::device::BrakeMode::Hold);
  }
  safety.setCorrectHeading(turn_angle);
  state().setTurning(false);
}

void driveTo(QLength distance, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  MotionObservation telemetry(MotionPhase::Drive);
  if (mclib::control::requireDrive("driveTo") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {distance.in(), max_voltage.volts(), min_voltage.volts()});
  if (!safety.running()) return;
  double distance_in = distance.in();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // Store initial encoder values
  double start_left = safety.left(), start_right = safety.right();
  stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);
  int drive_direction = distance_in > 0 ? 1 : -1;
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());
  const SlewPlan slew =
      planSlew(slewConfig(), drive_direction, exit, min_speed >= 0, min_speed_output);
  const double max_slew_fwd = slew.max_slew_fwd;
  const double max_slew_rev = slew.max_slew_rev;
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  // flip target so pid always works with a positive distance scalar regardless of command direction
  distance_in = distance_in * drive_direction;
  PID pid_distance = makePid(cfg().distance_pid);
  PID pid_heading = makePid(cfg().heading_pid);

  // Configure PID controllers
  pid_distance.setTarget(distance_in);
  pid_distance.setIntegralMax(3);
  applyExit(pid_distance, cfg().distance_exit);

  telemetry.target(pid_heading, safety.normalize(state().correctAngleDeg()));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  uint32_t start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_distance = 0, current_angle = 0;
  // Local mirror of the shared slew baseline: read once here, published once
  // at the end. The observation snapshot is published separately each tick.
  double prev_left_output = state().prevLeftOutput();
  double prev_right_output = state().prevRightOutput();

  // Main PID loop for driving straight
  while ((((!pid_distance.targetArrived()) && pros::millis() - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && pros::millis() - start_time <= time_limit_msec)) && safety.running())
  {
    // integrate wheel travel by converting encoder degrees into linear inches and averaging both treads
    current_distance = (fabs(encoderDegreesToInches(safety.left() - start_left)) + fabs(encoderDegreesToInches(safety.right() - start_right))) / 2;
    current_angle = safety.heading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = telemetry.heading(pid_heading, current_angle);
    telemetry.value.remaining_in = distance_in - current_distance;

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

    const double requested_drive = left_output;
    // heading pid adds and subtracts to create differential voltage that cancels yaw error
    left_output += correction_output;
    right_output -= correction_output;

    // Max Output Check
    const double mixed_left = left_output, mixed_right = right_output;
    scaleToMax(left_output, right_output, max_output);
    const bool voltage_limited = left_output != mixed_left || right_output != mixed_right;
    const double before_slew_left = left_output, before_slew_right = right_output;

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
    telemetry.publish(MotionPhase::Drive, requested_drive, correction_output,
                      left_output != before_slew_left || right_output != before_slew_right,
                      voltage_limited || fabs(left_output) > 12 || fabs(right_output) > 12);
    safety.write(left_output, right_output);
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
           pros::millis() - ramp_start < ramp_timeout && safety.running())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      telemetry.decelerate(prev_left_output, prev_right_output);
      safety.write(prev_left_output, prev_right_output);
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
  MotionObservation telemetry(MotionPhase::Arc);
  if (mclib::control::requireDrive("curveCircle") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {result_angle_target.deg(), center_radius.in(), max_voltage.volts(), min_voltage.volts()});
  if (!safety.running()) return;
  double result_angle_deg = result_angle_target.deg();
  // Signed: the sign picks the curve direction, the magnitude is the radius.
  const double center_radius_in = center_radius.in();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // Store initial encoder values for both sides
  double start_right = safety.right(), start_left = safety.left();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // Normalize the target angle to be within +/-180 degrees of the current heading
  result_angle_deg = safety.normalize(result_angle_deg);
  // convert delta heading into radians so arc length math can use radius times angle
  const double entry_angle_deg = safety.heading();
  result_angle = (result_angle_deg - entry_angle_deg) * 3.14159265359 / 180;

  // Calculate arc lengths for inner and outer wheels
  // The inner tread reverses when the centre radius lies inside the track.
  // Keeping that sign also makes a zero-radius arc a symmetric in-place turn.
  const double half_track_width_in = halfTrackWidthIn();
  in_arc = (fabs(center_radius_in) - half_track_width_in) * fabs(result_angle);
  out_arc = fabs((fabs(center_radius_in) + half_track_width_in) * result_angle);
  // out_arc is zero exactly when the normalised target heading is already the
  // entry heading, and in_arc is zero with it. Every loop below divides by
  // out_arc - once for `ratio`, once more per tick for `real_angle` - so the
  // whole routine would run on NaN, and NaN survives every clamp and
  // comparison in it. There is no arc to drive, so publish the heading and
  // leave.
  if (out_arc == 0)
  {
    if (exit == true)
    {
      stopChassis(mclib::device::BrakeMode::Hold);
      state().setPrevOutputs(0, 0);
    }
    safety.setCorrectHeading(result_angle_deg);
    return;
  }
  ratio = in_arc / out_arc;

  stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);

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
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());
  const SlewPlan slew =
      planSlew(slewConfig(), drive_direction, exit, min_speed >= 0, min_speed_output);
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  // Initialize PID controllers for arc distance and heading correction
  PID pid_out = makePid(cfg().distance_pid);
  PID pid_turn = makePid(cfg().heading_pid);

  pid_out.setTarget(out_arc);
  pid_out.setIntegralMax(0);
  pid_out.setIntegralRange(5);
  applyExit(pid_out, cfg().arc_exit);
  // A chained arc must keep producing distance output until the measured
  // outer-wheel travel crosses the endpoint, even inside its settle band.
  pid_out.setArrive(exit);

  telemetry.target(pid_turn, 0);
  pid_turn.setIntegralMax(0);
  pid_turn.setIntegralRange(1);
  pid_turn.setSmallBigErrorTolerance(0, 0);
  pid_turn.setSmallBigErrorDuration(0, 0);
  pid_turn.setDerivativeTolerance(0);
  pid_turn.setArrive(false);

  uint32_t start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0;
  while (!pid_out.targetArrived() &&
         pros::millis() - start_time <= time_limit_msec && safety.running())
  {
    current_angle = safety.heading();
    const double encoder_delta = curve_direction < 0
        ? safety.right() - start_right : safety.left() - start_left;
    // Backward travel is progress only when this arc requests reverse.
    const double progress = encoderDegreesToInches(encoder_delta) * drive_direction;
    // Preserve the preceding live command at a successful chaining boundary;
    // updating the PID after the crossing can reverse the drive for one tick.
    if (!exit && progress >= out_arc) break;
    real_angle = progress / out_arc * (result_angle_deg - entry_angle_deg) + entry_angle_deg;
    telemetry.target(pid_turn, safety.normalize(real_angle));
    const double outer_output = pid_out.update(progress) * drive_direction;
    left_output = curve_direction > 0 ? outer_output : outer_output * ratio;
    right_output = curve_direction < 0 ? outer_output : outer_output * ratio;
    correction_output = telemetry.heading(pid_turn, current_angle);
    telemetry.value.remaining_in = out_arc - progress;

    if (apply_min_speed_floor && min_speed_output > 0)
    {
      scaleToMin(left_output, right_output, min_speed_output);
    }

    const double requested_drive = (left_output + right_output) / 2;
    const double requested_yaw = (left_output - right_output) / 2 + correction_output;
    left_output += correction_output;
    right_output -= correction_output;
    const double mixed_left = left_output, mixed_right = right_output;
    scaleToMax(left_output, right_output, max_output);
    telemetry.publish(MotionPhase::Arc, requested_drive, requested_yaw, false,
                      left_output != mixed_left || right_output != mixed_right ||
                      fabs(left_output) > 12 || fabs(right_output) > 12);
    safety.write(left_output, right_output);
    pros::delay(10);
  }
  // Stop the chassis if required
  if (exit == true)
  {
    stopChassis(mclib::device::BrakeMode::Hold);
    left_output = right_output = 0;
  }
  state().setPrevOutputs(left_output, right_output);
  // Update the global heading
  safety.setCorrectHeading(result_angle_deg);
  state().setTurning(false);
}

void curveCircleReverse(QAngle result_angle, QLength center_radius, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  curveCircle(result_angle, center_radius, time_limit, exit, max_voltage, min_voltage, true);
}

void swing(QAngle swing_angle_target, double drive_direction, QTime time_limit, bool exit, QVoltage max_voltage, QVoltage min_voltage)
{
  MotionObservation telemetry(MotionPhase::Swing);
  if (mclib::control::requireDrive("swing") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {swing_angle_target.deg(), drive_direction, max_voltage.volts(), min_voltage.volts()});
  if (!safety.running()) return;
  double swing_angle = swing_angle_target.deg();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before starting swing
  state().setTurning(true);                      // Set turning state
  PID pid = makePid(cfg().turn_pid); // Initialize PID for turning

  swing_angle = safety.normalize(swing_angle); // Normalize target angle
  telemetry.target(pid, swing_angle);          // Set PID target
  pid.setIntegralMax(0);
  pid.setIntegralRange(5);
  applyExit(pid, cfg().turn_exit);
  pid.setArrive(exit);

  // Start the PID loop
  uint32_t start_time = pros::millis();
  double output;
  double left_output = 0, right_output = 0;
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());
  const double entry_angle_deg = safety.heading();
  double current_heading = entry_angle_deg;
  // choice encodes which tread stays locked so swing math can reuse one code
  // path per quadrant. Both the tread and the sign now come out of
  // control/swing_math.hpp, so the chaining path and the exit path cannot
  // disagree about direction the way they used to.
  const int choice = swingChoice(swing_angle, entry_angle_deg, drive_direction);
  const bool heading_must_rise = (choice == 2 || choice == 4);

  // Drive whichever tread is not held for one tick.
  auto commandSwing = [&](double out, double requested)
  {
    if (!std::isfinite(out)) { safety.write(0, 0); return; }
    const SwingCommand cmd = swingCommand(choice, out, drive_direction);
    left_output = cmd.drive_left ? cmd.voltage : 0;
    right_output = cmd.drive_left ? 0 : cmd.voltage;
    const SwingCommand demand = swingCommand(choice, requested, drive_direction);
    telemetry.publish(MotionPhase::Swing, demand.voltage / 2,
                      (demand.drive_left ? demand.voltage : -demand.voltage) / 2,
                      false, requested != out || fabs(cmd.voltage) > 12);
    if (cmd.drive_left)
    {
      safety.writeSide(true, cmd.voltage);
      holdRightSide();
    }
    else
    {
      holdLeftSide();
      safety.writeSide(false, cmd.voltage);
    }
  };

  // Chaining (exit == false): run until the heading crosses the target and
  // leave the drive moving for whatever comes next.
  while (exit == false &&
         pros::millis() - start_time <= time_limit_msec && safety.running())
  {
    current_heading = safety.heading();
    // Preserve the preceding live command at the crossing. Running another
    // PID tick beyond the target can reverse the swing before handing off.
    if (heading_must_rise ? current_heading >= swing_angle
                          : current_heading <= swing_angle) break;
    output = telemetry.heading(pid, current_heading);

    // Clamp output
    output = applyMinSpeedFloor(output, min_speed_output);
    const double requested = output;
    output = clampSymmetric(output, max_output);

    commandSwing(output, requested);
    pros::delay(10);
  }

  // PID loop for exit == true (stop at end)
  while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && exit == true && safety.running())
  {
    current_heading = safety.heading();
    output = telemetry.heading(pid, current_heading);

    // Clamp output
    const double requested = output;
    output = clampSymmetric(output, max_output);

    commandSwing(output, requested);
    pros::delay(10);
  }
  if (exit == true)
  {
    stopChassis(mclib::device::BrakeMode::Hold); // Stop chassis at end if required
    left_output = right_output = 0;
  }
  state().setPrevOutputs(left_output, right_output);
  safety.setCorrectHeading(swing_angle); // Update shared heading
  state().setTurning(false);          // Reset turning state
}

void correctHeading()
{
  if (mclib::control::requireDrive("correctHeading") == nullptr) {
    return;
  }
  double output = 0;
  PID pid = makePid(cfg().heading_pid);

  pid.setTarget(state().correctAngleDeg()); // Set PID target to current heading
  pid.setIntegralRange(fabs(state().correctAngleDeg()) / 2.5);

  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);

  // feed equal magnitude opposite sign voltages
  // this cancels drift while driving straight
  while (cfg().heading_correction && !headingCorrectionCancelled() &&
         !pros::competition::is_disabled())
  {
    pid.setTarget(state().correctAngleDeg());
    if (!state().isTurning())
    {
      const double heading = getInertialHeading();
      if (!std::isfinite(heading) || !std::isfinite(state().correctAngleDeg())) {
        driveVolts(0, 0);
        stopChassis(mclib::device::BrakeMode::Hold);
        break;
      }
      output = pid.update(heading);
      driveVolts(output, -output); // Apply correction to chassis
    }
    pros::delay(10);
  }
  if (!state().isTurning()) {
    driveVolts(0, 0);
    stopChassis(mclib::device::BrakeMode::Hold);
  }
}

bool wallReset(QLength reset_x, QLength reset_y, QAngle reset_heading,
               QVoltage drive_power, QTime time_limit,
               QCurrent current_threshold, QAngularVelocity velocity_threshold)
{
  MotionObservation telemetry(MotionPhase::Wall);
  if (mclib::control::requireDrive("wallReset") == nullptr) {
    return false;
  }
  MotionSafety safety(time_limit, {reset_x.in(), reset_y.in(), drive_power.volts(), current_threshold.mA(), velocity_threshold.rpm()});
  if (!safety.running()) return false;
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

  // Claim the drive for the whole push. correctHeading() runs at 10 ms and
  // overwrites both sides with (v, -v) whenever this is not set, so without it
  // the robot never actually pushes into the wall, never stalls, and the
  // routine times out and stamps reset_x/reset_y onto a pose it never reached.
  state().setTurning(true);

  // Drive into the wall
  telemetry.publish(MotionPhase::Wall, drive_power_volts, 0, false,
                    fabs(drive_power_volts) > 12);
  safety.write(drive_power_volts, drive_power_volts);

  // Give the robot a moment to start moving before checking stall
  while (pros::millis() - start_time < 200 && safety.running())
    pros::delay(1);

  while (pros::millis() - start_time <= time_limit_msec && safety.running())
  {
    // Average current across all drive motors (mA)
    const auto currents = drive().driveCurrentsMa();
    if (currents.empty() || !std::all_of(currents.begin(), currents.end(),
        [](double value) {
          return std::isfinite(value) && value >= 0 && value != PROS_ERR;
        })) break;
    double total_current = 0;
    for (auto c : currents)
    {
      total_current += c;
    }
    double avg_current = currents.empty() ? 0 : total_current / currents.size();

    // Average speed across all drive motors (RPM)
    const auto velocities = drive().driveVelocitiesRpm();
    if (velocities.empty() || !std::all_of(velocities.begin(), velocities.end(),
        [](double value) { return std::isfinite(value); })) break;
    double total_vel = 0;
    for (auto v : velocities)
    {
      total_vel += fabs(v);
    }
    double avg_vel = velocities.empty() ? 0 : total_vel / velocities.size();

    // Stall detection: high current, low velocity
    if (avg_current >= current_threshold_ma && avg_vel < velocity_threshold_rpm)
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

  // Stop motors.
  stopChassis(mclib::device::BrakeMode::Brake);

  if (stall_count < stall_cycles_needed || !safety.running() ||
      (!std::isnan(reset_heading_deg) && !std::isfinite(reset_heading_deg))) {
    return false;
  }

  // Reset heading first if a valid value was provided, so the pose below is
  // built from the IMU value we are actually going to keep.
  if (!std::isnan(reset_heading_deg))
  {
    drive().setHeadingDeg(reset_heading_deg);
    safety.setCorrectHeading(reset_heading_deg);
  }
  else
  {
    safety.setCorrectHeading(safety.heading());
  }

  // Reset position to known coordinates. This goes through the odometry, not
  // just the pose, so the next odometry tick re-seeds its encoder baseline
  // instead of integrating a delta across the teleport. A non-finite heading
  // leaves the odometry's heading alone.
  mclib::control::resetOdometry(
      mclib::Pose2D{reset_x_in, reset_y_in, degToRad(safety.heading())});

  // Released last, after correctAngleDeg is published, exactly like every
  // other routine here. Clearing it earlier lets correctHeading() tick once
  // against the stale pre-push target while the IMU has already jumped to
  // reset_heading, which is a full-voltage counter-rotation that undoes the
  // squaring this routine just did.
  state().setTurning(false);
  return true;
}

void turnToPoint(QLength x, QLength y, int direction, QTime time_limit, QVoltage min_voltage)
{
  MotionObservation telemetry(MotionPhase::Turn);
  if (mclib::control::requireDrive("turnToPoint") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {x.in(), y.in(), min_voltage.volts()});
  if (!safety.running()) return;
  const double x_in = x.in();
  const double y_in = y.in();
  const double time_limit_msec = time_limit.ms();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before turning
  state().setTurning(true);                      // Set turning state
  double add = 0;
  if (direction == -1)
  {
    add = 180; // Add 180 degrees if turning to face backward
  }
  // One locked read: x and y always come from the same odometry tick.
  mclib::Pose2D pose = safety.pose();
  // Calculate target angle using atan2 and normalize
  double turn_angle = safety.normalize(radToDeg(atan2(x_in - pose.x, y_in - pose.y))) + add;
  PID pid = makePid(cfg().turn_pid);

  telemetry.target(pid, turn_angle); // Set PID target
  pid.setIntegralMax(0);
  pid.setIntegralRange(3);
  // The turn exit rule, but with the settle durations doubled: the target
  // moves as the odometry updates, so this loop needs longer to be sure it
  // has stopped chasing.
  {
    mclib::PIDExit exit = cfg().turn_exit;
    exit.small_duration = exit.small_duration * 2.0;
    exit.big_duration = exit.big_duration * 2.0;
    applyExit(pid, exit);
  }

  uint32_t start_time = pros::millis();
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());
  while (!pid.targetArrived() && pros::millis() - start_time <= time_limit_msec && safety.running())
  {
    pose = safety.pose();
    telemetry.target(pid, safety.normalize(radToDeg(atan2(x_in - pose.x, y_in - pose.y))) + add);
    double output = telemetry.heading(pid, safety.heading());
    output = applyMinSpeedFloor(output, min_speed_output);
    telemetry.publish(MotionPhase::Turn, 0, output, false, fabs(output) > 12);
    safety.write(output, -output);
    pros::delay(10);
  }
  stopChassis(mclib::device::BrakeMode::Hold); // Stop at end
  safety.setCorrectHeading(safety.heading());  // Update shared heading
  state().setTurning(false);                    // Reset turning state
}

void moveToPoint(QLength x, QLength y, int dir, QTime time_limit, bool exit, QVoltage max_voltage, bool overturn, QVoltage min_voltage)
{
  MotionObservation telemetry(MotionPhase::Point);
  if (mclib::control::requireDrive("moveToPoint") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {x.in(), y.in(), max_voltage.volts(), min_voltage.volts()});
  if (!safety.running()) return;
  const double x_in = x.in();
  const double y_in = y.in();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  stopChassis(mclib::device::BrakeMode::Coast); // Stop chassis before moving
  state().setTurning(true);                      // Set turning state
  int add = dir > 0 ? 0 : 180;
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());
  const SlewPlan slew =
      planSlew(slewConfig(), dir, exit, min_speed >= 0, min_speed_output);
  const double max_slew_fwd = slew.max_slew_fwd;
  const double max_slew_rev = slew.max_slew_rev;
  const bool apply_min_speed_floor = slew.apply_min_speed_floor;

  PID pid_distance = makePid(cfg().distance_pid);
  PID pid_heading = makePid(cfg().heading_pid);

  // One locked read: x_in and y_in always come from the same odometry tick.
  mclib::Pose2D pose = safety.pose();
  // Set PID targets for distance and heading
  pid_distance.setTarget(hypot(x_in - pose.x, y_in - pose.y));
  pid_distance.setIntegralMax(0);
  pid_distance.setIntegralRange(3);
  applyExit(pid_distance, cfg().distance_exit);
  // A stopped move exits on this latch. A chained move must keep driving
  // until it crosses the target line, even inside the PID's settle band.
  pid_distance.setArrive(exit);

  telemetry.target(pid_heading, safety.normalize(radToDeg(atan2(x_in - pose.x, y_in - pose.y)) + add));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);

  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis
  uint32_t start_time = pros::millis();
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
  // Main PID loop for moving to point
  while (pros::millis() - start_time <= time_limit_msec && safety.running())
  {
    // Continuously update targets as robot moves
    pose = safety.pose();
    telemetry.target(pid_heading, safety.normalize(radToDeg(atan2(x_in - pose.x, y_in - pose.y)) + add));
    pid_distance.setTarget(hypot(x_in - pose.x, y_in - pose.y));
    current_angle = safety.heading();
    telemetry.value.heading_error_deg = telemetry.value.target_heading_deg - current_angle;
    // Calculate drive output based on heading and distance
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x_in - pose.x, y_in - pose.y) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    if (exit && pid_distance.targetArrived()) break;
    const double remaining = hypot(x_in - pose.x, y_in - pose.y);
    telemetry.value.remaining_in = remaining;
    const double position_band = std::max(cfg().distance_exit.small_error,
                                          cfg().distance_exit.big_error);
    const double bearing_error = mclib::wrapAngle(
        atan2(x_in - pose.x, y_in - pose.y) + degToRad(add - current_angle));
    // A nearby target beside/behind the robot still needs steering. Allow a
    // symmetric pivot before translating, without the distance-oriented slew
    // turning (+yaw,-yaw) into a lopsided drive command.
    if (remaining > position_band && fabs(bearing_error) > M_PI / 3.0) {
      const double requested_yaw = telemetry.heading(pid_heading, current_angle);
      correction_output = clampSymmetric(requested_yaw, max_output);
      prev_left_output = correction_output;
      prev_right_output = -correction_output;
      telemetry.publish(MotionPhase::Pivot, 0, requested_yaw, false,
                        requested_yaw != correction_output || fabs(correction_output) > 12);
      safety.write(prev_left_output, prev_right_output);
      pros::delay(10);
      continue;
    }
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((pose.y - y_in) * -cos(degToRad(safety.normalize(current_angle + add))) <= (pose.x - x_in) * sin(degToRad(safety.normalize(current_angle + add))) + exittolerance);
    if (!exit && perpendicular_line && !prev_perpendicular_line &&
        remaining <= std::max(position_band, exittolerance))
    {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    bool heading_voltage_limited = false;
    double requested_yaw = 0;
    // Continue endpoint steering until inside the configured position band.
    // Disabling it at eight inches made short lateral moves unreachable.
    if (remaining > position_band)
    {
      correction_output = telemetry.heading(pid_heading, current_angle);
      requested_yaw = correction_output;
      // Cap correction so it can't overwhelm the forward drive and cause a pivot
      double max_correction = remaining > 8 ? fabs(left_output) * 0.75 : max_output;
      heading_voltage_limited = remaining <= 8 && fabs(correction_output) > max_output;
      if (fabs(correction_output) > max_correction)
      {
        correction_output = (correction_output > 0) ? max_correction : -max_correction;
      }
    }
    else
    {
      correction_output = 0;
    }

    // Minimum Output Check
    if (apply_min_speed_floor && min_speed_output > 0)
    {
      scaleToMin(left_output, right_output, min_speed_output);
    }

    const double requested_drive = left_output;
    const bool mixing_limited = fabs(left_output) + fabs(correction_output) > max_output;
    // Overturn logic for sharp turns
    applyOverturnAndMix(left_output, right_output, correction_output, max_output, overturn);

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    const double before_slew_left = left_output, before_slew_right = right_output;
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
    telemetry.publish(MotionPhase::Point, requested_drive, requested_yaw,
                      left_output != before_slew_left || right_output != before_slew_right,
                      heading_voltage_limited || mixing_limited ||
                      fabs(left_output) > 12 || fabs(right_output) > 12);
    safety.write(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  if (exit == true)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    const double exit_decel = exitDecel(max_slew_fwd, max_slew_rev, max_output);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && safety.running())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      telemetry.decelerate(prev_left_output, prev_right_output);
      safety.write(prev_left_output, prev_right_output);
      pros::delay(10);
    }
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(mclib::device::BrakeMode::Hold); // Stop at end if required
  }
  // Publish the slew baseline so the next motion picks up where this one left
  // off (zero, on the exit path above). Same contract as driveTo/boomerang.
  state().setPrevOutputs(prev_left_output, prev_right_output);
  safety.setCorrectHeading(safety.heading()); // Update shared heading
  state().setTurning(false);                   // Reset turning state
}

void boomerang(QLength x, QLength y, int dir, QAngle final_heading, double dlead, QTime time_limit, bool exit, QVoltage max_voltage, bool overturn, QVoltage min_voltage)
{
  MotionObservation telemetry(MotionPhase::Pursuit);
  if (mclib::control::requireDrive("boomerang") == nullptr) {
    return;
  }
  MotionSafety safety(time_limit, {x.in(), y.in(), final_heading.deg(), dlead, max_voltage.volts(), min_voltage.volts()});
  if (!safety.running()) return;
  const double x_in = x.in();
  const double y_in = y.in();
  // The final heading, degrees. `dlead` is genuinely dimensionless.
  const double a = final_heading.deg();
  const double time_limit_msec = time_limit.ms();
  const double max_output = max_voltage.volts();
  const double min_speed = min_voltage.volts();

  // A chained pose that is already reached is a no-op handoff. Do not clear
  // its live wheel commands while retaining a nonzero shared slew baseline.
  if (exit) stopChassis(mclib::device::BrakeMode::Coast);
  state().setTurning(true);                      // Set turning state
  int add = dir > 0 ? 0 : 180;
  const double min_speed_output = minSpeedOutput(min_speed, minOutput());
  const SlewPlan slew =
      planSlew(slewConfig(), dir, exit, min_speed >= 0, min_speed_output);
  const double max_slew_fwd = slew.max_slew_fwd;
  const double max_slew_rev = slew.max_slew_rev;
  // Keep a non-stopping approach moving outside its endpoint band, using the
  // existing configured floor. Explicit min_speed=0 (or configured
  // min_voltage=0) still opts out; braking keeps its signed PID demand.
  const bool apply_min_speed_floor =
      slew.apply_min_speed_floor || (!exit && min_speed_output > 0);

  PID pid_distance = makePid(cfg().distance_pid);
  PID pid_heading = makePid(cfg().heading_pid);

  // One locked read: x_in and y_in always come from the same odometry tick.
  mclib::Pose2D pose = safety.pose();
  // Steering follows the carrot; speed/settlement use distance to the actual
  // endpoint, not the shorter distance to that moving intermediate target.
  double init_hyp = hypot(pose.x - x_in, pose.y - y_in);
  pid_distance.setTarget(init_hyp * dir);
  pid_distance.setIntegralMax(3);
  applyExit(pid_distance, cfg().distance_exit);
  pid_distance.setArrive(exit);
  pid_distance.setHoldOutput(true);
  PID final_turn = makePid(cfg().turn_pid);
  final_turn.setIntegralMax(0);
  applyExit(final_turn, cfg().turn_exit);
  bool aligning_heading = false;
  const double position_band = std::max(cfg().distance_exit.small_error,
                                        cfg().distance_exit.big_error);

  telemetry.target(pid_heading, safety.normalize(radToDeg(atan2(x_in - pose.x, y_in - pose.y))));
  pid_heading.setIntegralMax(0);
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  uint32_t start_time = pros::millis();
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0;
  // Local mirror of the shared slew baseline; see driveTo().
  double prev_left_output = state().prevLeftOutput();
  double prev_right_output = state().prevRightOutput();
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // Main PID loop for boomerang path
  while (pros::millis() - start_time <= time_limit_msec && safety.running())
  {
    pose = safety.pose();
    hypotenuse = hypot(pose.x - x_in, pose.y - y_in); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x_in - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y_in - hypotenuse * cos(degToRad(a + add)) * dlead;
    telemetry.value.remaining_in = hypotenuse;
    telemetry.value.carrot_x = carrot_x;
    telemetry.value.carrot_y = carrot_y;
    pid_distance.setTarget(hypotenuse * dir);
    current_angle = safety.heading();
    const double carrot_heading = radToDeg(atan2(carrot_x - pose.x, carrot_y - pose.y)) + add;
    const double bearing_error = mclib::wrapAngle(degToRad(carrot_heading - current_angle));
    // Calculate drive output based on carrot point
    left_output = pid_distance.update(0) * cos(bearing_error);
    right_output = left_output;
    // A moving handoff uses the same configured position band as a stopped
    // arrival, without waiting for settlement or final heading. An infinite
    // line allowed success arbitrarily far sideways, while requiring an
    // exact crossing conflicts with a distance PID that settles just short.
    // A level condition also permits recovery after an earlier distant pass.
    if (!exit && hypotenuse <= position_band) {
      break;
    }

    // Minimum Output Check
    if (apply_min_speed_floor && min_speed_output > 0)
    {
      scaleToMin(left_output, right_output, min_speed_output);
    }

    // Do not abandon endpoint steering six inches away or declare success
    // five inches short. First settle position, then settle the final heading.
    // If turning/drift leaves the position band, resume endpoint acquisition.
    if (aligning_heading && hypotenuse > position_band) {
      aligning_heading = false;
      pid_distance.reset();
      pid_heading.reset();
      final_turn.reset();
    }
    if (exit && pid_distance.targetArrived() && hypotenuse <= position_band) {
      aligning_heading = true;
    }
    if (aligning_heading) {
      telemetry.target(final_turn, safety.normalize(a));
      const double requested_yaw = telemetry.heading(final_turn, current_angle);
      correction_output = clampSymmetric(requested_yaw, max_output);
      if (final_turn.targetArrived()) break;
      // As in turnToAngle, command one yaw scalar and its negative;
      // final alignment has no translation demand or translation slew.
      prev_left_output = correction_output;
      prev_right_output = -correction_output;
      // The carrot is not an active target during final heading alignment.
      telemetry.value.carrot_x = telemetry.value.carrot_y = NAN;
      telemetry.publish(MotionPhase::FinalAlign, 0, requested_yaw, false,
                        requested_yaw != correction_output || fabs(correction_output) > 12);
      safety.write(prev_left_output, prev_right_output);
      pros::delay(10);
      continue;
    } else {
      telemetry.target(pid_heading, safety.normalize(carrot_heading));
      correction_output = telemetry.heading(pid_heading, current_angle);
    }

    // A carrot behind the selected travel direction needs reorientation,
    // not propulsion in the opposite direction from a negative cosine. Keep
    // the distance PID updated above, and retain small positional recovery
    // inside the endpoint band while its settlement timer runs.
    if (hypotenuse > position_band && fabs(bearing_error) > M_PI / 2.0) {
      const double requested_yaw = correction_output;
      correction_output = clampSymmetric(correction_output, max_output);
      prev_left_output = correction_output;
      prev_right_output = -correction_output;
      telemetry.publish(MotionPhase::Pivot, 0, requested_yaw, false,
                        requested_yaw != correction_output || fabs(correction_output) > 12);
      safety.write(prev_left_output, prev_right_output);
      pros::delay(10);
      continue;
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
        cfg().chase_power,
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

    // Slew translation before mixing yaw. Slewing the two wheels separately
    // turned (+12,-12) into (+1,-12) at startup: a pure pivot became backwards
    // motion. Use the previously *sent* mean, including any yaw saturation.
    // For a stopped move, limit acceleration in its requested direction but
    // let the signed PID brake immediately in either direction. The previous
    // motion may have had a larger voltage cap, so the unrestricted step must
    // cover its actual baseline as well as this motion's capped demand.
    // Chaining keeps both limits; yaw/rail priority may still reduce drive.
    const double previous_drive = (prev_left_output + prev_right_output) / 2.0;
    const double unrestricted_step = fabs(previous_drive) + max_output;
    const double requested_drive = left_output;
    const double before_slew = clampSymmetric(left_output, max_output);
    left_output = applySlewLimit(
        clampSymmetric(left_output, max_output),
        previous_drive,
        (!exit || dir > 0) ? max_slew_fwd : unrestricted_step,
        (!exit || dir < 0) ? max_slew_rev : unrestricted_step, 10);

    const bool slew_limited = left_output != before_slew;
    const bool voltage_limited = requested_drive != before_slew ||
        fabs(left_output) + fabs(correction_output) > max_output;
    // Overturn logic for sharp turns
    applyOverturnAndMix(left_output, right_output, correction_output, max_output, overturn);

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    prev_left_output = left_output;
    prev_right_output = right_output;
    telemetry.publish(MotionPhase::Pursuit, requested_drive, correction_output,
                      slew_limited,
                      voltage_limited || fabs(left_output) > 12 || fabs(right_output) > 12);
    safety.write(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  if (exit)
  {
    // Use a faster decel rate for the exit ramp so we actually reach 0 V
    const double exit_decel = exitDecel(max_slew_fwd, max_slew_rev, max_output);
    const double ramp_start = pros::millis();
    const double ramp_timeout = 500; // ms safety cap
    while ((fabs(prev_left_output) > 0.15 || fabs(prev_right_output) > 0.15) &&
           pros::millis() - ramp_start < ramp_timeout && safety.running())
    {
      prev_left_output = applySlewLimit(0, prev_left_output, exit_decel, exit_decel, 10);
      prev_right_output = applySlewLimit(0, prev_right_output, exit_decel, exit_decel, 10);
      telemetry.decelerate(prev_left_output, prev_right_output);
      safety.write(prev_left_output, prev_right_output);
      pros::delay(10);
    }
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(mclib::device::BrakeMode::Hold); // Stop at end if required
  }
  // Publish the slew baseline so the next motion picks up where this one left
  // off (zero, on the exit path above).
  state().setPrevOutputs(prev_left_output, prev_right_output);
  safety.setCorrectHeading(a);  // Update shared heading
  state().setTurning(false); // Reset turning state
}
