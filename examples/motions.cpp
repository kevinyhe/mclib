// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: the blocking motions in mclib/control/motion.hpp, one use case per
// routine. Pick a routine on the brain screen or the controller, then start
// autonomous. Every routine starts from the pose initialize() sets: (0, 0),
// facing 0 deg. Change every port and every number to match your robot.
#include "main.h"

using namespace mclib::units::literals;
using mclib::control::MotionResult;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

auto imu = std::make_shared<mclib::device::Inertial>(15);

// The drive base. Say which measurement you took of the wheel: calipers
// across it (fromDiameter) or a tape around the tread (fromCircumference).
mclib::Chassis chassis(
    {-11, 13, 14},
    {-16, 17, -18},
    mclib::device::Gearset::Blue,
    mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(2.75_in),
                             11.375_in, 1.0},
    imu);

// One tracking wheel, forward-rolling, on a rotation sensor. Leave it out and
// the odometry measures forward travel off the drive encoders instead.
mclib::device::Rotation vertical_tracker(-6);
const mclib::units::TrackingWheel vertical_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), 0.0_in, 1.0};

// Building the controller binds `chassis` as the drivetrain every motion
// routine runs on and installs this tuning as the one every drive loop reads.
// These are the library's defaults, written out so each can be changed. They
// were tuned on one robot: retune the gains for yours.
mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    // Volts per inch left to go. driveTo, moveToPoint, boomerang, curveCircle.
    .distance_pid = {0.4, 0.0, 3.0},
    // Volts per degree left to turn. turnToAngle, turnToPoint, swing.
    .turn_pid = {0.3, 0.0, 1.5},
    // Volts per degree off course, while the robot drives.
    .heading_pid = {0.3, 0.0, 1.5},
    // A motion that stops is finished once its error has stayed inside
    // small_error for small_duration, or inside big_error for big_duration,
    // while changing by less than `derivative` per 10 ms. Inches.
    .distance_exit = {.small_error = 0.5, .big_error = 1.5,
                      .small_duration = 50_ms, .big_duration = 250_ms,
                      .derivative = 5.0},
    // The same rule for turns, in degrees.
    .turn_exit = {.small_error = 1.0, .big_error = 3.0,
                  .small_duration = 50_ms, .big_duration = 250_ms,
                  .derivative = 4.5},
    // The cap for ChassisController::driveDistance() and turnToHeading(). The
    // motions in this file take their cap as an argument: see capVoltage().
    .max_voltage = 12_V,
    // The least voltage that still moves this drive. A motion that uses a
    // minimum reads this one, unless it is passed its own: see
    // setMinVoltage().
    .min_voltage = 1.5_V,
    // Most the drive voltage may change per 10 ms: speeding up and slowing
    // down, forward and backward. 1.0 takes 0.12 s from 0 to 12 V. Lower it if
    // the wheels spin when the robot sets off.
    .slew = {.accel_fwd = 1.0, .decel_fwd = 1.0,
             .accel_rev = 1.0, .decel_rev = 1.0},
    // For motions with exit = false. True keeps the slew limit at the start,
    // or the end, of each one, as a robot that reverses there needs. False
    // drops it there, for a chain that keeps driving the same way.
    .dir_change_start = true,
    .dir_change_end = true,
    // boomerang() caps its voltage on a curve at
    // sqrt(chase_power * radius in inches * 9.8). Lower is slower in curves.
    .chase_power = 10.0,
    // False makes correctHeading() return at once. driveTo() holds its
    // heading either way.
    .heading_correction = true,
    // What driveTo() measures distance with. Auto is the odometry pose when a
    // forward tracking wheel is fitted, as here, and the drive encoders when
    // not. The encoders count wheel slip as distance.
    .drive_distance_source = mclib::control::DistanceSource::Auto,
    // The least voltage a motion that stops uses until it is inside
    // small_error. Set it above the drive's kS, or the last inches crawl.
    .settle_min_voltage = 1.5_V,
    // The exit rule for curveCircle(), in inches along the outer wheel's path.
    .arc_exit = {.small_error = 0.3, .big_error = 0.9,
                 .small_duration = 50_ms, .big_duration = 250_ms,
                 .derivative = 2.25},
    // curveCircle() slows down to keep sideways acceleration under this, in
    // in/s^2. Lower it if the robot slides wide on arcs. 0 turns it off.
    .arc_max_lateral_accel = 60.0,
});

mclib::auton::AutonSelector selector;

// ---------------------------------------------------------------------------
// One routine per motion. Every motion takes a time limit and gives up when
// it runs out.
// ---------------------------------------------------------------------------

// A distance from where the robot is, not a field position.
void driveStraight() {
  driveTo(24_in, 2_s);
  driveTo(-24_in, 2_s);
}

// Headings are field headings, not amounts to turn by. 0 deg is +Y and
// clockwise is positive, so the second call turns the robot 180 deg.
void turnInPlace() {
  turnToAngle(90_deg, 1_s);
  turnToAngle(-90_deg, 1500_ms);
}

void facePoint() {
  turnToPoint(24_in, 24_in, 1, 1_s);
  // -1 points the back of the robot at the point.
  turnToPoint(24_in, 24_in, -1, 1_s);
}

void driveToPoint() {
  moveToPoint(24_in, 36_in, 1, 2500_ms);
  // -1 drives there backward.
  moveToPoint(0_in, 0_in, -1, 2500_ms);
}

// boomerang() arrives at a point facing a chosen way. The 0.6 is the lead, 0
// to 1: larger swings wider into the point, and 0 drives to the point and
// then turns.
void driveToPose() {
  boomerang(24_in, 36_in, 1, 90_deg, 0.6, 3_s);
  // Backward. The heading is still the way the front of the robot ends up.
  boomerang(0_in, 0_in, -1, 0_deg, 0.6, 3_s);
}

// One side of the drive holds still and the other side drives.
void swingTurn() {
  swing(90_deg, 1, 1500_ms);
  // -1 swings backward.
  swing(0_deg, -1, 1500_ms);
}

void driveArcs() {
  // A quarter circle to the right, ending at (24, 24) facing 90 deg. A
  // positive radius puts the circle's centre on the robot's right.
  curveCircle(90_deg, 24_in, 3_s);
  // Back along the same arc to the start. For a reverse arc the radius and
  // the heading change need opposite signs: here +24 in, and 90 deg down to 0.
  curveCircleReverse(0_deg, 24_in, 3_s);
  // A negative radius puts the centre on the left. Ends at (-24, 24).
  curveCircle(-90_deg, -24_in, 3_s);
}

void squareOnWall() {
  // This example's robot starts with its back 4 in off the perimeter wall, so
  // touching that wall puts its centre at y = -4 in. Measure your own.
  const QLength y_at_wall = -4_in;
  // The wall fixes y and the heading. It says nothing about x.
  const QLength x_now = chassis.getPose().x * 1_in;

  // -3 V backs into the wall until the motors stall, then sets the pose and
  // the IMU heading. The last two arguments are what counts as a stall: at
  // least this much motor current, at less than this motor speed. Push too
  // hard and the wheels slip against the wall, keep turning, and never count.
  // Use the lowest voltage that moves the robot, and a current under what the
  // motors draw stalled at it. The defaults, 2500 mA and 5 RPM, suit a push
  // near 6 V on a drive that does not slip.
  const bool touched_wall =
      wallReset(x_now, y_at_wall, 0_deg, -3_V, 2_s, 1000_mA, 5_rpm);
  if (!touched_wall) {
    // No stall inside 2 s. The pose was left as it was.
    master.setText(2, 0, "NO WALL");
    return;
  }
  driveTo(12_in, 1500_ms);

  // keep_current_heading resets the position and leaves the IMU alone.
  wallReset(x_now, y_at_wall, keep_current_heading, -3_V, 2_s, 1000_mA, 5_rpm);
}

// ---------------------------------------------------------------------------
// Voltage limits
// ---------------------------------------------------------------------------

// The cap comes after `exit`, so `true` (stop at the end) is written out.
void capVoltage() {
  driveTo(24_in, 3_s, true, 6_V);
  turnToAngle(90_deg, 2_s, true, 6_V);
  swing(0_deg, -1, 2_s, true, 6_V);
  curveCircle(90_deg, 24_in, 4_s, true, 6_V);
  boomerang(0_in, 36_in, -1, 0_deg, 0.6, 4_s, true, 8_V);

  // The `false` after the cap is `overturn`. When steering and driving
  // together ask for more than the cap, true (the default) cuts the driving
  // so the robot still turns. False scales both down.
  moveToPoint(0_in, 0_in, -1, 4_s, true, 8_V, false);
}

// The argument after the cap is the least voltage the motion commands.
void setMinVoltage() {
  // Without it, a motion that stops holds MotionConfig::settle_min_voltage
  // until it is inside small_error, and the PID finishes from there. With
  // it, driveTo() keeps 2 V on all the way in. That helps a drive with so
  // much friction that it stalls just short. Too high and the robot rocks
  // across the target until the time limit.
  driveTo(24_in, 3_s, true, 12_V, 2_V);
  // moveToPoint() lets go of the minimum inside small_error, so it can stop.
  moveToPoint(0_in, 0_in, -1, 3_s, true, 12_V, true, 2_V);

  // turnToPoint always uses a minimum: MotionConfig::min_voltage. This
  // raises it for one turn.
  turnToPoint(24_in, 24_in, 1, 1_s, 2.5_V);
  // 0 V turns the minimum off.
  turnToPoint(0_in, 24_in, 1, 1_s, 0_V);
}

// ---------------------------------------------------------------------------
// Keep moving into the next motion
// ---------------------------------------------------------------------------

// exit = false ends a motion as the robot reaches the target, with the drive
// still powered, and the next call takes over. A motion that runs out of time
// brakes anyway, so check each result. The last motion stops.
void chainThroughPoints() {
  if (moveToPoint(0_in, 24_in, 1, 2_s, false) != MotionResult::Reached) {
    return;
  }
  // The last argument hands off 6 in before the point, so the robot cuts the
  // corner instead of driving into it. The three before it are the defaults:
  // 12 V cap, overturn on, and -1 V for MotionConfig::min_voltage.
  if (moveToPoint(24_in, 48_in, 1, 2_s, false, 12_V, true, -1_V, 6_in) !=
      MotionResult::Reached) {
    return;
  }
  moveToPoint(48_in, 48_in, 1, 2_s);
}

void chainStraightIntoArc() {
  if (driveTo(18_in, 2_s, false) != MotionResult::Reached) {
    return;
  }
  if (curveCircle(90_deg, 18_in, 3_s, false) != MotionResult::Reached) {
    return;
  }
  driveTo(18_in, 2_s);
}

// ---------------------------------------------------------------------------
// How a motion ended
// ---------------------------------------------------------------------------

// Line 2 of the controller is free: the selector writes line 0 and the
// odometry warning line 1.
bool reached(const char* step_name, MotionResult result) {
  if (result == MotionResult::Reached) {
    return true;
  }
  master.print(2, 0, "%s %s", step_name, mclib::control::toString(result));
  return false;
}

void stopAtTheFirstFailure() {
  if (!reached("drive", driveTo(36_in, 2_s))) {
    return;
  }
  if (!reached("turn", turnToAngle(90_deg, 1_s))) {
    return;
  }
  reached("point", moveToPoint(36_in, 36_in, 1, 2_s));
}

void actOnEachResult() {
  switch (moveToPoint(24_in, 36_in, 1, 2_s)) {
    case MotionResult::Reached:
      break;
    case MotionResult::TimedOut:
      // Short of the point, or on it but still braking when the 2 s ran out.
      // Leave margin in every time limit. This routine backs off, in case
      // something was in the way.
      driveTo(-6_in, 1_s);
      return;
    case MotionResult::Cancelled:
    case MotionResult::Disabled:
      return;
    case MotionResult::InvalidValue:
      // A sensor or the pose read NaN, or an argument was out of range.
      master.setText(2, 0, "CHECK SENSORS");
      return;
    case MotionResult::NoDrive:
      // No ChassisController was built, so the motions have no drive.
      master.setText(2, 0, "NO DRIVE");
      return;
  }
  turnToAngle(180_deg, 1_s);
}

// ---------------------------------------------------------------------------
// Tuning and heading
// ---------------------------------------------------------------------------

void changeTuningBetweenMotions() {
  const mclib::ChassisControllerConfig usual_tuning = drive.getConfig();

  mclib::ChassisControllerConfig carrying_tuning = usual_tuning;
  carrying_tuning.slew = {.accel_fwd = 0.3, .decel_fwd = 0.3,
                          .accel_rev = 0.3, .decel_rev = 0.3};
  carrying_tuning.turn_pid = {0.35, 0.0, 2.0};
  carrying_tuning.distance_exit.small_error = 1.0;

  // Change the tuning between motions, never during one: a running motion
  // reads some of it on every tick.
  drive.setConfig(carrying_tuning);
  driveTo(36_in, 3_s);
  turnToAngle(90_deg, 1500_ms);

  drive.setConfig(usual_tuning);
  driveTo(-12_in, 2_s);
}

// driveTo() steers toward the heading the last turn was sent to, 90 deg here,
// not the heading that turn happened to stop on. What the turn left over is
// corrected during the drive.
void driveOnTheLastHeading() {
  turnToAngle(90_deg, 1_s);
  driveTo(24_in, 2_s);
}

void startFacingAnotherWay() {
  chassis.setPose(48_in, -60_in, 180_deg);
  // setPose() moves the IMU and the odometry. It does not change the heading
  // driveTo() holds, which is 0 deg until a motion has run. Without this line
  // the drive below turns toward 0 deg instead of going straight.
  mclib::control::robotState().setCorrectAngleDeg(180.0);
  driveTo(24_in, 2_s);
}

// correctHeading() holds the last heading while no motion is running: here,
// through the pause, when a mechanism or another robot could turn the robot.
void holdHeadingBetweenMotions() {
  // It never returns on its own, so it gets a task.
  pros::Task heading_hold([] { correctHeading(); });

  driveTo(24_in, 2_s);
  pros::delay(500);
  driveTo(-24_in, 2_s);

  // It stops on its own token, not the motions'. disabled() sets both.
  mclib::control::requestCancel(mclib::control::CancelToken::HeadingCorrection);
  heading_hold.join();
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);
}

void initialize() {
  // Calibrate before anything reads the IMU.
  imu->reset(true);

  // One odometry, on its own task, from here until the program ends. Nothing
  // else in mclib writes the pose.
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  // False means the setup was rejected or odometry is already running: every
  // motion would then steer on a pose that never moves. Say so, on line 1: the
  // selector below writes line 0.
  if (!mclib::control::startOdometry(odometry)) {
    master.setText(1, 0, "ODOM NOT RUNNING");
  }
  // Set the IMU and the odometry pose together. The odometry tracks heading as
  // deltas from the pose it was reset to, while the motion routines steer on
  // the raw IMU rotation; starting the two in different frames leaves every
  // moveToPoint aiming off by a constant.
  chassis.setPose(0_in, 0_in, 0_deg);

  selector.add("driveTo", driveStraight);
  selector.add("turnToAngle", turnInPlace);
  selector.add("turnToPoint", facePoint);
  selector.add("moveToPoint", driveToPoint);
  selector.add("boomerang", driveToPose);
  selector.add("swing", swingTurn);
  selector.add("curveCircle", driveArcs);
  selector.add("wallReset", squareOnWall);
  selector.add("voltage cap", capVoltage);
  selector.add("min voltage", setMinVoltage);
  selector.add("chain points", chainThroughPoints);
  selector.add("chain arc", chainStraightIntoArc);
  selector.add("stop on fail", stopAtTheFirstFailure);
  selector.add("each result", actOnEachResult);
  selector.add("change tuning", changeTuningBetweenMotions);
  selector.add("last heading", driveOnTheLastHeading);
  selector.add("start at 180", startFacingAnotherWay);
  selector.add("heading hold", holdHeadingBetweenMotions);
  // Three columns: 18 buttons are too small to press in the default two.
  selector.setLayout({.columns = 3});
  selector.bindController(master);
  selector.loadSelection();
  selector.startPolling();
}

void disabled() {
  mclib::control::requestCancel(mclib::control::CancelToken::Motion);
  mclib::control::requestCancel(mclib::control::CancelToken::HeadingCorrection);
  CommandScheduler::disable();
  chassis.stop();
}

void competition_initialize() {}

void autonomous() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);
  selector.stopPolling();
  selector.runSelected();
}

void opcontrol() {}
