// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: autonomous routines built step by step with mclib::auton::Routine,
// and the selector that picks one before the match. Every kind of step and
// step option, commands that run beside the steps, mechanisms in a routine, a
// routine exported from the path planner, and saving the pick to the SD card.
// Every routine starts from (0, 0) facing 0 deg. Change every port and every
// number to match your robot.
#include "main.h"

#include <optional>

using namespace mclib::units::literals;
using mclib::auton::Point;
using mclib::control::MotionResult;
using mclib::device::DigitalButton;

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
mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    .distance_pid = {0.4, 0.0, 3.0},
    .turn_pid = {0.3, 0.0, 1.5},
    .heading_pid = {0.3, 0.0, 1.5},
});

// What a followTrajectory step needs. examples/trajectories.cpp shows where
// each number comes from.
const mclib::control::RamseteConfig follower_config{
    .gains = {.b = 50.0, .zeta = 0.7},
    .feedforward = {.kS = 0.7_V,
                    .kV = 0.13_V / mclib::units::inps,
                    .kA = 0.02_V / mclib::control::inps2},
    .track_width = 13.5_in,
};

// Faces forward, to measure the gap to whatever is ahead.
mclib::device::Distance front_distance(9);

// No reading means the sensor is unplugged, and a robot that cannot see ahead
// stops.
bool somethingIsCloseAhead() {
  const std::optional<QLength> gap = front_distance.distance();
  return !gap.has_value() || *gap < 3_in;
}

// ---------------------------------------------------------------------------
// Mechanisms: a cascade lift and a pneumatic claw. Each is a subsystem:
// initialize() registers it, and a routine uses the commands it makes as
// steps.
// ---------------------------------------------------------------------------

// The lift reads its height from its three motors, in degrees turned since
// initialize() zeroed them, so start the program with the lift fully down.
// The gains are placeholders. Tune them on your lift.
mclib::device::MotorGroup lift_motors({-3, 4, 5},
                                      mclib::device::Gearset::Green);
mclib::mechanism::PositionMechanism lift(
    [] { return lift_motors.getAveragePositionDeg(); },
    [](double volts) { lift_motors.setVoltage(volts); },
    mclib::mechanism::PositionMechanismConfig{.kp = 0.15,
                                              .max_voltage = 12.0,
                                              .small_error = 5.0,
                                              .big_error = 15.0});

// Lift heights, in motor degrees up from fully down. The numbers are this
// example's: measure yours. "Over" carries a held stack clear of the Goal,
// and "on" sets it down on the Goal.
const double lift_down = 0.0;
const double lift_off_the_floor = 20.0;
const double lift_on_neutral_goal = 200.0;
const double lift_over_neutral_goal = 320.0;

// One piston closes the jaws. On this example's robot they close when it
// extends.
mclib::mechanism::PneumaticSubsystem claw({'A'});
const bool claw_closed = true;
const bool claw_open = false;

// ---------------------------------------------------------------------------
// Routines. Globals, built once in initialize(). The scheduler keeps a pointer
// to a routine while it runs, and the field can end autonomous() in the
// middle of one. A routine local to autonomous() would be gone by then.
// ---------------------------------------------------------------------------
mclib::auton::Routine every_motion_step(drive);
mclib::auton::Routine beside_the_steps(drive);
mclib::auton::Routine with_mechanisms(drive);

mclib::auton::AutonSelector selector;

// The path planner's "Export C++" writes this function to
// src/autons/left_side.cpp. main.cpp only declares it and adds it to the
// selector.
void runLeftSide();

// A stand-in, so this example builds without the planner's file. The planner
// writes the real one: delete this when you add yours.
void runLeftSide() {
  chassis.setPose(-48_in, -60_in, 0_deg);
  mclib::control::robotState().setCorrectAngleDeg(0.0);
  if (driveTo(24_in, 2_s) != MotionResult::Reached) {
    return;
  }
  turnToAngle(90_deg, 1_s);
}

// A selector entry is any function. This one calls the blocking motions
// itself, with no Routine.
void runPlainCalls() {
  if (driveTo(24_in, 2_s) != MotionResult::Reached) {
    return;
  }
  turnToAngle(90_deg, 1_s);
}

// Every motion step and every step option. It runs for longer than 15 s: it
// is a tour, not a match autonomous. An option applies to the step written
// just above it.
void buildEveryMotionStep() {
  mclib::path::TrajectoryConstraints limits;
  limits.max_velocity = 48_inps;
  limits.max_acceleration = 96 * mclib::control::inps2;
  limits.max_lateral_acceleration = 60 * mclib::control::inps2;
  limits.track_width = follower_config.track_width;
  // The step keeps its own copy of the trajectory, so a local is enough. Its
  // first waypoint is where the step before it leaves the robot.
  const mclib::path::Trajectory curve = mclib::path::Trajectory::generate(
      mclib::path::generateSpline({
          mclib::path::Waypoint{36_in, 30_in},
          mclib::path::Waypoint{36_in, 18_in},
          mclib::path::Waypoint{24_in, 6_in},
          mclib::path::Waypoint{12_in, 6_in},
      }),
      limits);

  every_motion_step
      // Backs into the perimeter wall behind the robot until the motors stall,
      // then sets the pose. This robot's centre is at y = -4 in when it
      // touches.
      // A push gentle enough that the wheels stop instead of slipping, and a
      // stall current under what the motors draw at it.
      .wallReset(0_in, -4_in, 0_deg, 2_s)
          .withDrivePower(-3_V)
          .withCurrentThreshold(1000_mA)
          .withVelocityThreshold(5_rpm)
      .driveTo(28_in, 2_s)
          .withMaxVoltage(10_V)
          .withMinVoltage(2_V)
      .turnToAngle(90_deg, 1_s)
      // Does not stop at the point. The next step takes over as the robot
      // reaches it.
      .moveToPoint(Point{24_in, 24_in}, 2_s)
          .withoutExit()
      // Does not stop either, and hands over 6 in before the point.
      .moveToPoint(Point{48_in, 36_in}, 2_s)
          .withEarlyExit(6_in)
          // When steering and driving together ask for more than the voltage
          // cap, scale both down instead of cutting the driving first.
          .withoutOverturn()
      // Arrives facing 0 deg. 0.6 is the lead: larger swings wider.
      .boomerang(Point{60_in, 60_in}, 1, 0_deg, 0.6, 3_s)
      // -1 points the back of the robot at the point, then drives there
      // backward.
      .turnToPoint(Point{36_in, 60_in}, 1_s)
          .withDirection(-1)
      .moveToPoint(Point{36_in, 60_in}, 2500_ms)
          .withDirection(-1)
      .swing(180_deg, 1, 1500_ms)
      .curveCircle(270_deg, 18_in, 3_s)
      // Back along the same arc. curveCircle(...).reversed() is the same step.
      .curveCircleReverse(180_deg, 18_in, 3_s)
      .moveToPoint(36_in, 30_in, 1, 2500_ms)
      // Only the voltage cap, withoutExit() and withEarlyExit() apply to a
      // trajectory step.
      .followTrajectory(curve, follower_config, curve.duration() + 3_s)
          .withMaxVoltage(10_V)
      // driveDistance and turnToHeading run inside the scheduler's loop, not
      // on a task of their own. They need the drive registered, as
      // initialize() does. withoutStop() goes on without holding the drive.
      .driveDistance(12_in, 2_s)
          .withoutStop()
      .turnToHeading(0_deg, 1_s);
}

// Commands that run while the steps go on. The robot takes a stack and
// carries it to a Goal, and the lift never makes it wait.
void buildBesideTheSteps() {
  beside_the_steps
      // trigger() starts a command and goes straight to the next step. This
      // command never finishes: it holds the lift down until another lift
      // trigger replaces it or the routine ends.
      .trigger(lift.makeStateCommand(lift_down))
      // until() ends a motion early: the drive brakes as soon as the stack is
      // close ahead. It only compiles on a motion step.
      .moveToPoint(Point{0_in, 36_in}, 1, 2_s)
          .until(somethingIsCloseAhead)
      // Holds the heading whenever no motion step is running: here, while the
      // claw closes.
      .trigger(drive.makeCorrectHeadingCommand())
      .then(claw.makeSetCommand(claw_closed))
      .wait(200_ms)
      // Replaces the lift trigger above.
      .trigger(lift.makeStateCommand(lift_off_the_floor))
      .turnToAngle(90_deg, 1_s)
      // A trigger() written after a motion starts when the motion ends. These
      // two start during it: after 18 in of travel, and 6 in before the end.
      .moveToPoint(Point{36_in, 36_in}, 1, 2500_ms)
          .atDistance(18_in, lift.makeStateCommand(lift_over_neutral_goal))
          .atDistanceFromEnd(6_in, lift.makeStateCommand(lift_on_neutral_goal))
      .wait(200_ms)
      .then(claw.makeSetCommand(claw_open))
      .wait(200_ms)
      // And this one starts 300 ms into the step.
      .driveTo(-8_in, 2_s)
          .atTime(300_ms, lift.makeStateCommand(lift_down));
}

// The same job with then(), which runs a command and waits for it to finish.
// Give it commands that finish. One that never does, such as
// lift.makeStateCommand(), belongs in trigger().
//
// then() does not stop a trigger that is still running. A lift trigger that
// never finishes would go on driving the lift against every then() that moves
// it. The one lift trigger here finishes, and waitUntil() waits for it.
void buildWithMechanisms() {
  with_mechanisms
      .driveTo(24_in, 2_s)
      .then(claw.makeSetCommand(claw_closed))
      // Gives the jaws time to close. Time your own piston.
      .wait(200_ms)
      // The lift rises while the robot drives to the Goal. 2000 is the lift's
      // own time limit in ms.
      .trigger(lift.makeMoveToCommand(lift_over_neutral_goal, 2000.0))
      .driveTo(24_in, 2_s)
      // Waits for the lift. After 2 s the routine goes on anyway, and
      // timedOutWaits() counts it.
      .waitUntil([] { return lift.atTarget(); }, 2_s)
      // Goes on when the lift is there, or after its 2000 ms.
      .then(lift.makeMoveToCommand(lift_on_neutral_goal, 2000.0))
      .then(claw.makeSetCommand(claw_open))
      .wait(200_ms)
      .runOnce([] { master.setText(2, 0, "STACK PLACED"); })
      .driveTo(-12_in, 2_s)
      .then(lift.makeMoveToCommand(lift_down, 2000.0));
}

// runBlocking() runs the scheduler until the routine has finished. Each count
// below also prints a line on the terminal when it happens.
void runAndReport(mclib::auton::Routine& routine) {
  routine.runBlocking();
  if (routine.timedOutWaits() > 0) {
    master.print(2, 0, "%u WAITS GAVE UP",
                 static_cast<unsigned>(routine.timedOutWaits()));
  }
  if (routine.missedEvents() > 0) {
    master.print(2, 0, "%u NOT STARTED",
                 static_cast<unsigned>(routine.missedEvents()));
  }
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

  // Driver control: arcade on the left stick, with a mild exponential curve
  // and a deadzone, run by the scheduler whenever nothing else owns the drive.
  static std::unique_ptr<Command> default_drive = drive.makeArcadeDriveCommand(
      master, mclib::control::DriveCurveConfig{.deadzone = 0.05, .gain = 5.0});
  CommandScheduler::registerSubsystem(&drive, default_drive.get());

  // A mechanism's default command runs whenever no other command uses it.
  lift_motors.tarePosition();
  lift.setName("lift");
  lift.setDefaultCommand(lift.makeStopCommand());
  lift.registerSelf();
  claw.setName("claw");
  claw.setDefaultCommand(claw.idleCommand());
  claw.registerSelf();

  buildEveryMotionStep();
  buildBesideTheSteps();
  buildWithMechanisms();
  // A routine with a configuration error runs none of its steps. The
  // terminal says what the error is.
  if (every_motion_step.hasConfigurationError() ||
      beside_the_steps.hasConfigurationError() ||
      with_mechanisms.hasConfigurationError()) {
    master.setText(2, 0, "ROUTINE ERROR");
  }

  // The names go on the brain screen buttons and on line 0 of the
  // controller. Keep them to 15 characters.
  const int left_side = selector.add("Left side", runLeftSide);
  selector.add("Every step", [] { runAndReport(every_motion_step); });
  selector.add("Beside steps", [] { runAndReport(beside_the_steps); });
  selector.add("Mechanisms", [] { runAndReport(with_mechanisms); });
  selector.add("Plain calls", runPlainCalls);
  // An entry with no function. The robot stays still, and runSelected()
  // returns false.
  selector.add("Do nothing", nullptr);

  // The defaults, except for three columns instead of two. `top` leaves room
  // for the PROS status bar.
  selector.setLayout({.screen_width = 480,
                      .screen_height = 240,
                      .top = 32,
                      .columns = 3,
                      .padding = 8});
  // Up and Down step through the list. Without the last two arguments it is
  // Left and Right.
  selector.bindController(master, DigitalButton::Up, DigitalButton::Down);

  // The pick saved on the SD card. False means no card, no saved file, or a
  // saved name that is no longer in the list. The selection is then left
  // where it was.
  if (!selector.loadSelection()) {
    master.setText(2, 0, "NO SAVED PICK");
    selector.select(left_side);
  }
  // Starts a task that draws the buttons and reads touches and the two
  // controller buttons every 50 ms. It ends when autonomous starts. To poll
  // from a loop of your own instead, call selector.poll() there.
  selector.startPolling();
}

void disabled() {
  mclib::control::requestCancel(mclib::control::CancelToken::Motion);
  mclib::control::requestCancel(mclib::control::CancelToken::HeadingCorrection);
  CommandScheduler::disable();
  chassis.stop();

  // The selector does not save the pick by itself. This saves it each time it
  // changes while the robot waits for the match, so a program restart keeps
  // it. PROS ends this loop when the robot is enabled.
  int last_pick_seen = -1;
  while (true) {
    if (selector.selected() != last_pick_seen) {
      last_pick_seen = selector.selected();
      if (!selector.saveSelection()) {
        master.setText(2, 0, "PICK NOT SAVED");
      }
    }
    pros::delay(100);
  }
}

void competition_initialize() {}

void autonomous() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);
  selector.stopPolling();
  // False for "Do nothing", and for a selector with no entries.
  if (!selector.runSelected()) {
    master.setText(2, 0, "NO ROUTINE RUN");
  }
}

void opcontrol() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);
  selector.stopPolling();
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}
