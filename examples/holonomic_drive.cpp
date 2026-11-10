// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: an X-drive or mecanum robot, in driver control and in autonomous.
//
// A holonomic drive moves in any direction while facing any way, so it has
// its own chassis and controller classes. The blocking motions in
// mclib/control/motion.hpp, such as driveTo() and moveToPoint(), need a
// differential drive: on this robot they return NoDrive and move nothing.
// Autonomous here is built from the controller's commands instead.
//
// On the controller:
//
//   left stick    move, as a direction on the field
//   right stick   turn
//   X             switch the left stick between field and robot directions
//   A             call the way the robot faces now heading 0
//   L2, held      slow, with the left stick as robot directions
//   R1, held      keep facing the same way while the left stick moves
//   Y             sidestep 12 in to the robot's right
//   Down          wiring check, only with no competition switch plugged in
//
// Change every port and every number to match your robot.
#include "main.h"

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <memory>

using namespace mclib::units::literals;
using mclib::device::AnalogAxis;
using mclib::device::BrakeMode;
using mclib::device::DigitalButton;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

auto imu = std::make_shared<mclib::device::Inertial>(15);

// The four corners in the order front left, front right, back left, back
// right, seen from above with the front of the robot at the top. Each is a
// list of ports, and a negative port reverses that motor. Kind::Mecanum is
// the other wheel layout.
mclib::HolonomicChassis chassis(
    {11},
    {-16},
    {13},
    {-18},
    mclib::device::Gearset::Green,
    mclib::holonomic::Kind::XDrive,
    imu);

// This chassis holds no wheel size and gives odometry no motor encoders, so
// the pose comes from the IMU and two tracking wheels. It takes both wheels:
// a robot that strafes travels sideways as much as forward.
mclib::device::Rotation vertical_tracker(-6);
mclib::device::Rotation horizontal_tracker(7);

// Wheel size, offset from the tracking centre, and wheel turns per sensor
// turn. The forward-rolling wheel's offset is how far it sits to the robot's
// right. The sideways-rolling wheel's is how far it sits ahead: this one is
// 3.5 in behind.
const mclib::units::TrackingWheel vertical_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), 0.0_in, 1.0};
const mclib::units::TrackingWheel horizontal_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), -3.5_in, 1.0};

// translation_pid and heading_pid run moveToPose() and moveToPoint(): volts
// per inch left to go, and volts per degree of heading error. A move has
// arrived when both are inside translation_exit and heading_exit, left at
// their defaults here.
//
// follower runs followTrajectory(). Its feedforward is the volts each
// direction needs to hold a speed: kS to get moving, plus kV for every in/s
// or rad/s. Measure the three one at a time: drive forward, strafe, spin in
// place. An X-drive strafes as fast as it drives. Mecanum rollers slip
// sideways, so a mecanum drive needs a larger strafe kV. followTrajectory()
// refuses to run until the forward kV is above zero.
const mclib::HolonomicControllerConfig tuning{
    .translation_pid = {0.4, 0.0, 3.0},
    .heading_pid = {0.3, 0.0, 1.5},
    .max_voltage = 12_V,
    .follower =
        {
            // Extra speed per inch off the planned position, and extra turn
            // rate per radian off the planned heading.
            .translation_kp = 3.0,
            .heading_kp = 4.0,
            .feedforward =
                {
                    .forward = {0.6_V, 0.24_V / mclib::units::inps, {}},
                    .strafe = {0.6_V, 0.24_V / mclib::units::inps, {}},
                    .turn = {0.8_V, 2.6_V / mclib::units::radps},
                },
        },
    // The left stick starts as a direction on the field.
    .field_centric_teleop = true,
};

// The subsystem. Nothing it starts moves the robot unless the scheduler runs:
// it steers from its periodic(), which CommandScheduler::run() calls.
mclib::HolonomicController drive(chassis, tuning);

// ---------------------------------------------------------------------------
// Wiring check. Run it before anything else, with the wheels off the floor.
// Front left, front right, back left, back right: each in turn has to spin
// the way that would push the robot forward. Flip the sign of the port of one
// that does not. After that, chassis.drive(0.3, 0, 0) on the floor has to
// drive straight forward. One wheel wired backward makes it strafe and turn.
// ---------------------------------------------------------------------------
void spinEachWheelForward() {
  constexpr double test_volts = 4.0;
  constexpr std::uint32_t each_wheel_ms = 1000;

  chassis.driveVoltage({.front_left = test_volts});
  pros::delay(each_wheel_ms);
  chassis.driveVoltage({.front_right = test_volts});
  pros::delay(each_wheel_ms);
  chassis.driveVoltage({.back_left = test_volts});
  pros::delay(each_wheel_ms);
  chassis.driveVoltage({.back_right = test_volts});
  pros::delay(each_wheel_ms);
  chassis.stop();
}

// ---------------------------------------------------------------------------
// Driver control
// ---------------------------------------------------------------------------

const BrakeMode driver_brake_mode = BrakeMode::Coast;

const mclib::control::DriveCurveConfig stick_curve{.deadzone = 0.05,
                                                   .gain = 5.0};

double shaped(AnalogAxis axis) {
  return mclib::control::shapeDriveInput(master.analog(axis), stick_curve);
}

// X. The drive command the controller makes reads this setting on every
// pass. Field directions: stick up drives toward field +Y whichever way the
// robot faces. Robot directions: stick up drives the way the robot faces.
void switchStickDirections() {
  mclib::HolonomicControllerConfig config = drive.getConfig();
  config.field_centric_teleop = !config.field_centric_teleop;
  drive.setConfig(config);
  master.setText(0, 0, config.field_centric_teleop ? "FIELD STICKS"
                                                   : "ROBOT STICKS");
}

// A. Field directions are only as good as the heading. Point the robot
// straight away from the driver, then press A.
void callThisHeadingZero() {
  const mclib::Pose2D pose = chassis.getPose();
  chassis.setPose({pose.x, pose.y, 0.0});
}

// The commands below each need the drive, so scheduling one interrupts the
// stick command. When it ends, the scheduler starts the stick command again.
// The scheduler does not own them. Keep them alive for the whole program.

// L2. For lining up: robot directions at 40% power.
std::unique_ptr<Command> drive_slowly = drive.run([] {
  constexpr double scale = 0.4;
  chassis.drive(scale * shaped(AnalogAxis::LeftY),
                scale * shaped(AnalogAxis::LeftX),
                scale * shaped(AnalogAxis::RightX));
});

// R1. The left stick moves the robot on the field and the heading stays where
// it was when the button went down.
QAngle held_heading = 0_deg;
constexpr double turn_per_degree_off = 0.03;

std::unique_ptr<Command> hold_heading = std::make_unique<FunctionalCommand>(
    [] { held_heading = chassis.heading(); },
    [] {
      const double degrees_off = (held_heading - chassis.heading()).deg();
      const double turn =
          std::clamp(turn_per_degree_off * degrees_off, -1.0, 1.0);
      // Field x comes first here. drive() takes forward first.
      chassis.driveFieldCentric(shaped(AnalogAxis::LeftX),
                                shaped(AnalogAxis::LeftY), turn);
    },
    [](bool) { chassis.drive(0.0, 0.0, 0.0); },
    [] { return false; },
    std::initializer_list<Subsystem*>{&drive});

// Y. The target of a make...Command() is fixed when the command is built.
// This one is worked out when the command starts, so the command calls the
// controller itself. moveToPoint() and followTrajectory() can be called the
// same way.
//
// Call them only from a command that needs the drive, as here. Called from
// anywhere else, the stick command stays scheduled and writes the motors
// after the move does on every scheduler pass, so the move never happens.
std::unique_ptr<Command> sidestep_right = std::make_unique<FunctionalCommand>(
    [] {
      // In the robot's frame: 12 in to its right, no turn.
      const mclib::Pose2D one_step_right{12.0, 0.0, 0.0};
      drive.moveToPose(mclib::compose(chassis.getPose(), one_step_right), 2_s);
    },
    [] {},
    [](bool interrupted) {
      if (interrupted) {
        drive.cancel();
      }
    },
    [] { return drive.isSettled(); },
    std::initializer_list<Subsystem*>{&drive});

// makeDriveCommand() takes stick up as field +Y. For a driver who looks along
// another field direction, turn the stick into the robot's frame yourself and
// make this the default command in initialize(). `driver_looks_along` is a
// compass heading: 90_deg for a driver who looks along field +X.
std::unique_ptr<Command> makeDriveCommandFor(QAngle driver_looks_along) {
  return drive.run([driver_looks_along] {
    const mclib::holonomic::RobotFrameInput move =
        mclib::holonomic::fieldToRobot(
            shaped(AnalogAxis::LeftX), shaped(AnalogAxis::LeftY),
            chassis.headingRad() - driver_looks_along.rad());
    chassis.drive(move.forward, move.strafe, shaped(AnalogAxis::RightX));
  });
}

// ---------------------------------------------------------------------------
// Autonomous, from the controller's commands
// ---------------------------------------------------------------------------

// A curve from the start pose. A trajectory has to start where the robot is.
// Keep the top speed under what the drive can do while it strafes and turns
// at once.
mclib::path::Trajectory planSweep() {
  const mclib::path::Path route = mclib::path::generateSpline({
      {24_in, 12_in},
      {36_in, 36_in},
      {60_in, 48_in},
  });
  mclib::path::TrajectoryConstraints limits;
  limits.max_velocity = 30_inps;
  limits.max_acceleration = 60 * mclib::control::inps2;

  const mclib::path::Trajectory sweep =
      mclib::path::Trajectory::generate(route, limits);
  // Empty means the limits or the waypoints were rejected.
  if (sweep.empty()) {
    master.setText(1, 0, "NO TRAJECTORY");
  }
  return sweep;
}

void runToEnd(Command& command) {
  command.schedule();
  while (command.scheduled()) {
    CommandScheduler::run();
    pros::delay(10);
  }
}

// The controller does not say whether a move arrived or ran out of time, so
// measure it.
bool isWithin(QLength distance, const mclib::Pose2D& target) {
  return chassis.getPose().distanceTo(target) <= distance.in();
}

void autonomousWithCommands() {
  const mclib::path::Trajectory sweep = planSweep();
  if (sweep.empty()) {
    return;
  }

  // `static` keeps each command alive after this function returns. The
  // scheduler holds a pointer to a command and does not own it.

  // Drives the curve, turning from the heading it starts with to 90 deg on
  // the way, then settles on the last point. 5 s for all of it.
  static const std::unique_ptr<Command> follow_sweep =
      drive.makeFollowTrajectoryCommand(sweep, 90_deg, 5_s);

  // A Pose2D is inches, inches and radians. There is no overload with units.
  const mclib::Pose2D far_side{96.0, 48.0, (180_deg).rad()};
  // false: do not stop on the pose. The next move starts with the robot still
  // rolling.
  static const std::unique_ptr<Command> cross_field =
      drive.makeMoveToPoseCommand(far_side, 3_s, false);

  // Keeps the heading the robot has when the command starts.
  const mclib::Pose2D last_stop{96.0, 24.0, far_side.theta};
  static const std::unique_ptr<Command> back_off =
      drive.makeMoveToPointCommand(last_stop.x * 1_in, last_stop.y * 1_in, 3_s);

  runToEnd(*follow_sweep);
  runToEnd(*cross_field);
  runToEnd(*back_off);

  if (!isWithin(2_in, last_stop)) {
    master.setText(1, 0, "MISSED TARGET");
  }
}

// ---------------------------------------------------------------------------
// Autonomous without the controller, for a program that does not run the
// scheduler. This loop does what the controller does on each pass of
// followTrajectory(). It ends when the plan does and holds there. The
// controller's settle on the last point is not part of it.
// ---------------------------------------------------------------------------
void followWithoutController(const mclib::path::Trajectory& trajectory,
                             QAngle final_heading) {
  const QAngle start_heading = chassis.getPose().theta * 1_rad;
  const QTime duration = trajectory.duration();
  const double max_volts = tuning.max_voltage.volts();
  const QTime started = mclib::time::now();

  for (QTime elapsed = 0_s; elapsed < duration;
       elapsed = mclib::time::now() - started) {
    // Where the plan puts the robot now, and which way it should face. The
    // heading turns evenly from start to final over the whole plan, the short
    // way round.
    const mclib::control::HolonomicTarget target =
        mclib::control::holonomicTarget(trajectory.sample(elapsed),
                                        start_heading, final_heading, duration,
                                        elapsed);
    const mclib::control::HolonomicVolts volts =
        mclib::control::holonomicFollowStep(chassis.getPose(), target,
                                            tuning.follower);

    // mix() works in fractions of full power. When one wheel would pass full
    // power it scales all four down together, which keeps the direction of
    // travel.
    mclib::holonomic::WheelSpeeds wheels = mclib::holonomic::mix(
        volts.forward / max_volts, volts.strafe / max_volts,
        volts.turn / max_volts, chassis.kind());
    wheels.front_left *= max_volts;
    wheels.front_right *= max_volts;
    wheels.back_left *= max_volts;
    wheels.back_right *= max_volts;
    chassis.driveVoltage(wheels);

    pros::delay(10);
  }
  chassis.stop(BrakeMode::Hold);
}

void autonomousWithoutScheduler() {
  const mclib::path::Trajectory sweep = planSweep();
  if (sweep.empty()) {
    return;
  }
  followWithoutController(sweep, 90_deg);
}

void initialize() {
  // Calibrate before anything reads the IMU.
  imu->reset(true);

  // Built field by field. startOdometry() with no arguments and
  // odometrySetupFrom() both need a differential Chassis.
  mclib::control::OdometrySetup odometry;
  odometry.heading_deg = [] { return chassis.headingDeg(); };
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  odometry.horizontal = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(horizontal_tracker), horizontal_wheel};
  // False means the setup was rejected or odometry is already running: every
  // move would then run to its timeout on a pose that never moves.
  if (!mclib::control::startOdometry(odometry)) {
    master.setText(1, 0, "ODOM NOT RUNNING");
  }

  // x in inches, y in inches, heading in radians. It sets the IMU and the
  // odometry pose together. Heading 0 is field +Y, which is where stick up
  // drives: start the robot facing straight away from the driver.
  chassis.setPose({24.0, 12.0, 0.0});

  static std::unique_ptr<Command> default_drive =
      drive.makeDriveCommand(master, stick_curve);
  CommandScheduler::registerSubsystem(&drive, default_drive.get());
}

void disabled() {
  // No cancel tokens here. They stop the motion.hpp routines, which this
  // robot does not run.
  CommandScheduler::disable();
  chassis.stop();
}

void competition_initialize() {}

void autonomous() {
  // Change this line to autonomousWithoutScheduler() to run the other one.
  autonomousWithCommands();
}

void opcontrol() {
  // Set the brake mode here, not in initialize(). A move that stops leaves
  // the drive on Hold, and a disable leaves it on Brake.
  chassis.setBrakeMode(driver_brake_mode);

  bool move_was_running = false;
  while (true) {
    if (master.getNewPress(DigitalButton::X)) {
      switchStickDirections();
    }
    if (master.getNewPress(DigitalButton::A)) {
      callThisHeadingZero();
    }
    if (master.getNewPress(DigitalButton::L2)) {
      drive_slowly->schedule();
    }
    if (master.getNewRelease(DigitalButton::L2)) {
      drive_slowly->cancel();
    }
    if (master.getNewPress(DigitalButton::R1)) {
      hold_heading->schedule();
    }
    if (master.getNewRelease(DigitalButton::R1)) {
      hold_heading->cancel();
    }
    if (master.getNewPress(DigitalButton::Y)) {
      sidestep_right->schedule();
    }
    if (master.getNewPress(DigitalButton::Down) &&
        !pros::competition::is_connected()) {
      spinEachWheelForward();
    }

    CommandScheduler::run();

    // isActive() is true while the controller runs a move. A move that stops
    // ends on Hold, so hand the driver's brake mode back.
    const bool move_is_running = drive.isActive();
    if (move_was_running && !move_is_running) {
      chassis.setBrakeMode(driver_brake_mode);
    }
    move_was_running = move_is_running;

    pros::delay(10);
  }
}
