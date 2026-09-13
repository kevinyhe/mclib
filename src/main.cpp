// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// An example robot program. Nothing in here is part of the library: it is
// what a team's own main.cpp looks like when it uses mclib. Change every port
// and every number to match your robot.
#include "main.h"

using namespace mclib::units::literals;

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

mclib::auton::AutonSelector selector;

// ---------------------------------------------------------------------------
// Autonomous routines. Each is a function the selector can run.
// ---------------------------------------------------------------------------
void leftAuton() {
  moveToPoint(0_in, 24_in, 1, 2_s);
  turnToAngle(90_deg, 1_s);
}

void rightAuton() {
  moveToPoint(0_in, 24_in, 1, 2_s);
  turnToAngle(-90_deg, 1_s);
}

void skillsAuton() {
  driveTo(12_in, 1500_ms);
}

void initialize() {
  // Calibrate before anything reads the IMU. The odometry tracks heading as
  // deltas from the pose it was reset to, while the motion routines steer on
  // the raw IMU rotation; starting the two in different frames leaves every
  // moveToPoint aiming off by a constant.
  imu->reset(true);
  imu->setRotationDeg(0.0);

  // One odometry, on its own task, from here until the program ends. Nothing
  // else in mclib writes the pose.
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  mclib::control::startOdometry(odometry);
  mclib::control::resetOdometry(mclib::Pose2D{});

  // Pick the autonomous on the brain screen or with the controller's left and
  // right buttons. Polling stops by itself when the match starts.
  selector.add("Left", leftAuton);
  selector.add("Right", rightAuton);
  selector.add("Skills", skillsAuton);
  selector.bindController(master);
  selector.loadSelection();
  selector.startPolling();

  // Driver control: arcade on the left stick, with a mild exponential curve
  // and a deadzone, run by the scheduler whenever nothing else owns the drive.
  static std::unique_ptr<Command> default_drive = drive.makeArcadeDriveCommand(
      master, mclib::control::DriveCurveConfig{.deadzone = 0.05, .gain = 5.0});
  CommandScheduler::registerSubsystem(&drive, default_drive.get());
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

void opcontrol() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);
  selector.stopPolling();
  while (true) {
    CommandScheduler::run();
    pros::delay(10);
  }
}
