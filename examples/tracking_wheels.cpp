// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: every way to set up odometry. Each start...() function below is one
// sensor layout. initialize() calls one of them: change that line to the
// layout your robot has. The rest of the file is what every layout needs
// afterwards: checking that odometry started, setting the IMU and the pose
// together, and reading the pose. Change every port and every number to match
// your robot.
#include "main.h"

#include <cmath>
#include <cstdint>

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

// Building the controller binds `chassis` as the drivetrain every motion
// routine runs on and installs this tuning as the one every drive loop reads.
mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    .distance_pid = {0.4, 0.0, 3.0},
    .turn_pid = {0.3, 0.0, 1.5},
    .heading_pid = {0.3, 0.0, 1.5},
});

// ---------------------------------------------------------------------------
// Tracking wheel sensors. Either kind of sensor works on either wheel. Reverse
// a sensor until pushing the robot forward (forward-rolling wheel) or to its
// right (sideways-rolling wheel) counts up.
// ---------------------------------------------------------------------------

// A V5 Rotation sensor. A negative port reverses it.
mclib::device::Rotation vertical_tracker(-6);

// A 3-wire shaft encoder in ADI ports A and B. A third argument of true
// reverses it.
mclib::device::AdiEncoder horizontal_tracker('A', 'B');

// The vertical wheel again, on a PROS device instead of an mclib one, as in a
// program that already has its own sensor objects. See
// startWithYourOwnReaders().
pros::Rotation pros_vertical_tracker(7);

// ---------------------------------------------------------------------------
// The tracking wheels: size, offset, and wheel turns per sensor turn.
//
// The offset is measured from the tracking centre, the point whose pose the
// odometry reports. That is normally the centre of rotation. Offsets are
// signed. For a forward-rolling wheel it is how far the wheel sits to the
// robot's right. For a sideways-rolling wheel it is how far it sits ahead.
// ---------------------------------------------------------------------------

// Measured across with calipers. 1.25 in to the right of the tracking centre.
const mclib::units::TrackingWheel vertical_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), 1.25_in, 1.0};

// Measured with a tape around the tread. 3.5 in behind the tracking centre.
const mclib::units::TrackingWheel horizontal_wheel{
    mclib::units::Wheel::fromCircumference(8.64_in), -3.5_in, 1.0};

// ---------------------------------------------------------------------------
// The layouts. Each returns what startOdometry() returned.
//
//   Layout                 Forward travel     Sideways travel
//   drive encoders only    drive encoders     not measured
//   vertical wheel         that wheel         not measured
//   horizontal wheel       drive encoders     that wheel
//   both wheels            vertical wheel     horizontal wheel
//   accelerometer          either             estimated from the IMU
//
// The heading comes from the IMU in all of them. Drive wheels slip when the
// robot accelerates or is pushed, and a tracking wheel does not. Omni drive
// wheels also slide sideways in every turn. A horizontal wheel measures that;
// the IMU's accelerometer estimates it.
//
// startOdometry() takes the sample period as a last argument. The default,
// 10 ms, matches the motion loops.
// ---------------------------------------------------------------------------

bool startOnDriveEncodersOnly() {
  // No arguments: the heading and drive encoders of the bound drive. Building
  // `drive` bound `chassis`.
  return mclib::control::startOdometry();
}

bool startWithVerticalWheel() {
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  return mclib::control::startOdometry(odometry);
}

bool startWithHorizontalWheel() {
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.horizontal = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(horizontal_tracker), horizontal_wheel};
  return mclib::control::startOdometry(odometry);
}

bool startWithBothWheels() {
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  odometry.horizontal = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(horizontal_tracker), horizontal_wheel};
  return mclib::control::startOdometry(odometry);
}

// No horizontal wheel, so the IMU's accelerometer estimates the sideways
// slide instead. It works with drive encoders alone or with a vertical wheel.
// With drive encoders alone it also reads the IMU's forward axis, so drive
// wheels spinning faster than the robot moves are not counted as travel.
//
// Give the IMU axis that points to the robot's right, and the one that points
// to its front. To find them, show imu->accelerationG(PlusX), PlusY and PlusZ
// on the screen, push the robot by hand to its right, then forward, and see
// which one jumps and which way. A jump down on PlusY makes it MinusY.
//
// imu_forward_in and imu_right_in are where the IMU sits, measured from the
// tracking centre. Spinning at 6 rad/s, an IMU 3 in off the centre reads
// 0.28 g that is the turn, not a slide, and the odometry takes it away using
// these. imu_position_sigma_in is how well you measured them: 0.25 in by
// default.
//
// The accelerometer's zero is learned while the robot stands still after
// startOdometry(). Start odometry before the robot moves, as calibrating the
// IMU needs anyway.
bool startWithAccelerometer() {
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  mclib::control::AccelerometerSensor accelerometer{
      .right_ips2 =
          mclib::control::accelerationReader(*imu, mclib::device::ImuAxis::MinusY)};
  accelerometer.slide.imu_forward_in = 3.0;
  accelerometer.slide.imu_right_in = -2.0;
  odometry.accelerometer = accelerometer;
  return mclib::control::startOdometry(odometry);
}

bool startWithAccelerometerAndDriveEncoders() {
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  mclib::control::AccelerometerSensor accelerometer{
      .right_ips2 =
          mclib::control::accelerationReader(*imu, mclib::device::ImuAxis::MinusY),
      .forward_ips2 =
          mclib::control::accelerationReader(*imu, mclib::device::ImuAxis::PlusX)};
  accelerometer.slide.imu_forward_in = 3.0;
  accelerometer.slide.imu_right_in = -2.0;
  odometry.accelerometer = accelerometer;
  return mclib::control::startOdometry(odometry);
}

// A reader is any function that returns the wheel's total rotation in degrees.
// It must return NaN when the sensor is not reporting: odometry skips a NaN
// sample, and would count PROS_ERR as 21 million degrees of travel.
double prosVerticalTrackerDegrees() {
  const std::int32_t centidegrees = pros_vertical_tracker.get_position();
  return centidegrees == PROS_ERR ? NAN : centidegrees / 100.0;
}

// The same setup built field by field instead of from the chassis.
// heading_deg is required. left_deg and right_deg, the drive encoders, are
// left empty: with a vertical wheel fitted the odometry does not read them.
bool startWithYourOwnReaders() {
  mclib::control::OdometrySetup odometry;
  odometry.heading_deg = [] { return imu->getRotationDeg(); };
  odometry.vertical = mclib::control::TrackingWheelSensor{
      prosVerticalTrackerDegrees, vertical_wheel};
  odometry.horizontal = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(horizontal_tracker), horizontal_wheel};
  return mclib::control::startOdometry(odometry);
}

// ---------------------------------------------------------------------------
// Reading the pose
// ---------------------------------------------------------------------------

constexpr std::uint32_t screen_period_ms = 100;

void showPose() {
  // x, y in inches and theta in radians, all from the same odometry sample.
  // mclib::control::robotState().pose() is the same read without a chassis.
  const mclib::Pose2D pose = chassis.getPose();
  pros::screen::print(pros::E_TEXT_MEDIUM, 0, "x %.1f in   y %.1f in", pose.x,
                      pose.y);
  // The pose heading wraps at 180 deg. The IMU heading keeps counting past a
  // full turn. Whole turns aside, the two match as long as the IMU and the
  // pose were set together.
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "pose heading %.1f deg",
                      radToDeg(pose.theta));
  pros::screen::print(pros::E_TEXT_MEDIUM, 2, "IMU heading %.1f deg",
                      getInertialHeading());
  // Samples skipped because a sensor read NaN. A count that climbs means an
  // unplugged tracking wheel or an IMU that has stopped reporting.
  pros::screen::print(pros::E_TEXT_MEDIUM, 3, "skipped samples %d",
                      static_cast<int>(mclib::control::odometryFaultCount()));
}

// How far a pose built from the drive encoders alone has drifted from the
// odometry pose. With tracking wheels fitted, that is how much the drive
// wheels slipped and slid. With none it stays near zero.
void showDriveEncoderDrift(const mclib::Pose2D& drive_encoder_pose) {
  pros::screen::print(pros::E_TEXT_MEDIUM, 4, "drive encoders off by %.1f in",
                      chassis.getPose().distanceTo(drive_encoder_pose));
}

void initialize() {
  // Calibrate before anything reads the IMU.
  imu->reset(true);

  // Change this line to the layout your robot has. One odometry runs, on its
  // own task, from here until the program ends: a second startOdometry() call
  // returns false and changes nothing. To swap layouts while the program runs,
  // call mclib::control::stopOdometry() first.
  const bool odometry_started = startWithBothWheels();
  // False means the setup was rejected or odometry is already running: every
  // motion would then steer on a pose that never moves.
  if (!odometry_started) {
    master.setText(1, 0, "ODOM NOT RUNNING");
  }

  // Set the IMU and the odometry pose together. The odometry tracks heading as
  // deltas from the pose it was reset to, while the motion routines steer on
  // the raw IMU rotation; starting the two in different frames leaves every
  // moveToPoint aiming off by a constant. Pass the heading with a unit:
  // Pose2D holds radians, so Pose2D(24, 12, 90) faces 90 rad.
  chassis.setPose(24_in, 12_in, 90_deg);

  static std::unique_ptr<Command> default_drive =
      drive.makeArcadeDriveCommand(master);
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

  moveToPoint(48_in, 12_in, 1, 2_s);
  // With a vertical wheel fitted, driveTo() measures its distance along the
  // pose, so wheel slip does not count as progress. Without one it counts
  // drive encoder turns. MotionConfig::drive_distance_source overrides that.
  driveTo(-12_in, 1500_ms);
}

void opcontrol() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);

  // A second odometry, kept in this task and fed the same sensor readings. It
  // uses the drive encoders alone, whatever layout the real one runs on.
  const mclib::control::OdometrySetup drive_encoders =
      mclib::control::odometrySetupFrom(chassis);
  mclib::control::Odometry drive_encoder_odometry(
      mclib::control::odometryConfigFrom(drive_encoders));
  drive_encoder_odometry.reset(chassis.getPose());

  std::uint32_t screen_updated_ms = 0;
  while (true) {
    CommandScheduler::run();

    const mclib::Pose2D drive_encoder_pose = drive_encoder_odometry.update(
        mclib::control::sampleOdometrySensors(drive_encoders));
    if (pros::millis() - screen_updated_ms >= screen_period_ms) {
      screen_updated_ms = pros::millis();
      showPose();
      showDriveEncoderDrift(drive_encoder_pose);
    }
    pros::delay(10);
  }
}
