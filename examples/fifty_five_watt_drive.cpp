// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: a drive with 11 W and 5.5 W motors, such as the 55 W drive
// Override allows (R11): four 11 W motors and two 5.5 W motors.
//
//   - Gear every motor to turn the wheels at the same speed at full power. A
//     5.5 W motor tops out at 200 rpm and a blue 11 W motor at 600 rpm, so on
//     a blue drive each 5.5 W motor needs three times the speed-up of the
//     11 W motors. Next to green 11 W motors it shares their gearing.
//   - List the 5.5 W motors in their own lists. mclib then reads them on the
//     11 W motors' shaft and cuts their voltage to 8/12, so both kinds run
//     at the same fraction of full speed.
//   - Check the gearing on the robot: drive forward and read the motor speeds
//     printed on the PROS terminal. Every speed on a side should match.
//   - Check the 8/12 voltage cut on the robot: put it on a stand with the
//     wheels off the ground and press X. The drive runs at 3, 6, 9 and 12 V
//     and prints each motor's current. If the 5.5 W motors draw several times
//     what the 11 W ones do below 12 V, the 5.5 W motors run ahead of them.
//
// Change every port and every number to match your robot.
#include "main.h"

#include <cstdio>
#include <memory>
#include <vector>

using namespace mclib::units::literals;

mclib::device::Controller master(mclib::device::ControllerId::Master);

auto imu = std::make_shared<mclib::device::Inertial>(15);

// Two blue 11 W motors a side through 36:48, which is 450 rpm at the wheels,
// and one 5.5 W motor a side geared to the same 450 rpm. The gear ratio is
// wheel turns per turn of an 11 W motor's shaft.
mclib::Chassis chassis(
    {-1, 2},
    {3, -4},
    mclib::device::Gearset::Blue,
    {-5},
    {6},
    mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(3.25_in),
                             11.5_in, 36.0 / 48.0},
    imu);

mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    .distance_pid = {0.4, 0.0, 3.0},
    .turn_pid = {0.3, 0.0, 1.5},
    .heading_pid = {0.3, 0.0, 1.5},
});

// A group that is not a drive takes the same three lists: an intake with one
// 11 W motor and one 5.5 W motor geared to the same roller speed.
mclib::device::MotorGroup intake({7}, mclib::device::Gearset::Blue, {8});

void printSide(const char* name, mclib::device::MotorGroup& side) {
  std::printf("%s rpm:", name);
  for (const double rpm : side.getActualVelocities()) std::printf(" %6.1f", rpm);
  std::printf("\n");
}

void printCurrents(const char* name, mclib::device::MotorGroup& side) {
  std::printf("%s mA:", name);
  for (const double milliamps : side.getCurrentDraws()) std::printf(" %6.0f", milliamps);
  std::printf("\n");
}

// Blocks for 8 s. The wheels must be off the ground.
void runStandTest() {
  for (const double volts : {3.0, 6.0, 9.0, 12.0}) {
    chassis.setDriveVoltage(volts, volts);
    pros::delay(2000);
    std::printf("%.0f V\n", volts);
    printSide("left ", chassis.leftMotors());
    printCurrents("left ", chassis.leftMotors());
    printSide("right", chassis.rightMotors());
    printCurrents("right", chassis.rightMotors());
  }
  chassis.stop(mclib::device::BrakeMode::Coast);
}

void initialize() {
  imu->reset(true);
  mclib::control::startOdometry();

  static auto default_drive = drive.makeArcadeDriveCommand(master);
  CommandScheduler::registerSubsystem(&drive, default_drive.get());
}

void autonomous() {
  moveToPoint(0_in, 24_in, 1, 2_s);
  turnToAngle(90_deg, 1_s);
}

void opcontrol() {
  std::uint32_t last_print_ms = 0;
  while (true) {
    CommandScheduler::run();
    intake.setPercent(master.getDigital(mclib::device::DigitalButton::R1) ? 1.0 : 0.0);
    if (master.getDigital(mclib::device::DigitalButton::X)) {
      runStandTest();
      while (master.getDigital(mclib::device::DigitalButton::X)) pros::delay(10);
    }
    // A 5.5 W motor in an 11 W list reads a third of the others on a blue
    // drive, and mclib prints which port it is once.
    if (pros::millis() - last_print_ms >= 500) {
      last_print_ms = pros::millis();
      printSide("left ", chassis.leftMotors());
      printSide("right", chassis.rightMotors());
    }
    pros::delay(10);
  }
}
