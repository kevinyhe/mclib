// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: measure how well the drive's tires grip the field tiles, for
// tests/vexsim/fit_grip.py, which fits vexsim's tire grip to the logs. The
// robot is a 4-motor, 450 rpm drive on 2.75 in omnis with one sideways
// tracking wheel at its centre. Change the ports and numbers to match yours.
//
// 1. Clear 3 by 3 tiles. Put the robot in the middle and run autonomous: about
//    51 s of spins, circles and launches, logged to /usd/grip000.csv.
// 2. Put the robot's front square against a field wall and press A in driver
//    control: 4 s of pushing, logged to /usd/wall000.csv.
// 3. On a computer:
//    python3 tests/vexsim/fit_grip.py grip000.csv wall000.csv --vexsim ../vexsim
//
// Use a charged battery and the tiles you compete on: grip changes with both.
#include "main.h"

#include "mclib/control/characterize.hpp"

using namespace mclib::units::literals;
using mclib::control::MotionResult;

mclib::device::Controller master(mclib::device::ControllerId::Master);

auto imu = std::make_shared<mclib::device::Inertial>(15);

// 36:48 from blue cartridges: the wheels turn 0.75 times per motor turn.
mclib::Chassis chassis({-11, -12}, {13, 14}, mclib::device::Gearset::Blue,
                       mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(2.75_in),
                                                11.5_in, 0.75},
                       imu);

mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{});

// Reversed until pushing the robot to its right counts up, as odometry needs.
mclib::device::Rotation sideways_tracker(-6);

const mclib::control::TrackingWheelSensor sideways{
    mclib::control::encoderReader(sideways_tracker),
    mclib::units::TrackingWheel{mclib::units::Wheel::fromDiameter(2.0_in), 0_in, 1.0}};

bool reached(const char* step_name, MotionResult result) {
  if (result == MotionResult::Reached) {
    return true;
  }
  master.print(2, 0, "%s %s", step_name, mclib::control::toString(result));
  return false;
}

// Logs one routine to /usd/<name>000.csv, then <name>001.csv and so on.
template <class Routine>
void logGrip(const char* name, Routine routine) {
  // Static, because 512 rows do not fit on a task's stack.
  static mclib::telemetry::RowBuffer<512> rows;
  mclib::telemetry::SdCardSink sd_card(name);
  if (!sd_card.isAvailable()) {
    master.setText(2, 0, "NO SD CARD");
    return;
  }
  mclib::telemetry::Logger logger(sd_card, rows, {.period = 10_ms});
  mclib::control::GripLog grip_log(logger, sideways);
  mclib::telemetry::FlushTask card_writer(logger);
  reached(name, routine(grip_log));
  card_writer.stop();
  if (sd_card.writeFailed()) {
    master.setText(2, 0, "SD WRITE FAILED");
    return;
  }
  master.print(2, 0, "%s logged", name);
}

void initialize() {
  imu->reset(true);
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.horizontal = sideways;
  if (!mclib::control::startOdometry(odometry)) {
    master.setText(1, 0, "ODOMETRY OFF");
  }
}

void autonomous() {
  logGrip("grip", [](mclib::control::GripLog& log) {
    // The defaults: spins at 4, 8 and 12 V, circles from 6/3 V to 12/3 V,
    // launches at 12 V, with 1 s at 0 V between runs.
    return mclib::control::characterizeGrip(log);
  });
}

void opcontrol() {
  while (true) {
    if (master.getNewPress(mclib::device::DigitalButton::A)) {
      logGrip("wall", [](mclib::control::GripLog& log) {
        return mclib::control::characterizeWallPush(log, 12_V, 3_s, 1_s);
      });
    }
    pros::delay(20);
  }
}
