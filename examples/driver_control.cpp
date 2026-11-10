// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: every way to drive the robot in opcontrol.
//
//   - Stick shaping: deadzone, curve, minimum output and slew.
//   - The drive commands the library makes: arcade on two sticks or on one,
//     tank, and curvature.
//   - Mixing the sticks yourself, for what those commands do not cover.
//   - Commands that borrow the drive from the sticks: hold position, and two
//     driver assists.
//   - Brake modes.
//   - The same robot with no ChassisController and no scheduler.
//
// Upload it and let the driver compare. On the controller:
//
//   Left / Right   previous / next drive style, named on the screen
//   B, held        hold position
//   A              turn to face heading 0
//   Y              back up 6 in
//   L2, held       slow mode, in "Own: slow mode"
//   R1, held       turn in place, in "Own: quick turn"
//   Up / Down      turn curve gain, in the three "Own:" styles that turn
//
// Change every port and every number to match your robot.
#include "main.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <initializer_list>
#include <memory>
#include <utility>

using namespace mclib::units::literals;
using mclib::device::AnalogAxis;
using mclib::device::BrakeMode;
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

// Named, because the last section of this file installs the same tuning
// without a ChassisController.
const mclib::ChassisControllerConfig tuning{
    .distance_pid = {0.4, 0.0, 3.0},
    .turn_pid = {0.3, 0.0, 1.5},
    .heading_pid = {0.3, 0.0, 1.5},
};

// Building the controller binds `chassis` as the drivetrain every motion
// routine runs on and installs this tuning as the one every drive loop reads.
mclib::ChassisController drive(chassis, tuning);

// What the motors do when they are told to stop: Coast, Brake or Hold.
const BrakeMode driver_brake_mode = BrakeMode::Coast;

// ---------------------------------------------------------------------------
// Stick shaping. A stick reading is a fraction from -1 to 1. It goes through
// these four steps, in this order, before it reaches the motors.
// ---------------------------------------------------------------------------
const mclib::control::DriveCurveConfig forward_curve{
    // Readings closer to centre than this count as zero. Raise it until the
    // robot stops creeping with the sticks released.
    .deadzone = 0.05,
    // 0 is a straight line. Higher gives finer control near the centre and
    // still reaches full power at full stick.
    .gain = 5.0,
    // The least power any stick outside the deadzone commands. Set it just
    // under what starts the robot rolling. 0 turns it off.
    .min_output = 0.06,
    // The most the output may change in one 10 ms pass. 0.1 takes 100 ms from
    // rest to full power. 0 turns it off.
    .slew_per_tick = 0.1,
};

// Turning usually wants a stronger curve than driving. The fields left out
// keep their defaults: a 0.05 deadzone, and the rest off.
const mclib::control::DriveCurveConfig turn_curve{.gain = 10.0};

// ---------------------------------------------------------------------------
// Mixing the sticks yourself. Each function below is one pass of a drive
// loop: read the sticks, shape them, write the motors. Call it every 10 ms.
// ---------------------------------------------------------------------------

// A DriveCurve is the same shaping plus the slew limit, which has to remember
// its last output. One per stick axis.
mclib::control::DriveCurve forward_shaper(forward_curve);
mclib::control::DriveCurve turn_shaper(turn_curve);

// Call this whenever the sticks take the drive. Without it the slew limit
// ramps from whatever the output was the last time the sticks had it.
void resetShapers() {
  forward_shaper.reset();
  turn_shaper.reset();
}

// Arcade with a slow mode. A positive turn is clockwise, here and everywhere
// else in the library.
void arcadeWithSlowMode() {
  const double scale = master.getDigital(DigitalButton::L2) ? 0.4 : 1.0;
  chassis.arcade(
      scale * forward_shaper.apply(master.analog(AnalogAxis::LeftY)),
      scale * turn_shaper.apply(master.analog(AnalogAxis::RightX)));
}

// Tank. shapeDriveInput() is the shaping with no memory, so it has no slew
// limit and needs no reset.
void tankWithoutSlew() {
  chassis.tank(
      mclib::control::shapeDriveInput(master.analog(AnalogAxis::LeftY),
                                      forward_curve),
      mclib::control::shapeDriveInput(master.analog(AnalogAxis::RightY),
                                      forward_curve));
}

// Curvature drive: the turn stick sets how tightly the path bends, not how
// fast the robot spins. With the drive stick centred the robot turns in place
// only while R1 is held, so a bumped turn stick cannot spin a robot that is
// standing still.
void curvatureWithQuickTurn() {
  const mclib::chassis_math::DrivePair sides =
      mclib::chassis_math::curvatureMix(
          forward_shaper.apply(master.analog(AnalogAxis::LeftY)),
          turn_shaper.apply(master.analog(AnalogAxis::RightX)),
          master.getDigital(DigitalButton::R1));
  chassis.tank(sides.left, sides.right);
}

// Arcade in volts. Full forward with full turn asks one side for 24 V.
// scaleToMax() brings the pair back inside 12 V and keeps the ratio between
// the sides, so the robot still follows the arc the sticks asked for.
// chassis.arcade() clips each side on its own instead.
void arcadeInVolts() {
  const double forward = forward_shaper.apply(master.analog(AnalogAxis::LeftY));
  const double turn = turn_shaper.apply(master.analog(AnalogAxis::RightX));
  double left_volts = 12.0 * (forward + turn);
  double right_volts = 12.0 * (forward - turn);
  scaleToMax(left_volts, right_volts, 12.0);
  chassis.tankVoltage(left_volts * 1_V, right_volts * 1_V);
}

// Up and Down change the turn curve while the robot drives. When the driver
// is happy, copy the gain from the controller screen into turn_curve.
void adjustTurnGain() {
  mclib::control::DriveCurveConfig curve = turn_shaper.config();
  if (master.getNewPress(DigitalButton::Up)) {
    curve.gain += 1.0;
  } else if (master.getNewPress(DigitalButton::Down)) {
    curve.gain = std::max(0.0, curve.gain - 1.0);
  } else {
    return;
  }
  turn_shaper.setConfig(curve);
  master.print(1, 0, "turn gain %-5.0f", curve.gain);
}

// ---------------------------------------------------------------------------
// The drive styles. Whichever one is chosen becomes the drive's default
// command: the scheduler runs it whenever no other command has the drive.
// ---------------------------------------------------------------------------

// One of the hand-written functions above as a command that needs the drive,
// built the way the library builds its own drive commands.
std::unique_ptr<Command> makeOwnDriveCommand(
    std::function<void()> drive_one_pass) {
  return std::make_unique<FunctionalCommand>(
      resetShapers,                          // when the command starts
      std::move(drive_one_pass),             // on every scheduler pass
      [](bool) { chassis.tank(0.0, 0.0); },  // when it ends or is interrupted
      [] { return false; },                  // it never finishes by itself
      std::initializer_list<Subsystem*>{&drive});
}

struct DriveStyle {
  const char* name;
  std::function<std::unique_ptr<Command>()> make_command;
};

const std::array<DriveStyle, 8> drive_styles{{
    // Left stick up and down drives, right stick left and right turns.
    {"Arcade",
     [] {
       return drive.makeArcadeDriveCommand(master, forward_curve, turn_curve);
     }},
    // Any two axes work. These two put both on the left stick.
    {"1-stick arcade",
     [] {
       return drive.makeArcadeDriveCommand(
           master, forward_curve, turn_curve, AnalogAxis::LeftY,
           AnalogAxis::LeftX);
     }},
    // Each stick drives its own side. Both sticks share one curve.
    {"Tank", [] { return drive.makeTankDriveCommand(master, forward_curve); }},
    // The turn stick bends the path. With the drive stick centred this one
    // always turns in place.
    {"Curvature",
     [] {
       return drive.makeCurvatureDriveCommand(master, forward_curve,
                                              turn_curve);
     }},
    {"Own: slow mode", [] { return makeOwnDriveCommand(arcadeWithSlowMode); }},
    {"Own: tank", [] { return makeOwnDriveCommand(tankWithoutSlew); }},
    {"Own: quick turn",
     [] { return makeOwnDriveCommand(curvatureWithQuickTurn); }},
    {"Own: volts", [] { return makeOwnDriveCommand(arcadeInVolts); }},
}};

std::size_t drive_style_index = 0;

// The drive owns its default command. Handing it a new one ends the old one
// first, so this is safe while the robot is being driven.
void useDriveStyle(std::size_t index) {
  drive_style_index = index;
  drive.setDefaultCommand(drive_styles[index].make_command());
  master.print(0, 0, "%-15s", drive_styles[index].name);
}

void stepDriveStyle(std::size_t step) {
  useDriveStyle((drive_style_index + step) % drive_styles.size());
}

// ---------------------------------------------------------------------------
// Commands that borrow the drive. Each needs the drive subsystem, so
// scheduling one interrupts the stick command. When it ends, the scheduler
// starts the stick command again.
//
// The scheduler does not own these. Keep them alive for the whole program.
// ---------------------------------------------------------------------------

// Brakes and resists being pushed for as long as it runs.
std::unique_ptr<Command> hold_position = drive.startEnd(
    [] { chassis.stop(BrakeMode::Hold); },
    [] { chassis.setBrakeMode(driver_brake_mode); });

// Turns to heading 0, the way the robot faced when the IMU calibrated, at no
// more than 8 V. Gives up after 1.5 s.
std::unique_ptr<Command> face_heading_zero =
    drive.makeTurnToHeadingCommand(0_deg, 1500_ms, true, 8_V);

std::unique_ptr<Command> back_up =
    drive.makeDriveDistanceCommand(-6_in, 1_s);

// ---------------------------------------------------------------------------
// Without a ChassisController, and without the scheduler. To run the robot
// this way, delete `drive` and the two sections above that use it, and make
// initialize() and opcontrol() call these two functions instead of what they
// do now.
// ---------------------------------------------------------------------------
void initializeWithoutController() {
  imu->reset(true);
  // What building a ChassisController does for you. The motion routines,
  // startOdometry() and free functions such as stopChassis() act on the bound
  // drive, and the motion routines read this tuning. With no drive bound,
  // every motion returns MotionResult::NoDrive.
  mclib::control::bindDrive(&chassis);
  mclib::control::setMotionConfig(tuning);
}

void opcontrolWithoutController() {
  chassis.setBrakeMode(driver_brake_mode);
  resetShapers();
  while (true) {
    adjustTurnGain();
    if (master.getNewRelease(DigitalButton::B)) {
      chassis.setBrakeMode(driver_brake_mode);
      resetShapers();
    }
    if (master.getDigital(DigitalButton::B)) {
      stopChassis(BrakeMode::Hold);
    } else {
      arcadeWithSlowMode();
    }
    pros::delay(10);
  }
}

void initialize() {
  // Calibrate before anything reads the IMU.
  imu->reset(true);

  // main.cpp keeps its drive command in a static and registers it by pointer.
  // Here the drive owns the command, which is what lets a button replace it.
  useDriveStyle(0);
  drive.registerSelf();
}

void disabled() {
  CommandScheduler::disable();
  chassis.stop();
}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
  // Set the brake mode here, not in initialize(). Every autonomous motion
  // that stops leaves the drive on Hold, and a disable leaves it on Brake.
  chassis.setBrakeMode(driver_brake_mode);

  bool goal_was_running = false;
  while (true) {
    if (master.getNewPress(DigitalButton::Right)) {
      stepDriveStyle(1);
    }
    if (master.getNewPress(DigitalButton::Left)) {
      stepDriveStyle(drive_styles.size() - 1);
    }
    adjustTurnGain();

    if (master.getNewPress(DigitalButton::B)) {
      hold_position->schedule();
    }
    if (master.getNewRelease(DigitalButton::B)) {
      hold_position->cancel();
    }
    if (master.getNewPress(DigitalButton::A)) {
      face_heading_zero->schedule();
    }
    if (master.getNewPress(DigitalButton::Y)) {
      back_up->schedule();
    }

    CommandScheduler::run();

    // isActive() is true while the drive runs a turn or a distance goal. Such
    // a goal ends on Hold, so hand the driver's brake mode back.
    const bool goal_is_running = drive.isActive();
    if (goal_was_running && !goal_is_running) {
      chassis.setBrakeMode(driver_brake_mode);
    }
    goal_was_running = goal_is_running;

    pros::delay(10);
  }
}
