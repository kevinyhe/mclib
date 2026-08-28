// mclib
#pragma once

#include "mclib/device/controller.hpp"
#include "mclib/device/distance.hpp"
#include "mclib/device/inertial.hpp"
#include "mclib/device/motor.hpp"
#include "mclib/device/motor_group.hpp"
#include "mclib/device/pneumatic.hpp"
#include "mclib/device/rotation.hpp"
#include "mclib/robot_geometry.hpp"
#include "mclib/units/geometry.hpp"

extern mclib::device::Controller master;

extern mclib::device::MotorGroup left_chassis;
extern mclib::device::MotorGroup right_chassis;

extern mclib::device::Inertial inertial_sensor;
extern mclib::device::Motor intake_top_motor;
extern mclib::device::Motor intake_bottom_motor;
extern mclib::device::MotorGroup intake;

extern mclib::device::Distance left_reset;
extern mclib::device::Distance right_reset;

extern mclib::device::Rotation vertical_tracker;

extern mclib::device::Pneumatic flap;
extern mclib::device::Pneumatic middle_descore;
extern mclib::device::Pneumatic scraper;
extern mclib::device::Pneumatic hood;
extern mclib::device::Pneumatic wing;
extern mclib::device::Pneumatic tilter;

// ---------------------------------------------------------------------------
// Drivetrain geometry.
//
// `mclib::config::robot_drive_geometry` and
// `mclib::config::vertical_tracking_wheel` live in mclib/robot_geometry.hpp,
// included above. That is the one place this robot's drive base is described,
// and every consumer - motion.cpp, the odometry task, Chassis - reads it at
// call time. Edit it for your robot, or, if you installed mclib as a PROS
// template, assign to `robot_drive_geometry` in initialize(): motion.cpp and
// the odometry task ship precompiled, so editing the header alone would not
// reach them.
//
// The four `double` geometry globals that used to be declared here
// (`distance_between_wheels`, `wheel_distance_in`, `vertical_tracker_diameter`,
// `vertical_tracker_dist_from_center`) are gone. See config.cpp.
// ---------------------------------------------------------------------------

extern double distance_kp;
extern double distance_ki;
extern double distance_kd;
extern double turn_kp;
extern double turn_ki;
extern double turn_kd;
extern double heading_correction_kp;
extern double heading_correction_ki;
extern double heading_correction_kd;

extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
/**
 * Stiction floor for drive output, in VOLTS. Default 1.5 V.
 *
 * Every motion function takes `min_speed` defaulting to -1.0, and every one of
 * them does `min_speed < 0 ? min_output : min_speed`. The result is a floor on
 * the PID output, which goes to driveChassis() -> MotorGroup::setVoltage().
 * So the unit is volts.
 *
 * What it is for: a drivetrain does not move at all below some voltage. As the
 * PID converges, its output shrinks toward zero and at some point stops being
 * enough to break friction, so the robot stalls an inch short of target and
 * the loop times out there. The floor keeps the last bit of travel moving.
 * 1.5 V of a 12 V rail is about 12% - enough to creep a geared V5 drive,
 * little enough that arriving at the target is gentle.
 *
 * The floor is applied unconditionally in turnToAngle()'s two chained
 * branches, in swing(), and in turnToPoint(); in driveTo(), curveCircle(),
 * moveToPoint() and boomerang() it is gated behind `apply_min_speed_floor`,
 * which starts as `min_speed >= 0` - false for the default - and is only
 * turned on by the chaining branches.
 *
 * This was 10 for a long time, which as volts is 83% of the rail as a
 * *minimum*: `turnToAngle(90, 1000, false)` really did turn at no less than
 * 10 V, and slammed into its target. 10 is almost certainly a leftover from a
 * 0..127 or 0..100 output scale, where it would have been an 8-10% stiction
 * floor - the same intent, the right number for a different scale. Lowering
 * it to 1.5 V restores that intent; turn PID gains tuned against the old
 * behaviour may want revisiting.
 */
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
/**
 * Boomerang slip-speed coefficient - also unit-inconsistent, also unchanged.
 *
 * Used once, as `sqrt(chase_power * getRadius(...) * 9.8)` in boomerang(). The
 * 9.8 is gravity in m/s^2 while getRadius() returns inches, and the result is
 * compared against outputs that are volts. Three unit systems in one
 * expression; the value is an empirical fudge factor, not a physical quantity.
 */
extern double chase_power;

// ---------------------------------------------------------------------------
// Typed views of the tuning globals.
//
// The globals above stay `double` and stay mutable: they are what a driver
// edits between matches, and `motion.cpp` reads them inside a 10 ms loop whose
// numbers this robot was tuned on. This gives the four slew rates one place
// that says what they are, which `motion.cpp` now goes through.
//
// The other tuning globals are not mirrored here. A typed copy of a constant
// that already has a spelling elsewhere is a second thing to keep in sync, and
// this file has been bitten by that before.
// ---------------------------------------------------------------------------

namespace mclib {
namespace config {

/**
 * @brief The four `max_slew_*` globals, gathered.
 *
 * Each is **volts per nominal 10 ms tick**. That is a rate with no name in the
 * unit system - the tick is the loop period, not a measured time - so it stays
 * a double, and the doc comment carries the unit instead.
 */
struct SlewRates {
  double accel_fwd;
  double decel_fwd;
  double accel_rev;
  double decel_rev;
};

/// @brief The four `max_slew_*` globals as they stand right now.
inline SlewRates slewRates() {
  return SlewRates{max_slew_accel_fwd,
                   max_slew_decel_fwd,
                   max_slew_accel_rev,
                   max_slew_decel_rev};
}

}  // namespace config
}  // namespace mclib
