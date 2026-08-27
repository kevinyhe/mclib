// mclib
#pragma once

#include "mclib/device/controller.hpp"
#include "mclib/device/distance.hpp"
#include "mclib/device/inertial.hpp"
#include "mclib/device/motor.hpp"
#include "mclib/device/motor_group.hpp"
#include "mclib/device/pneumatic.hpp"
#include "mclib/device/rotation.hpp"
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
// Drivetrain geometry, typed.
//
// The four `double` globals further down describe this robot's geometry in
// three different conventions, and one of them is named for a convention it
// does not use. The two constants here state it once, in units, with diameter
// and circumference distinguished by type - and config.cpp now *derives* the
// globals from them, so there is one source of truth and nothing to keep in
// sync:
//
//   distance_between_wheels           = robot_drive_geometry.track_width.in()
//   wheel_distance_in                 = robot_drive_geometry.wheel.circumference().in()
//   vertical_tracker_diameter         = vertical_tracking_wheel.wheel.diameter().in()
//   vertical_tracker_dist_from_center = vertical_tracking_wheel.offset.in()
//
// and the two open-coded conversions in motion.cpp / odometry.cpp are:
//
//   deg * wheel_distance_in / 360.0
//       == robot_drive_geometry.encoderToDistance(deg * degree).in()
//   deg * vertical_tracker_diameter * M_PI / 360.0
//       == vertical_tracking_wheel.encoderToDistance(deg * degree).in()
//
// The globals are still what motion.cpp and odometry.cpp read; moving those
// call sites onto these constants is Phase 3, not this change.
//
// Note the third, conflicting description of the same robot:
// `mclib::ChassisDimensions` defaults to wheel_diameter_in = 2.75 and
// track_width_in = 11.5, while `wheel_distance_in = 9.06` implies a 2.8839 in
// diameter and the track width here is 11.375. Chassis and motion.cpp will
// disagree by 4.87% on distance and 1.1% on turn arc until one wins. Picking
// the winner needs a tape measure, so it is left as-is and flagged.
// ---------------------------------------------------------------------------

namespace mclib {
namespace config {

/// @brief The drive base: 9.06 in of rolling circumference per wheel
///        revolution, 11.375 in between the wheels, sensor on the wheel shaft.
inline constexpr units::DriveGeometry robot_drive_geometry{
    units::Wheel::fromCircumference(9.06 * units::inch),
    11.375 * units::inch,
    1.0,
};

/// @brief The vertical (forward/back) tracking wheel: a 2 in wheel on the
///        tracking centre.
inline constexpr units::TrackingWheel vertical_tracking_wheel{
    units::Wheel::fromDiameter(2.0 * units::inch),
    0.0 * units::inch,
    1.0,
};

// These pin the constants to the numbers this robot was tuned with, so a typo
// while editing them - a diameter typed into fromCircumference, a decimal
// point slipped - stops the build instead of quietly re-scaling every
// autonomous. They cannot catch a drift between these constants and the
// globals in config.cpp, because those are mutable `extern double`s that no
// static_assert can see; that drift is prevented instead by deriving them,
// which config.cpp does.
static_assert(units::abs(robot_drive_geometry.wheel.circumference() -
                         9.06 * units::inch) < 1e-9 * units::inch,
              "robot_drive_geometry no longer matches wheel_distance_in = 9.06");
static_assert(units::abs(robot_drive_geometry.track_width - 11.375 * units::inch) <
                  1e-9 * units::inch,
              "robot_drive_geometry no longer matches distance_between_wheels = 11.375");
static_assert(units::abs(vertical_tracking_wheel.wheel.diameter() - 2.0 * units::inch) <
                  1e-9 * units::inch,
              "vertical_tracking_wheel no longer matches vertical_tracker_diameter = 2");
static_assert(units::abs(vertical_tracking_wheel.offset) < 1e-9 * units::inch,
              "vertical_tracking_wheel no longer matches "
              "vertical_tracker_dist_from_center = 0");

}  // namespace config
}  // namespace mclib

extern double distance_between_wheels;
extern double wheel_distance_in;
extern double vertical_tracker_diameter;
extern double vertical_tracker_dist_from_center;

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
 * Minimum drive output, in VOLTS - audited, deliberately unchanged.
 *
 * Every motion function takes `min_speed` defaulting to -1.0, and every one of
 * them does `min_speed < 0 ? min_output : min_speed`. The result is a floor on
 * the PID output, which goes to driveChassis() -> MotorGroup::setVoltage().
 * So the unit is volts, and 10 is 83% of the 12 V rail as a *minimum* speed.
 *
 * That is only survivable because most call sites gate it off. The floor is
 * applied unconditionally in turnToAngle()'s two chained branches, in swing(),
 * and in turnToPoint(); in driveTo(), curveCircle(), moveToPoint() and
 * boomerang() it is gated behind `apply_min_speed_floor`, which starts as
 * `min_speed >= 0` - false for the default - and is only turned on by the
 * chaining branches. So a plain `turnToAngle(90, 1000, false)` really does
 * turn at no less than 10 V.
 *
 * 10 reads like a leftover from a 0..127 or 0..100 scale, where it would have
 * been 8% or 10% of full power - a plausible stiction floor. As volts it is
 * not a stiction floor, it is nearly full power. Changing it changes tuned
 * autonomous behaviour, so it is left alone and flagged here.
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
