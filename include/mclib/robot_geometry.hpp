// mclib
#pragma once

#include "mclib/units/geometry.hpp"

/**
 * @file robot_geometry.hpp
 * @brief YOUR ROBOT'S DRIVETRAIN GEOMETRY. Measure it and edit it.
 *
 * This is the one place mclib learns how big your robot is. Everything that
 * converts encoder degrees into inches goes through the two constants below:
 * `motion.cpp`'s `driveTo()` and `curveCircle()`, the odometry task, and
 * `Chassis` (whose `ChassisDimensions` is `units::DriveGeometry`, the same
 * type as `robot_drive_geometry`).
 *
 * It used to be described three times, in three conventions, with three
 * numbers:
 *
 *     config.cpp   wheel_distance_in = 9.06          a CIRCUMFERENCE, misnamed
 *     config.cpp   vertical_tracker_diameter = 2     a DIAMETER
 *     chassis.hpp  wheel_diameter_in = 2.75          a DIAMETER, a third value
 *
 * 9.06 in of circumference is a 2.8839 in wheel, so the drive path and the
 * `Chassis` path disagreed by 4.87% on distance and 1.1% on turn arc - about
 * half an inch of error per ten inches driven, depending which code path you
 * happened to be on. The four `double` globals that carried those numbers are
 * gone; see the note in `config.cpp`.
 *
 * `units::Wheel`'s two named factories are what stops it happening again.
 * There is no constructor taking a bare number, so you have to say which
 * measurement you have:
 *
 *     Wheel::fromCircumference(9.06 * units::inch)   tape around the tread
 *     Wheel::fromDiameter(2.75 * units::inch)        calipers across the wheel
 *
 * The values here are the ones this robot's autonomous was tuned against:
 * 9.06 in measured around a compressed tread, 11.375 in between the wheel
 * contact patches. **If you copy this template, put your own numbers in.**
 *
 * `gear_ratio` is wheel revolutions per **motor** revolution, because the drive
 * encoders mclib reads are the V5 motors' own: `getLeftRotationDegree()` goes
 * to `left_chassis.getPositionsDeg()`, and `Chassis::leftPositionDeg()` reads
 * the same motors. Direct drive is 1.0; a 36:48 external gearing is 36.0/48.0.
 *
 * If you installed mclib as a PROS template, editing this header is not enough
 * on its own - `motion.cpp` and the odometry task are already compiled into
 * `mclib.a`. Assign to `robot_drive_geometry` in `initialize()` instead; see
 * its doc comment below.
 *
 * This header exists separately from `config.hpp` only because `config.hpp`
 * pulls in the PROS device wrappers, which do not build on the host. Splitting
 * the geometry out lets `tests/geometry_test.cpp` assert against the real
 * values rather than a restated copy of them. `config.hpp` includes this, so
 * there is still one file to edit.
 */

namespace mclib {
namespace config {

namespace detail {

// The numbers, written once. These are `constexpr` so the static_asserts below
// can see them; they are not what the rest of the library reads. Edit here.
inline constexpr units::DriveGeometry declared_drive_geometry{
    units::Wheel::fromCircumference(9.06 * units::inch),
    11.375 * units::inch,
    1.0,
};

inline constexpr units::TrackingWheel declared_vertical_tracking_wheel{
    units::Wheel::fromDiameter(2.0 * units::inch),
    0.0 * units::inch,
    1.0,
};

// Sanity, not tuning. These no longer pin the geometry to one team's numbers -
// you are supposed to edit them - but a zero or negative wheel, track width or
// gear ratio is a typo in every case, and it would otherwise show up as an
// autonomous that never moves or that drives backwards.
static_assert(declared_drive_geometry.wheel.diameter() > 0.0 * units::inch,
              "drive geometry wheel must have a positive diameter");
static_assert(declared_drive_geometry.track_width > 0.0 * units::inch,
              "drive geometry track_width must be positive");
static_assert(declared_drive_geometry.gear_ratio > 0.0,
              "drive geometry gear_ratio must be positive");
static_assert(declared_vertical_tracking_wheel.wheel.diameter() > 0.0 * units::inch,
              "vertical tracking wheel must have a positive diameter");
static_assert(declared_vertical_tracking_wheel.gear_ratio > 0.0,
              "vertical tracking wheel gear_ratio must be positive");

}  // namespace detail

/**
 * @brief The drive base every part of mclib measures with.
 *
 * Defaults to the geometry declared above: 9.06 in of rolling circumference
 * per wheel revolution, 11.375 in between the wheel contact patches, drive
 * encoders geared 1:1 to the wheels.
 *
 * Mutable on purpose. `motion.cpp` and `odometry_task.cpp` ship **precompiled**
 * inside `bin/mclib.a`, so a team that installs mclib as a PROS template and
 * edits this header only changes the translation units they compile
 * themselves - `driveTo()` and the odometry task would keep the numbers the
 * archive was built with, which is exactly the silent scaling error this file
 * exists to prevent. Assigning to it works everywhere:
 *
 * @code
 * void initialize() {
 *   using namespace mclib::units;
 *   mclib::config::robot_drive_geometry = {Wheel::fromDiameter(3.25 * inch),
 *                                          12.5 * inch, 36.0 / 48.0};
 *   mclib::control::startOdometry();
 * }
 * @endcode
 *
 * Set it once, in `initialize()`, before anything moves. `motion.cpp` reads it
 * inside its control loops and `startOdometry()` reads it when it starts, so
 * changing it mid-motion rescales a motion that is already running.
 *
 * The initialiser is a constant expression, so this is constant-initialised
 * before any static constructor runs - no static initialisation order hazard.
 */
inline units::DriveGeometry robot_drive_geometry = detail::declared_drive_geometry;

/// @brief The vertical (forward/back) tracking wheel: a 2 in wheel on the
///        tracking centre. Mutable for the same reason as
///        `robot_drive_geometry`; `startOdometry()` reads it when it starts.
inline units::TrackingWheel vertical_tracking_wheel =
    detail::declared_vertical_tracking_wheel;

}  // namespace config
}  // namespace mclib
