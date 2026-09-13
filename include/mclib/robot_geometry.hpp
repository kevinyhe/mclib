// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/geometry.hpp"

/**
 * @file robot_geometry.hpp
 * @brief A place to write down your drivetrain geometry once.
 *
 * Nothing in the library reads these constants. They exist so a robot program
 * has one named value to hand to `mclib::Chassis` and to the odometry setup,
 * instead of repeating a wheel size in two places that can drift apart:
 *
 * @code
 * mclib::Chassis chassis({-11, 13, 14}, {-16, 17, -18},
 *                        mclib::device::Gearset::Blue,
 *                        mclib::config::robot_drive_geometry, imu);
 * @endcode
 *
 * `units::Wheel` has two named factories and no constructor taking a bare
 * number, so you have to say which measurement you took:
 *
 *     Wheel::fromCircumference(9.06 * units::inch)   tape around the tread
 *     Wheel::fromDiameter(2.75 * units::inch)        calipers across the wheel
 *
 * A wrong wheel size is a silent 5% scaling error on every autonomous. The
 * values here are one competition robot's: 9.06 in measured around a
 * compressed tread, 11.375 in between the wheel contact patches. **Put your
 * own numbers in**, or ignore this file and state the geometry inline where
 * you build the Chassis.
 *
 * `gear_ratio` is wheel revolutions per **motor** revolution, because the drive
 * encoders mclib reads are the V5 motors' own. Direct drive is 1.0; a 36:48
 * external gearing is 36.0/48.0.
 *
 * This header has no PROS dependency, so `tests/geometry_test.cpp` can assert
 * against the real values.
 */

namespace mclib {
namespace config {

namespace detail {

// The numbers, written once, `constexpr` so the static_asserts below can see
// them.
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

// Sanity, not tuning. A zero or negative wheel, track width or gear ratio is
// a typo in every case, and it would otherwise show up as an autonomous that
// never moves or that drives backwards.
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
 * @brief An example drive base: 9.06 in of rolling circumference per wheel
 *        revolution, 11.375 in between the wheel contact patches, drive
 *        encoders geared 1:1 to the wheels.
 *
 * Mutable so a program can assign its own numbers in `initialize()` before
 * building the Chassis, if it would rather not edit a library header.
 */
inline units::DriveGeometry robot_drive_geometry = detail::declared_drive_geometry;

/// @brief An example vertical (forward/back) tracking wheel: a 2 in wheel on
///        the tracking centre.
inline units::TrackingWheel vertical_tracking_wheel =
    detail::declared_vertical_tracking_wheel;

}  // namespace config
}  // namespace mclib
