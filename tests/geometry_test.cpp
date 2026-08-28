// mclib
/**
 * @file geometry_test.cpp
 * @brief Host tests for mclib/units/geometry.hpp and the one robot geometry in
 *        mclib/robot_geometry.hpp.
 *
 * Covers the three things that go wrong with wheel geometry: mixing up
 * diameter and circumference, forgetting the /360, and more than one config
 * value describing the same robot.
 *
 * The third one is now structural rather than checked. There is exactly one
 * drive geometry, `mclib::config::robot_drive_geometry`, and every consumer
 * reads it: `motion.cpp`'s `encoderDegreesToInches()` and `halfTrackWidthIn()`,
 * `odometry_task.cpp`'s `odometryConfigFromGlobals()`, and `Chassis`, whose
 * `ChassisDimensions` is an alias for the same `units::DriveGeometry` type.
 * The tests below recompute what each of those consumers computes, from the
 * real constant, and check they all agree.
 */

#include "mclib/control/odometry_task.hpp"
#include "mclib/robot_geometry.hpp"
#include "mclib/units/geometry.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <type_traits>

using mclib::units::DriveGeometry;
using mclib::units::QAngle;
using mclib::units::QLength;
using mclib::units::TrackingWheel;
using mclib::units::Wheel;
using mclib::units::degree;
using mclib::units::inch;
using mclib::units::pi;

namespace {

// The one geometry, straight from the header the user edits. Not a copy of it.
//
// `robot_drive_geometry` is the mutable value every consumer reads;
// `detail::declared_drive_geometry` is the constexpr constant it is
// initialised from, which is what the compile-time assertions can see.
constexpr const DriveGeometry& kDeclared = mclib::config::detail::declared_drive_geometry;
const DriveGeometry& kDrive = mclib::config::robot_drive_geometry;
const TrackingWheel& kTracker = mclib::config::vertical_tracking_wheel;

// --- Type-level guarantees --------------------------------------------------
//
// A bare length cannot become a Wheel: you must say which measurement it is.
static_assert(!std::is_constructible_v<Wheel, QLength>,
              "Wheel must not be constructible from a bare QLength");
static_assert(!std::is_default_constructible_v<Wheel>,
              "Wheel must not be default-constructible");
static_assert(!std::is_constructible_v<Wheel, double>,
              "Wheel must not be constructible from a bare double");

// A diameter cannot be passed where a circumference is meant, and vice versa,
// because neither is a number by the time it reaches DriveGeometry: it is a
// Wheel, and the only way to build one is to name the measurement. These two
// assertions are the proof.
//
// 1. DriveGeometry's first member is a Wheel, so no QLength - diameter or
//    circumference - can be aggregate-initialised into its place.
static_assert(!std::is_constructible_v<DriveGeometry, QLength, QLength, double>,
              "DriveGeometry must not accept a bare length as its wheel");
static_assert(std::is_constructible_v<DriveGeometry, Wheel, QLength, double>,
              "DriveGeometry must accept a Wheel as its wheel");
// 2. And DriveGeometry has no default, so it cannot be built empty and filled
//    in later either. `ChassisDimensions{}` is this same check.
static_assert(!std::is_default_constructible_v<DriveGeometry>,
              "DriveGeometry must not be default-constructible");
static_assert(!std::is_default_constructible_v<TrackingWheel>,
              "TrackingWheel must not be default-constructible");

// The two factories are not interchangeable: the same number gives different
// wheels, so a swap changes the answer by pi instead of passing silently.
static_assert(mclib::units::abs(Wheel::fromDiameter(2.75 * inch).diameter() -
                                Wheel::fromCircumference(2.75 * inch).diameter()) >
                  0.1 * inch,
              "fromDiameter and fromCircumference must not agree");

// The unit system itself blocks the other half of the confusion: an angle is
// not a length, so no encoder reading can be used as a distance by accident.
static_assert(!std::is_assignable_v<QLength&, QAngle>, "QAngle must not assign to QLength");
static_assert(!std::is_assignable_v<QAngle&, QLength>, "QLength must not assign to QAngle");

// One revolution rolls exactly one circumference, checked at compile time.
static_assert(mclib::units::abs(kDeclared.encoderToDistance(360.0 * degree) -
                                kDeclared.wheel.circumference()) < 1e-12 * inch,
              "360 encoder degrees must roll exactly one circumference");

// --- What each consumer computes, restated here ------------------------------

/// @brief `motion.cpp`'s helper: encoder degrees to inches rolled.
double motionEncoderDegreesToInches(double deg) {
  return kDrive.encoderToDistance(deg * degree).in();
}

/// @brief `motion.cpp`'s helper: half the track width, for curveCircle()'s
///        inner and outer arc lengths.
double motionHalfTrackWidthIn() {
  return kDrive.turnRadius().in();
}

/// @brief `odometry_task.cpp`: inches of travel per drive encoder revolution.
///
/// Calls the real thing rather than restating it. It used to restate it as
/// `kDrive.wheel.circumference().in()`, which is the `gear_ratio`-dropping bug
/// `odometryConfigFrom()` was fixed to remove - so this file would have kept
/// passing if the bug came back, and the 36:48 block at the bottom asserted
/// the wrong number outright.
double odometryDriveInchesPerRevolution() {
  return mclib::control::odometryConfigFrom(kDrive, kTracker)
      .drive_inches_per_revolution.in();
}

/// @brief `odometry_task.cpp`: circumference of the vertical tracking wheel.
double odometryVerticalCircumferenceIn() {
  return mclib::control::odometryConfigFrom(kDrive, kTracker)
      .vertical_circumference.in();
}

/// @brief `Chassis::degreesToInches()`, which stays in inches on purpose.
double chassisDegreesToInches(double deg) {
  return (deg / 360.0) * kDrive.wheel.diameter().in() * pi * kDrive.gear_ratio;
}

/// @brief The formula motion.cpp and odometry.cpp open-coded before this
///        change: `deg * wheel_distance_in / 360.0`, where `wheel_distance_in`
///        was the drive wheel circumference in inches.
double legacyDriveInches(double deg) {
  return deg * kDrive.wheel.circumference().in() / 360.0;
}

/// @brief The formula odometry.cpp open-coded for the vertical tracker:
///        `deg * vertical_tracker_diameter * M_PI / 360.0`.
double legacyTrackerInches(double deg) {
  return deg * kTracker.wheel.diameter().in() * pi / 360.0;
}

/// @brief `distance_between_wheels / 2`, the expression curveCircle() used.
double legacyHalfTrackWidthIn() {
  return kDrive.track_width.in() / 2.0;
}

constexpr double kSamples[] = {0.0,    1.0,     -1.0,     37.5,    90.0,
                               360.0,  -720.0,  1234.5,   -845.25, 3600.0,
                               100000.0};

}  // namespace

int main() {
  // --- The live geometry starts as the declared one -------------------------
  CHECK_EQ(kDrive.wheel.diameter().in(), kDeclared.wheel.diameter().in());
  CHECK_EQ(kDrive.track_width.in(), kDeclared.track_width.in());
  CHECK_EQ(kDrive.gear_ratio, kDeclared.gear_ratio);

  // --- The one geometry, as robot_geometry.hpp states it ---------------------
  // 9.06 in of rolling circumference, 11.375 in track width, 1:1.
  CHECK_NEAR(kDrive.wheel.circumference().in(), 9.06, 1e-12);
  CHECK_NEAR(kDrive.wheel.diameter().in(), 9.06 / pi, 1e-12);
  CHECK_NEAR(kDrive.wheel.diameter().in(), 2.8838875688, 1e-9);
  CHECK_NEAR(kDrive.wheel.radius().in(), 1.4419437844, 1e-9);
  CHECK_NEAR(kDrive.track_width.in(), 11.375, 1e-12);
  CHECK_EQ(kDrive.gear_ratio, 1.0);
  CHECK_NEAR(kTracker.wheel.diameter().in(), 2.0, 1e-12);
  CHECK_NEAR(kTracker.offset.in(), 0.0, 1e-12);
  CHECK_EQ(kTracker.gear_ratio, 1.0);

  // --- Wheel: the two constructions describe the same circle -----------------
  const Wheel from_diameter = Wheel::fromDiameter(2.75 * inch);
  CHECK_NEAR(from_diameter.circumference().in(), 2.75 * pi, 1e-12);
  CHECK_NEAR(from_diameter.circumference().in(), 8.639380, 1e-6);
  // Round-trip: circumference back into a wheel gives the same wheel.
  CHECK_NEAR(Wheel::fromCircumference(from_diameter.circumference()).diameter().in(), 2.75,
             1e-12);

  // --- Encoder degrees to inches --------------------------------------------
  CHECK_NEAR(kDrive.encoderToDistance(360.0 * degree).in(), 9.06, 1e-12);
  CHECK_NEAR(kDrive.encoderToDistance(0.0 * degree).in(), 0.0, 1e-12);
  CHECK_NEAR(kDrive.encoderToDistance(180.0 * degree).in(), 4.53, 1e-12);
  CHECK_NEAR(kDrive.encoderToDistance(-720.0 * degree).in(), -18.12, 1e-12);

  // Inverse.
  CHECK_NEAR(kDrive.distanceToEncoder(9.06 * inch).deg(), 360.0, 1e-9);
  CHECK_NEAR(kDrive.distanceToEncoder(kDrive.encoderToDistance(1234.5 * degree)).deg(), 1234.5,
             1e-9);

  // --- The refactor did not move the arithmetic ------------------------------
  //
  // motion.cpp's seven `deg * wheel_distance_in / 360.0` sites now call
  // DriveGeometry::encoderToDistance(). That routes through metres - a
  // multiply by 0.0254 and a divide by it - so it is not bit-identical to the
  // inches-only spelling. The gap is bounded here: 2 ulp, 4.61e-16 relative.
  // Over a 10 in drive that is 5e-15 in.
  for (const double deg : kSamples) {
    const double migrated = motionEncoderDegreesToInches(deg);
    const double legacy = legacyDriveInches(deg);
    // Absolute, for the zero case; relative, for everything else.
    CHECK_NEAR(migrated, legacy, 1e-12);
    if (legacy != 0.0) {
      CHECK(std::abs((migrated - legacy) / legacy) <= 1e-15);
    }
  }

  // The tracking wheel's diameter-form conversion, likewise.
  for (const double deg : kSamples) {
    CHECK_NEAR(kTracker.encoderToDistance(deg * degree).in(), legacyTrackerInches(deg), 1e-12);
  }

  // curveCircle()'s `distance_between_wheels / 2` is exactly turnRadius().
  CHECK_EQ(motionHalfTrackWidthIn(), legacyHalfTrackWidthIn());
  CHECK_EQ(motionHalfTrackWidthIn(), 5.6875);

  // --- Every consumer reads the same geometry --------------------------------
  //
  // This is what replaced the two knownBug() entries. The drive path, the
  // odometry path and the Chassis path used to be 4.87% and 1.1% apart because
  // each carried its own numbers. They now agree because they share one value.

  // One drive wheel revolution is the same distance to all three.
  CHECK_NEAR(odometryDriveInchesPerRevolution(), 9.06, 1e-12);
  CHECK_NEAR(motionEncoderDegreesToInches(360.0), odometryDriveInchesPerRevolution(), 1e-12);
  CHECK_NEAR(chassisDegreesToInches(360.0), odometryDriveInchesPerRevolution(), 1e-12);

  // ...and so is every other encoder reading. Chassis stays in inches for
  // bit-identity with its old loop, so it is compared at 1e-12 too.
  for (const double deg : kSamples) {
    CHECK_NEAR(chassisDegreesToInches(deg), motionEncoderDegreesToInches(deg), 1e-9);
  }
  // Chassis computes `diameter * pi` where motion.cpp's legacy formula used
  // the circumference directly. Same number, different rounding - within
  // 1e-15 relative, which is the point: they are one geometry now, not two.
  for (const double deg : kSamples) {
    const double chassis = chassisDegreesToInches(deg);
    const double legacy = legacyDriveInches(deg);
    CHECK_NEAR(chassis, legacy, 1e-12);
    if (legacy != 0.0) {
      CHECK(std::abs((chassis - legacy) / legacy) <= 1e-15);
    }
  }

  // The old 2.75 in ChassisDimensions default is gone. Had it survived, it
  // would put Chassis 4.87% away from the drive path - half an inch of error
  // per ten inches driven.
  const double stale_chassis_rev = Wheel::fromDiameter(2.75 * inch).circumference().in();
  CHECK_NEAR((odometryDriveInchesPerRevolution() - stale_chassis_rev) / stale_chassis_rev,
             0.0486864, 1e-7);
  // And the geometry Chassis actually uses is not that one.
  CHECK(std::abs(odometryDriveInchesPerRevolution() - stale_chassis_rev) > 0.4);

  // The vertical tracker is a 2 in wheel: 6.2832 in per revolution, and it is
  // a diameter, so it is nothing like the drive wheel's 9.06.
  CHECK_NEAR(odometryVerticalCircumferenceIn(), 2.0 * pi, 1e-12);
  CHECK_NEAR(odometryVerticalCircumferenceIn(), 6.283185307, 1e-9);

  // --- Track width ----------------------------------------------------------
  CHECK_NEAR(kDrive.turnRadius().in(), 5.6875, 1e-12);
  // Spinning 90 deg in place rolls each side pi/2 * 5.6875 in.
  CHECK_NEAR(kDrive.spinArc(90.0 * degree).in(), 5.6875 * pi / 2.0, 1e-12);
  CHECK_NEAR(kDrive.spinArc(90.0 * degree).in(), 8.933904, 1e-6);

  // --- Gear ratio -----------------------------------------------------------
  // Half a wheel turn per encoder turn halves the distance.
  constexpr DriveGeometry geared{Wheel::fromCircumference(9.06 * inch), 11.375 * inch, 0.5};
  CHECK_NEAR(geared.encoderToDistance(360.0 * degree).in(), 4.53, 1e-12);

  // --- The runtime override every consumer has to honour ---------------------
  //
  // motion.cpp and odometry_task.cpp ship precompiled in mclib.a, so a team
  // installing mclib as a PROS template cannot change the geometry by editing
  // the header - they assign to `robot_drive_geometry` in initialize(). The
  // helpers above read the same reference the library does, so this checks
  // that the assignment reaches all of them. Done last, and undone after.
  const DriveGeometry saved = mclib::config::robot_drive_geometry;
  mclib::config::robot_drive_geometry =
      DriveGeometry{Wheel::fromDiameter(3.25 * inch), 12.5 * inch, 36.0 / 48.0};

  // A 3.25 in wheel geared 36:48 rolls 3.25 * pi * 0.75 = 7.6576 in per motor
  // revolution, not 9.06.
  CHECK_NEAR(motionEncoderDegreesToInches(360.0), 3.25 * pi * 0.75, 1e-12);
  CHECK_NEAR(motionEncoderDegreesToInches(360.0), 7.6576321, 1e-7);
  // Odometry has to measure the same 7.6576 in, not the ungeared 10.21 in it
  // used to: that 4/3 disagreement with driveTo() is the bug this line was
  // pinning as expected behaviour.
  CHECK_NEAR(odometryDriveInchesPerRevolution(), 3.25 * pi * 0.75, 1e-12);
  CHECK_NEAR(odometryDriveInchesPerRevolution(), motionEncoderDegreesToInches(360.0),
             1e-12);
  CHECK_NEAR(chassisDegreesToInches(360.0), 3.25 * pi * 0.75, 1e-12);
  CHECK_NEAR(motionHalfTrackWidthIn(), 6.25, 1e-12);

  mclib::config::robot_drive_geometry = saved;
  CHECK_NEAR(motionEncoderDegreesToInches(360.0), 9.06, 1e-12);
  CHECK_NEAR(motionHalfTrackWidthIn(), 5.6875, 1e-12);

  return mclib::test::summary("geometry");
}
