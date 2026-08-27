// mclib
/**
 * @file geometry_test.cpp
 * @brief Host tests for mclib/units/geometry.hpp.
 *
 * Covers the three things that go wrong with wheel geometry: mixing up
 * diameter and circumference, forgetting the /360, and two config values for
 * one robot silently disagreeing.
 */

#include "mclib/units/geometry.hpp"
#include "test_assert.hpp"

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

// The drivetrain as config.cpp describes it today: wheel_distance_in = 9.06 is
// a circumference, distance_between_wheels = 11.375 is the track width.
constexpr DriveGeometry kDrive{Wheel::fromCircumference(9.06 * inch), 11.375 * inch, 1.0};

// vertical_tracker_diameter = 2 really is a diameter.
constexpr TrackingWheel kTracker{Wheel::fromDiameter(2.0 * inch), 0.0 * inch, 1.0};

// --- Type-level guarantees --------------------------------------------------
//
// A bare length cannot become a Wheel: you must say which measurement it is.
static_assert(!std::is_constructible_v<Wheel, QLength>,
              "Wheel must not be constructible from a bare QLength");
static_assert(!std::is_default_constructible_v<Wheel>,
              "Wheel must not be default-constructible");
static_assert(!std::is_constructible_v<Wheel, double>,
              "Wheel must not be constructible from a bare double");

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
static_assert(mclib::units::abs(kDrive.encoderToDistance(360.0 * degree) -
                                kDrive.wheel.circumference()) < 1e-12 * inch,
              "360 encoder degrees must roll exactly one circumference");

/// @brief The formula motion.cpp and odometry.cpp open-code today.
double legacyDriveInches(double deg) { return deg * 9.06 / 360.0; }

/// @brief The formula odometry.cpp open-codes for the vertical tracker.
double legacyTrackerInches(double deg) { return deg * 2.0 * pi / 360.0; }

}  // namespace

int main() {
  // --- Wheel: the two constructions describe the same circle -----------------
  CHECK_NEAR(kDrive.wheel.circumference().in(), 9.06, 1e-12);
  CHECK_NEAR(kDrive.wheel.diameter().in(), 9.06 / pi, 1e-12);
  CHECK_NEAR(kDrive.wheel.radius().in(), 9.06 / (2.0 * pi), 1e-12);
  // 9.06 in of circumference is a 2.8839 in wheel, not a 2.75 in one.
  CHECK_NEAR(kDrive.wheel.diameter().in(), 2.883888, 1e-6);

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

  // Matches the legacy `deg * wheel_distance_in / 360.0` at every sample.
  for (const double deg : {0.0, 1.0, 37.5, 360.0, 1234.5, -845.25, 100000.0}) {
    CHECK_NEAR(kDrive.encoderToDistance(deg * degree).in(), legacyDriveInches(deg), 1e-9);
  }

  // Inverse.
  CHECK_NEAR(kDrive.distanceToEncoder(9.06 * inch).deg(), 360.0, 1e-9);
  CHECK_NEAR(kDrive.distanceToEncoder(kDrive.encoderToDistance(1234.5 * degree)).deg(), 1234.5,
             1e-9);

  // --- Tracking wheel: the diameter-form formula ----------------------------
  CHECK_NEAR(kTracker.encoderToDistance(360.0 * degree).in(), 2.0 * pi, 1e-12);
  CHECK_NEAR(kTracker.encoderToDistance(360.0 * degree).in(), 6.283185, 1e-6);
  for (const double deg : {0.0, 90.0, 360.0, 1500.0, -270.0}) {
    CHECK_NEAR(kTracker.encoderToDistance(deg * degree).in(), legacyTrackerInches(deg), 1e-9);
  }

  // --- Track width ----------------------------------------------------------
  CHECK_NEAR(kDrive.track_width.in(), 11.375, 1e-12);
  CHECK_NEAR(kDrive.turnRadius().in(), 5.6875, 1e-12);
  // Spinning 90 deg in place rolls each side pi/2 * 5.6875 in.
  CHECK_NEAR(kDrive.spinArc(90.0 * degree).in(), 5.6875 * pi / 2.0, 1e-12);
  CHECK_NEAR(kDrive.spinArc(90.0 * degree).in(), 8.933904, 1e-6);

  // --- Gear ratio -----------------------------------------------------------
  // Half a wheel turn per encoder turn halves the distance.
  constexpr DriveGeometry geared{Wheel::fromCircumference(9.06 * inch), 11.375 * inch, 0.5};
  CHECK_NEAR(geared.encoderToDistance(360.0 * degree).in(), 4.53, 1e-12);

  // --- The disagreement this type exists to expose ---------------------------
  // config.cpp says 9.06 in per revolution. ChassisDimensions says a 2.75 in
  // wheel, i.e. 8.6394 in per revolution. Same robot, 4.87% apart. Documented,
  // not asserted away: choosing the right one needs a tape measure.
  const double drive_rev = kDrive.wheel.circumference().in();
  const double chassis_rev = Wheel::fromDiameter(2.75 * inch).circumference().in();
  const double disagreement = (drive_rev - chassis_rev) / chassis_rev;
  CHECK_NEAR(disagreement, 0.0486864, 1e-7);
  mclib::test::knownBug(disagreement > 0.001,
                        "config.cpp wheel_distance_in = 9.06 (2.8839 in wheel) and "
                        "ChassisDimensions::wheel_diameter_in = 2.75 describe the same "
                        "robot 4.87% apart");
  mclib::test::knownBug(11.375 != 11.5,
                        "config.cpp distance_between_wheels = 11.375 and "
                        "ChassisDimensions::track_width_in = 11.5 disagree by 1.1%");

  return mclib::test::summary("geometry");
}
