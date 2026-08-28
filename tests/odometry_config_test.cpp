// mclib
/**
 * @file odometry_config_test.cpp
 * @brief Odometry and motion have to measure the same drive base.
 *
 * `odometryConfigFrom()` builds the `OdometryConfig` that `startOdometry()`
 * runs on. It used to assign `wheel.circumference()` straight into
 * `drive_inches_per_revolution`, which drops `gear_ratio`. `driveTo()` goes
 * through `DriveGeometry::encoderToDistance()`, which does not. With the
 * 36:48 gearing from `robot_geometry.hpp`'s own worked example the two
 * disagreed by 4/3 - odometry integrated 33% long on every straight - and
 * nothing said which one was right. That is latent on the shipped default
 * because its `gear_ratio` is 1.0, so it is checked here on a geared robot.
 *
 * `odometry_task.cpp` itself includes PROS and cannot be built on a host, so
 * the arithmetic lives in `odometry_task.hpp` and this file links that.
 */

#include "mclib/control/odometry_task.hpp"

#include "mclib/robot_geometry.hpp"
#include "mclib/units/geometry.hpp"
#include "test_assert.hpp"

using mclib::control::odometryConfigFrom;
using mclib::control::OdometryConfig;
using mclib::units::degree;
using mclib::units::DriveGeometry;
using mclib::units::inch;
using mclib::units::TrackingWheel;
using mclib::units::Wheel;

namespace {

/// @brief The worked example in robot_geometry.hpp: 3.25 in wheels, 12.5 in
///        track, drive encoders geared 36:48.
constexpr DriveGeometry kGeared{Wheel::fromDiameter(3.25 * inch),
                                12.5 * inch,
                                36.0 / 48.0};

constexpr TrackingWheel kGearedTracker{Wheel::fromDiameter(2.0 * inch),
                                       1.5 * inch,
                                       36.0 / 48.0};

/// @brief `motion.cpp`'s `encoderDegreesToInches()`, written out. This is the
///        number `driveTo()` measures with.
double motionInchesPerDegree(const DriveGeometry& drive) {
  return drive.encoderToDistance(1.0 * degree).in();
}

/// @brief What the odometry integrates per encoder degree: `odometry.cpp`
///        divides `drive_inches_per_revolution` by 360.
double odometryInchesPerDegree(const OdometryConfig& config) {
  return config.drive_inches_per_revolution.in() / 360.0;
}

void testDriveMatchesMotion() {
  const OdometryConfig config = odometryConfigFrom(kGeared, kGearedTracker);

  const double odom = odometryInchesPerDegree(config);
  const double motion = motionInchesPerDegree(kGeared);
  CHECK_NEAR(odom, motion, 1e-15);

  // The absolute value, so a future refactor cannot move both sides together.
  // 3.25 in diameter * pi * (36/48) / 360 = 0.021270...
  const double expected = 3.25 * mclib::units::pi * (36.0 / 48.0) / 360.0;
  CHECK_NEAR(odom, expected, 1e-15);

  // What the bug cost. The old code assigned the bare circumference, so it
  // integrated 1 / (36/48) = 4/3 of the true distance: 0.028362 in per degree
  // against a true 0.021270. Over a 24 in drive the pose ran 8 in long.
  const double buggy = kGeared.wheel.circumference().in() / 360.0;
  CHECK_NEAR(buggy / odom, 48.0 / 36.0, 1e-15);
  CHECK_NEAR(buggy, 0.0283616003, 1e-9);
  CHECK_NEAR(odom, 0.0212712003, 1e-9);
  CHECK_NEAR(24.0 * (buggy / odom) - 24.0, 8.0, 1e-12);
}

void testTrackerMatchesGeometry() {
  const OdometryConfig config = odometryConfigFrom(kGeared, kGearedTracker);

  // Same story for the vertical tracking wheel, which has its own gear_ratio.
  CHECK_NEAR(config.vertical_circumference.in() / 360.0,
             kGearedTracker.encoderToDistance(1.0 * degree).in(), 1e-15);
  CHECK_NEAR(config.vertical_circumference.in(),
             2.0 * mclib::units::pi * (36.0 / 48.0), 1e-14);

  // The offset comes straight across, sign and all.
  CHECK_NEAR(config.vertical_offset_right.in(), 1.5, 1e-15);
}

void testDirectDriveUnchanged() {
  // With gear_ratio 1.0 the answer is the plain circumference, which is what
  // the shipped default has always produced. The fix is not a behaviour change
  // for a direct-drive robot.
  const OdometryConfig config =
      odometryConfigFrom(mclib::config::detail::declared_drive_geometry,
                         mclib::config::detail::declared_vertical_tracking_wheel);
  CHECK_NEAR(config.drive_inches_per_revolution.in(), 9.06, 1e-13);
  CHECK_NEAR(config.vertical_circumference.in(), 2.0 * mclib::units::pi, 1e-13);
}

void testFlagsAreOff() {
  const OdometryConfig config = odometryConfigFrom(kGeared, kGearedTracker);
  CHECK(!config.use_vertical_tracker);
  CHECK(!config.use_horizontal_tracker);
}

}  // namespace

int main() {
  testDriveMatchesMotion();
  testTrackerMatchesGeometry();
  testDirectDriveUnchanged();
  testFlagsAreOff();
  return mclib::test::summary("odometry_config");
}
