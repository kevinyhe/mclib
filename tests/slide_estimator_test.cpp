// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/slide_estimator.hpp"

#include "mclib/control/odometry.hpp"
#include "mclib/control/odometry_task.hpp"
#include "mclib/device/inertial.hpp"
#include "mclib/units/units.hpp"

#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cmath>

using mclib::control::Odometry;
using mclib::control::OdometryConfig;
using mclib::control::OdometrySample;
using mclib::control::SlideEstimator;
using mclib::control::SlideEstimatorConfig;

namespace {

constexpr double kDt = 0.01;
constexpr double kGravity = mclib::control::kGravityIps2;

/// @brief Feeds the estimator a robot standing still, the accelerometer
///        reading @p zero, for @p seconds. Returns the sideways travel.
double standStill(SlideEstimator& estimator, double zero, double seconds) {
  double travel = 0.0;
  for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
    travel += estimator.update(kDt, 0.0, 0.0, zero);
  }
  return travel;
}

void testStandingStillLearnsTheZero() {
  SlideEstimator estimator;
  const double zero = 0.03 * kGravity;
  const double travel = standStill(estimator, zero, 2.0);
  CHECK_NEAR(estimator.accelZeroIps2(), zero, 0.5);
  CHECK_NEAR(estimator.sidewaysSpeedIps(), 0.0, 0.05);
  CHECK(std::fabs(travel) < 0.05);
}

void testStartingToMoveIsNotTheZero() {
  // The first tick of a spin: wheels and heading haven't moved yet, but an
  // IMU 3 in ahead of the centre already feels the spin speeding up.
  SlideEstimator estimator;
  const double zero = 0.03 * kGravity;
  standStill(estimator, zero, 0.5);
  estimator.update(kDt, 0.0, 0.0, zero + 110.0);
  CHECK_NEAR(estimator.accelZeroIps2(), zero, 0.01);
}

void testTurnWithoutSlideReadsNoSlide() {
  // A 24 in radius turn at 20 in/s, gripping: the accelerometer feels only
  // the pull toward the centre, which the turn explains.
  SlideEstimator estimator;
  const double speed = 20.0, radius = 24.0;
  const double rate = speed / radius;
  double travel = 0.0;
  for (int i = 0; i < 150; ++i) {
    travel += estimator.update(kDt, speed * kDt, rate * kDt, rate * speed);
  }
  CHECK(std::fabs(travel) < 0.01);
}

void testSlideIsAddedUp() {
  // Turning right at 2 rad/s and 34 in/s asks 68 in/s^2 of the tires. They
  // give less, so the robot slides out (left) at up to 15 in/s and back.
  SlideEstimator estimator;
  standStill(estimator, 0.0, 0.5);
  const double speed = 34.0, rate = 2.0;
  const double slide_time = 0.6;
  const double peak = -15.0;
  double true_travel = 0.0, estimated = 0.0;
  double previous_slide = 0.0;
  for (int i = 1; i <= static_cast<int>(slide_time / kDt); ++i) {
    const double t = i * kDt;
    const double slide = peak * std::sin(mclib::kPi * t / slide_time);
    const double slide_rate = (slide - previous_slide) / kDt;
    const double accel_right = slide_rate + rate * speed;
    estimated += estimator.update(kDt, speed * kDt, rate * kDt, accel_right);
    true_travel += 0.5 * (slide + previous_slide) * kDt;
    previous_slide = slide;
  }
  CHECK_NEAR(estimated, true_travel, 0.1 * std::fabs(true_travel));
}

void testImuOffTheCentreFeelsTheSpin() {
  // Spinning in place, an IMU 3 in right of the centre feels the pull
  // toward it, and one 4 in ahead feels the spin speeding up. Neither is
  // a slide.
  SlideEstimatorConfig config;
  config.imu_right_in = 3.0;
  config.imu_forward_in = 4.0;
  SlideEstimator estimator(config);
  double travel = 0.0;
  double rate = 0.0;
  for (int i = 0; i < 200; ++i) {
    const double previous = rate;
    rate = std::fmin(6.0, rate + 0.1);
    const double spin_up = (rate - previous) / kDt;
    const double accel_right = spin_up * config.imu_forward_in - rate * rate * config.imu_right_in;
    travel += estimator.update(kDt, 0.0, rate * kDt, accel_right);
  }
  CHECK(std::fabs(travel) < 0.01);
}

void testNoTimeChangesNothing() {
  SlideEstimator estimator;
  CHECK_EQ(estimator.update(0.0, 1.0, 0.1, 100.0), 0.0);
  CHECK_EQ(estimator.update(-0.01, 1.0, 0.1, 100.0), 0.0);
  CHECK_EQ(estimator.sidewaysSpeedIps(), 0.0);
}

void testConfigValidity() {
  CHECK(mclib::control::slideEstimatorConfigIsValid({}));
  SlideEstimatorConfig bad;
  bad.grip_sigma_ips = 0.0;
  CHECK(!mclib::control::slideEstimatorConfigIsValid(bad));
  bad = {};
  bad.imu_forward_in = NAN;
  CHECK(!mclib::control::slideEstimatorConfigIsValid(bad));
}

OdometryConfig accelerometerConfig() {
  OdometryConfig config;
  config.drive_inches_per_revolution = 9.0 * mclib::units::inch;
  config.use_accelerometer = true;
  return config;
}

void testOdometryRefusesAccelerometerWithHorizontalWheel() {
  OdometryConfig both = accelerometerConfig();
  both.use_horizontal_tracker = true;
  CHECK(!mclib::control::odometryConfigIsValid(both));

  Odometry odometry;
  CHECK(!odometry.hasConfigurationError());
  CHECK(!odometry.setConfig(both));
  CHECK(!odometry.getConfig().use_accelerometer);

  Odometry misconfigured(both);
  CHECK(misconfigured.hasConfigurationError());
  OdometrySample sample;
  sample.time_ms = 0.0;
  sample.accel_right_ips2 = 0.0;
  misconfigured.update(sample);
  sample.time_ms = 10.0;
  sample.left_deg = sample.right_deg = 400.0;
  misconfigured.update(sample);
  CHECK_EQ(misconfigured.getPose().y, 0.0);
  CHECK_EQ(static_cast<double>(misconfigured.faultCount()), 2.0);
  CHECK(misconfigured.setConfig(accelerometerConfig()));
  CHECK(!misconfigured.hasConfigurationError());
}

void testOdometryNeedsTimeAndAccel() {
  Odometry odometry(accelerometerConfig());
  OdometrySample sample;
  sample.time_ms = 0.0;
  odometry.update(sample);
  CHECK_EQ(static_cast<double>(odometry.faultCount()), 1.0);
  sample.accel_right_ips2 = 0.0;
  sample.time_ms = NAN;
  odometry.update(sample);
  CHECK_EQ(static_cast<double>(odometry.faultCount()), 2.0);
}

void testOdometryMovesWithAShove() {
  // Standing still, facing +Y, something shoves the robot to its right: the
  // accelerometer reads 100 in/s^2 for 0.2 s and -100 for 0.2 s as the
  // tires stop it. The wheels never turn. That is 4 in to the right.
  Odometry odometry(accelerometerConfig());
  OdometrySample sample;
  sample.accel_right_ips2 = 0.0;
  for (int i = 0; i <= 100; ++i) {
    sample.time_ms = i * 10.0;
    odometry.update(sample);
  }
  for (int i = 1; i <= 40; ++i) {
    sample.time_ms = 1000.0 + i * 10.0;
    sample.accel_right_ips2 = i <= 20 ? 100.0 : -100.0;
    odometry.update(sample);
  }
  CHECK_NEAR(odometry.getPose().x, 4.0, 0.4);
  CHECK_NEAR(odometry.getPose().y, 0.0, 1e-9);
}

void testStraightDriveIsUnchanged() {
  Odometry plain([] {
    OdometryConfig config = accelerometerConfig();
    config.use_accelerometer = false;
    return config;
  }());
  Odometry with_accelerometer(accelerometerConfig());
  OdometrySample sample;
  sample.accel_right_ips2 = 0.0;
  for (int i = 0; i <= 100; ++i) {
    sample.time_ms = i * 10.0;
    sample.left_deg = sample.right_deg = i * 8.0;
    plain.update(sample);
    with_accelerometer.update(sample);
  }
  CHECK_NEAR(with_accelerometer.getPose().x, plain.getPose().x, 1e-12);
  CHECK_NEAR(with_accelerometer.getPose().y, plain.getPose().y, 1e-12);
}

void testSetupTurnsTheAccelerometerOn() {
  mclib::control::OdometrySetup setup;
  CHECK(!mclib::control::odometryConfigFrom(setup).use_accelerometer);
  mclib::control::AccelerometerSensor accelerometer;
  accelerometer.right_ips2 = [] { return 0.0; };
  accelerometer.slide.imu_right_in = -2.5;
  setup.accelerometer = accelerometer;
  const OdometryConfig config = mclib::control::odometryConfigFrom(setup);
  CHECK(config.use_accelerometer);
  CHECK_EQ(config.slide.imu_right_in, -2.5);
}

void testReaderPicksTheAxis() {
  namespace hd = mclib::test::host_devices;
  hd::reset();
  mclib::device::Inertial imu(4);
  hd::imu_accel_g[4] = {0.1, -0.2, 1.0};
  const auto right = mclib::control::sidewaysAccelReader(imu, mclib::device::ImuAxis::MinusY);
  CHECK_NEAR(right(), 0.2 * kGravity, 1e-9);
  hd::imu_accel_g[4] = {INFINITY, INFINITY, INFINITY};
  CHECK(std::isnan(right()));
}

}  // namespace

int main() {
  testStandingStillLearnsTheZero();
  testStartingToMoveIsNotTheZero();
  testTurnWithoutSlideReadsNoSlide();
  testSlideIsAddedUp();
  testImuOffTheCentreFeelsTheSpin();
  testNoTimeChangesNothing();
  testConfigValidity();
  testOdometryRefusesAccelerometerWithHorizontalWheel();
  testOdometryNeedsTimeAndAccel();
  testOdometryMovesWithAShove();
  testStraightDriveIsUnchanged();
  testSetupTurnsTheAccelerometerOn();
  testReaderPicksTheAxis();
  return mclib::test::summary("slide_estimator");
}
