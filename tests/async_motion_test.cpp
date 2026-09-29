// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file async_motion_test.cpp
 * @brief AsyncMotion on the host: real threads, a real clock, and a drive
 *        whose encoders move with the voltage it is given.
 *
 * The motion runs on its own std::thread here, as it would on its own
 * pros::Task on the brain. pros::millis() and pros::delay() are the wall
 * clock, so this test takes about two seconds.
 */

#include "mclib/control/async_motion.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/sync.hpp"
#include "mclib/time.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
const Clock::time_point kEpoch = Clock::now();

std::uint32_t wallMs() {
  return static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - kEpoch).count());
}

/**
 * Both sides roll at 10 in/s per volt. Every encoder read integrates the
 * voltage since the last read and publishes the distance as the odometry
 * pose, straight along +Y. All of it runs on the motion's thread; the test
 * thread only reads the pose, which RobotState locks.
 */
struct RollingDrive : mclib::control::DriveHardware {
  mclib::sync::Mutex mutex;
  double volts = 0.0;
  double inches = 0.0;
  std::uint32_t last_ms = 0;
  mclib::units::DriveGeometry geometry{
      mclib::units::Wheel::fromDiameter(4 * mclib::units::inch),
      12 * mclib::units::inch, 1};

  void advance() {
    const std::uint32_t now = wallMs();
    inches += volts * 10.0 * (now - last_ms) / 1000.0;
    last_ms = now;
    mclib::control::resetOdometry({0.0, inches, 0.0});
  }
  double degrees() {
    // encoderToDistance: one wheel turn of a 4 in wheel is 4 * pi inches.
    return inches / (4.0 * M_PI) * 360.0;
  }
  void setDriveVoltage(double l, double r) override {
    mclib::sync::LockGuard lock(mutex);
    advance();
    volts = (l + r) / 2.0;
  }
  void setSideVoltage(bool, double v) override {
    mclib::sync::LockGuard lock(mutex);
    advance();
    volts = v / 2.0;
  }
  void brakeDrive(mclib::device::BrakeMode) override {
    mclib::sync::LockGuard lock(mutex);
    advance();
    volts = 0.0;
  }
  void brakeSide(bool, mclib::device::BrakeMode) override { brakeDrive({}); }
  void tareDrive() override {
    mclib::sync::LockGuard lock(mutex);
    inches = 0.0;
  }
  double leftPositionDeg() override {
    mclib::sync::LockGuard lock(mutex);
    advance();
    return degrees();
  }
  double rightPositionDeg() override { return leftPositionDeg(); }
  double headingDeg() override { return 0.0; }
  void setHeadingDeg(double) override {}
  std::vector<double> driveCurrentsMa() override { return {0.0}; }
  std::vector<double> driveVelocitiesRpm() override { return {0.0}; }
  const mclib::units::DriveGeometry& driveGeometry() const override { return geometry; }

  double currentVolts() {
    mclib::sync::LockGuard lock(mutex);
    return volts;
  }
  void reset() {
    mclib::sync::LockGuard lock(mutex);
    volts = 0.0;
    inches = 0.0;
    last_ms = wallMs();
    mclib::control::resetOdometry({0.0, 0.0, 0.0});
  }
};

}  // namespace

namespace pros {
extern "C" std::uint32_t millis() { return wallMs(); }
extern "C" void delay(std::uint32_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
}  // namespace pros

int main() {
  using namespace mclib::units::literals;
  using namespace mclib::control;
  // PID settle timers read mclib::time, the motion loops read pros::millis().
  mclib::time::ScopedClock clock(wallMs);
  RollingDrive hw;
  bindDrive(&hw);
  mclib::test::setCompetitionStatus(0);

  std::printf("-- an empty handle\n");
  {
    AsyncMotion none;
    CHECK(!none.valid());
    CHECK(!none.isRunning());
    CHECK(none.wait() == MotionResult::NoDrive);
    CHECK(none.cancel() == MotionResult::NoDrive);
    CHECK(!none.waitUntilElapsed(1_s));
  }

  std::printf("-- run, act partway, then wait for the result\n");
  {
    hw.reset();
    const std::uint32_t start = wallMs();
    AsyncMotion drive = AsyncMotion::start([] { return driveTo(24_in, 3_s); });
    CHECK(drive.valid());
    CHECK(drive.isRunning());
    CHECK(!drive.result().has_value());
    CHECK(wallMs() - start < 50);  // start() did not block

    CHECK(drive.waitUntilTravelled(12_in));
    const double at = robotState().pose().y;
    std::printf("   waitUntilTravelled(12 in) returned at %.2f in, %u ms\n", at,
                static_cast<unsigned>(wallMs() - start));
    CHECK(at >= 12.0);
    CHECK(at < 24.0);
    CHECK(drive.isRunning());

    const MotionResult result = drive.wait();
    std::printf("   finished: %s at %.2f in after %u ms\n", toString(result),
                robotState().pose().y, static_cast<unsigned>(wallMs() - start));
    CHECK(result == MotionResult::Reached);
    CHECK(!drive.isRunning());
    CHECK(drive.result() == MotionResult::Reached);
    CHECK(std::fabs(robotState().pose().y - 24.0) < 1.5);
    CHECK_EQ(hw.currentVolts(), 0.0);
  }

  std::printf("-- cancel stops within a tick or two\n");
  {
    hw.reset();
    AsyncMotion drive = AsyncMotion::start([] { return driveTo(100_in, 5_s); });
    CHECK(drive.waitUntilElapsed(150_ms));
    CHECK(hw.currentVolts() > 0.0);
    const std::uint32_t asked = wallMs();
    CHECK(drive.cancel() == MotionResult::Cancelled);
    const std::uint32_t took = wallMs() - asked;
    std::printf("   cancel() returned after %u ms\n", static_cast<unsigned>(took));
    CHECK(took < 60);
    CHECK_EQ(hw.currentVolts(), 0.0);
    CHECK(!cancelRequested(CancelToken::Motion));
    // Cancelling again is harmless.
    CHECK(drive.cancel() == MotionResult::Cancelled);
  }

  std::printf("-- starting a second motion cancels the first\n");
  {
    hw.reset();
    AsyncMotion first = AsyncMotion::start([] { return driveTo(100_in, 5_s); });
    CHECK(first.waitUntilElapsed(50_ms));
    AsyncMotion second = AsyncMotion::start([] { return driveTo(-6_in, 2_s); });
    CHECK(!first.isRunning());
    CHECK(first.result() == MotionResult::Cancelled);
    CHECK(second.isRunning());
    CHECK(second.wait() == MotionResult::Reached);
  }

  std::printf("-- dropping the handle cancels the motion\n");
  {
    hw.reset();
    {
      AsyncMotion drive = AsyncMotion::start([] { return driveTo(100_in, 5_s); });
      CHECK(drive.waitUntilElapsed(50_ms));
    }
    CHECK_EQ(hw.currentVolts(), 0.0);
    const double stopped_at = robotState().pose().y;
    pros::delay(100);
    CHECK_NEAR(robotState().pose().y, stopped_at, 1e-9);
  }

  std::printf("-- a wait that the motion outlives returns false\n");
  {
    hw.reset();
    AsyncMotion quick = AsyncMotion::start([] { return driveTo(100_in, 100_ms); });
    CHECK(!quick.waitUntilElapsed(2_s));
    CHECK(quick.result() == MotionResult::TimedOut);
    CHECK(!quick.waitUntilTravelled(1_in));
    CHECK(!quick.waitUntil([] { return false; }));
  }

  std::printf("-- moving a handle keeps the motion\n");
  {
    hw.reset();
    AsyncMotion a = AsyncMotion::start([] { return driveTo(6_in, 2_s); });
    AsyncMotion b = std::move(a);
    CHECK(!a.valid());
    CHECK(b.isRunning());
    CHECK(b.wait() == MotionResult::Reached);
  }

  bindDrive(nullptr);
  return mclib::test::summary("async motion");
}
