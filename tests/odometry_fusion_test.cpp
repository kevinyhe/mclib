// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// OdometryFusion: a pose that is driven is stepped through the filter and
// corrected; a pose that is set restarts the filter where the robot now is.

#include "mclib/control/odometry_fusion.hpp"
#include "mclib/math.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <optional>

using mclib::Pose2D;
using mclib::control::OdometryFusion;
using mclib::control::OdometryReading;
using mclib::control::PoseFusion;
using mclib::control::PoseFusionConfig;
using mclib::control::readOdometry;
using mclib::control::resetOdometry;

namespace {

constexpr double kMapCentreIn = 72.0;
constexpr double kStartSigmaIn = 0.5;

// Moves the pose without a jump, as a tick of driving does.
void drive(double dx_in, double dy_in) {
  const OdometryReading now = readOdometry();
  CHECK(mclib::control::correctOdometryPosition(now.pose.x + dx_in, now.pose.y + dy_in,
                                                now.jumps));
}

// A GPS that reports the robot at a fixed place on the map.
mclib::control::GpsInput gpsAt(std::function<std::optional<Pose2D>()> on_map) {
  mclib::control::GpsInput gps;
  gps.read = [on_map]() -> std::optional<Pose2D> {
    const std::optional<Pose2D> pose = on_map();
    if (!pose.has_value()) return std::nullopt;
    return Pose2D{pose->x - kMapCentreIn, pose->y - kMapCentreIn, pose->theta};
  };
  return gps;
}

void aPoseSetRightAfterTheStartIsAJump() {
  std::printf("-- a pose set between the start and the first step is a jump\n");
  resetOdometry({20, 20, 0});
  PoseFusion fusion(PoseFusionConfig{}, {});
  OdometryFusion on_odometry(fusion);

  // setPose() in initialize(), before the odometry task has run once.
  resetOdometry({36, 7, 0});
  on_odometry.step(10);

  CHECK_NEAR(fusion.pose().x, 36.0, 1e-9);
  CHECK_NEAR(fusion.pose().y, 7.0, 1e-9);
  // Stepped across, the 21 in would count as driven and leave it 1.4 in unsure.
  CHECK_NEAR(fusion.positionSigmaIn(), kStartSigmaIn, 1e-9);
}

void drivingIsSteppedAndCorrected() {
  std::printf("-- a driven pose is stepped, and corrected without a jump\n");
  resetOdometry({40, 40, 0});
  PoseFusion fusion(PoseFusionConfig{}, {});
  // Odometry is 2 in left of where the GPS has the robot, all the way up.
  fusion.setGps(gpsAt([] { return Pose2D{42, readOdometry().pose.y, 0}; }));
  OdometryFusion on_odometry(fusion);
  const std::uint32_t jumps_at_start = mclib::control::odometryJumpCount();

  std::uint32_t now_ms = 0;
  for (int tick = 0; tick < 120; ++tick) {
    drive(0, 0.25);
    on_odometry.step(now_ms += 10);
  }

  const Pose2D pose = readOdometry().pose;
  std::printf("   odometry x %.2f in after 30 in, GPS at 42\n", pose.x);
  CHECK(pose.x > 41.0);
  CHECK(pose.x < 42.5);
  CHECK_NEAR(pose.y, 70.0, 0.01);
  CHECK(mclib::control::odometryJumpCount() == jumps_at_start);
}

void aJumpDropsTheCorrectionTheFilterStillOwed() {
  std::printf("-- a jump restarts the filter at the new pose\n");
  resetOdometry({40, 40, 0});
  PoseFusion fusion(PoseFusionConfig{}, {});
  bool gps_has_a_reading = true;
  fusion.setGps(gpsAt([&]() -> std::optional<Pose2D> {
    if (!gps_has_a_reading) return std::nullopt;
    return Pose2D{42, 40, 0};
  }));
  OdometryFusion on_odometry(fusion);

  // Standing still, odometry moves 0.002 in a tick, so the filter gets ahead.
  std::uint32_t now_ms = 0;
  for (int tick = 0; tick < 40; ++tick) on_odometry.step(now_ms += 10);
  const double owed_in = fusion.pose().x - readOdometry().pose.x;
  std::printf("   filter %.2f in ahead of odometry before the jump\n", owed_in);
  CHECK(owed_in > 1.0);

  gps_has_a_reading = false;
  resetOdometry({10, 100, mclib::kPi / 2});
  on_odometry.step(now_ms += 10);
  CHECK_NEAR(fusion.pose().x, 10.0, 1e-9);
  CHECK_NEAR(fusion.pose().y, 100.0, 1e-9);
  CHECK_NEAR(fusion.pose().theta, mclib::kPi / 2, 1e-9);

  on_odometry.step(now_ms += 10);
  CHECK_NEAR(readOdometry().pose.x, 10.0, 1e-9);
  CHECK_NEAR(readOdometry().pose.y, 100.0, 1e-9);
}

void aCorrectionWorkedOutBeforeAJumpIsNotWritten() {
  std::printf("-- a pose set while a step runs is not overwritten by that step\n");
  resetOdometry({40, 40, 0});
  PoseFusion fusion(PoseFusionConfig{}, {});
  bool set_the_pose_during_this_read = false;
  fusion.setGps(gpsAt([&]() -> std::optional<Pose2D> {
    if (set_the_pose_during_this_read) {
      set_the_pose_during_this_read = false;
      resetOdometry({10, 100, 0});
    }
    return Pose2D{42, 40, 0};
  }));
  OdometryFusion on_odometry(fusion);

  std::uint32_t now_ms = 0;
  for (int tick = 0; tick < 40; ++tick) on_odometry.step(now_ms += 10);
  CHECK(readOdometry().pose.x > 40.0);

  // The step has read (40.08, 40) and works out a correction for it. Another
  // task calls setPose() before the correction is written.
  set_the_pose_during_this_read = true;
  on_odometry.step(now_ms += 20);
  CHECK(!set_the_pose_during_this_read);
  CHECK_NEAR(readOdometry().pose.x, 10.0, 1e-9);
  CHECK_NEAR(readOdometry().pose.y, 100.0, 1e-9);
}

void aRestartIsNoSurerThanTheFilterWas() {
  std::printf("-- a restart keeps the filter's uncertainty when it is larger\n");
  resetOdometry({20, 20, 0});
  PoseFusion fusion(PoseFusionConfig{}, {});
  OdometryFusion on_odometry(fusion);

  // 60 in with no sensor reading.
  std::uint32_t now_ms = 0;
  for (int tick = 0; tick < 240; ++tick) {
    drive(0, 0.25);
    on_odometry.step(now_ms += 10);
  }
  const double sigma_before_in = fusion.positionSigmaIn();
  std::printf("   %.2f in unsure after 60 in\n", sigma_before_in);
  CHECK(sigma_before_in > 2.0);

  // wallReset() on the y = 82.5 wall: y is known now, x is as it was.
  resetOdometry({20, 82.5, 0});
  on_odometry.step(now_ms += 10);
  CHECK_NEAR(fusion.pose().y, 82.5, 1e-9);
  CHECK_NEAR(fusion.positionSigmaIn(), sigma_before_in, 1e-9);
}

void aSnapshotsCorrectionIsAJump() {
  std::printf("-- a position written from outside restarts the filter too\n");
  resetOdometry({40, 40, 0});
  PoseFusion fusion(PoseFusionConfig{}, {});
  OdometryFusion on_odometry(fusion);
  on_odometry.step(10);

  CHECK(mclib::control::correctOdometryPosition(43.0, 38.0));
  on_odometry.step(20);
  CHECK_NEAR(fusion.pose().x, 43.0, 1e-9);
  CHECK_NEAR(fusion.pose().y, 38.0, 1e-9);
  CHECK_NEAR(fusion.positionSigmaIn(), kStartSigmaIn, 1e-9);
}

}  // namespace

int main() {
  aPoseSetRightAfterTheStartIsAJump();
  drivingIsSteppedAndCorrected();
  aJumpDropsTheCorrectionTheFilterStillOwed();
  aCorrectionWorkedOutBeforeAJumpIsNotWritten();
  aRestartIsNoSurerThanTheFilterWas();
  aSnapshotsCorrectionIsAJump();
  return mclib::test::summary("odometry_fusion");
}
