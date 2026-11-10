// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Chassis::setPose() moves the IMU, the odometry pose and the heading that
// driveTo() and correctHeading() hold, together.
//
// chassis.cpp is not in HOST_TEST_SRC, so its source is pulled in here, the
// way the routine tests pull in autonomous_routine.cpp.
#undef _GNU_SOURCE
#define _GNU_SOURCE

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "../src/mclib/chassis/chassis.cpp"  // NOLINT
#pragma GCC diagnostic pop

#include "mclib/device/inertial.hpp"
#include "mclib/math.hpp"
#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <memory>

using mclib::Chassis;
using mclib::control::robotState;
using mclib::device::Inertial;
using namespace mclib::units::literals;
namespace host = mclib::test::host_devices;

namespace {

constexpr int kImuPort = 9;

const mclib::ChassisDimensions dimensions{mclib::units::Wheel::fromDiameter(2.75_in), 11.5_in, 1.0};

Chassis chassisWith(std::shared_ptr<Inertial> imu) {
  return Chassis({1, 2}, {-3, -4}, mclib::device::Gearset::Blue, dimensions, std::move(imu));
}

void theHeldHeadingMovesWithTheImu() {
  std::printf("-- setPose() moves the IMU, the pose and the held heading\n");
  host::reset();
  Chassis chassis = chassisWith(std::make_shared<Inertial>(kImuPort));
  robotState().setCorrectAngleDeg(0);

  chassis.setPose(36_in, 7_in, 90_deg);
  CHECK_NEAR(host::imu_rotation_deg[kImuPort], 90.0, 1e-9);
  CHECK_NEAR(robotState().correctAngleDeg(), 90.0, 1e-9);
  CHECK_NEAR(robotState().pose().x, 36.0, 1e-9);
  CHECK_NEAR(robotState().pose().y, 7.0, 1e-9);
  CHECK_NEAR(robotState().pose().theta, mclib::kPi / 2, 1e-9);

  // 270 deg is the same heading as -90, and that is what the IMU is given.
  chassis.setPose(36_in, 7_in, 270_deg);
  CHECK_NEAR(host::imu_rotation_deg[kImuPort], -90.0, 1e-9);
  CHECK_NEAR(robotState().correctAngleDeg(), -90.0, 1e-9);
}

void theHeldHeadingIsWhatTheImuReadsBack() {
  std::printf("-- the held heading is the IMU's reading after the write\n");
  host::reset();
  // A gain of 1.01: the sensor is told 90 / 1.01 and reads back 90.
  Chassis chassis = chassisWith(std::make_shared<Inertial>(kImuPort, 1.01));
  robotState().setCorrectAngleDeg(0);

  chassis.setPose(0_in, 0_in, 90_deg);
  CHECK_NEAR(host::imu_rotation_deg[kImuPort], 90.0 / 1.01, 1e-9);
  CHECK_NEAR(robotState().correctAngleDeg(), 90.0, 1e-9);
}

void anImuThatCannotBeReadLeavesTheHeldHeadingAlone() {
  std::printf("-- an IMU that cannot be read leaves the held heading alone\n");
  host::reset();
  // A gain of zero is a configuration error: the write is ignored and the
  // heading reads NaN.
  Chassis chassis = chassisWith(std::make_shared<Inertial>(kImuPort, 0.0));
  robotState().setCorrectAngleDeg(12.0);

  chassis.setPose(36_in, 7_in, 90_deg);
  CHECK_NEAR(robotState().correctAngleDeg(), 12.0, 1e-9);
  CHECK_NEAR(robotState().pose().x, 36.0, 1e-9);
}

void withoutAnImuTheEncoderHeadingMoves() {
  std::printf("-- with no IMU the encoder heading and the held heading move\n");
  host::reset();
  Chassis chassis = chassisWith(nullptr);
  robotState().setCorrectAngleDeg(0);

  chassis.setPose(10_in, 20_in, 45_deg);
  CHECK_NEAR(chassis.headingDeg(), 45.0, 1e-9);
  CHECK_NEAR(robotState().correctAngleDeg(), 45.0, 1e-9);
}

}  // namespace

int main() {
  theHeldHeadingMovesWithTheImu();
  theHeldHeadingIsWhatTheImuReadsBack();
  anImuThatCannotBeReadLeavesTheHeldHeadingAlone();
  withoutAnImuTheEncoderHeadingMoves();
  return mclib::test::summary("chassis_set_pose");
}
