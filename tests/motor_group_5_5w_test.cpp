// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// 11 W and 5.5 W motors in one MotorGroup or Chassis, against the stand-ins in
// tests/support/host_devices.cpp.

// chassis.cpp is not in HOST_TEST_SRC, so its source is pulled in here, the
// way chassis_set_pose_test.cpp does.
#undef _GNU_SOURCE
#define _GNU_SOURCE

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "../src/mclib/chassis/chassis.cpp"  // NOLINT
#pragma GCC diagnostic pop

#include "mclib/device/motor_group.hpp"
#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <unistd.h>

namespace host = mclib::test::host_devices;
using mclib::device::Gearset;
using mclib::device::MotorGroup;
using namespace mclib::units::literals;

namespace {

constexpr std::int32_t kType11W = 0;
constexpr std::int32_t kType5_5W = 1;
constexpr std::int32_t kTypeNotReporting = INT32_MAX;

std::string printedBy(void (*action)()) {
  std::fflush(stdout);
  std::FILE* capture = std::tmpfile();
  const int saved = dup(fileno(stdout));
  dup2(fileno(capture), fileno(stdout));
  action();
  std::fflush(stdout);
  dup2(saved, fileno(stdout));
  close(saved);
  std::rewind(capture);
  std::string text;
  char buffer[256];
  while (std::fgets(buffer, sizeof(buffer), capture) != nullptr) text += buffer;
  std::fclose(capture);
  return text;
}

void readingsAreOnTheElevenWattShaft() {
  host::reset();
  host::motors[3].type = kType5_5W;
  MotorGroup blue({1, 2}, Gearset::Blue, {3});
  host::motors[1].position_deg = 300.0;
  host::motors[2].position_deg = 300.0;
  host::motors[3].position_deg = 100.0;  // a third as far: 200 rpm against 600
  host::motors[1].rpm = 600.0;
  host::motors[2].rpm = 600.0;
  host::motors[3].rpm = 200.0;
  const std::vector<double> positions = blue.getPositionsDeg();
  CHECK_EQ(positions.size(), std::size_t{3});
  CHECK_NEAR(positions[2], 300.0, 1e-9);
  CHECK_NEAR(blue.getAveragePositionDeg(), 300.0, 1e-9);
  CHECK_NEAR(blue.getAverageActualVelocity(), 600.0, 1e-9);
  CHECK_NEAR(blue.averagePosition()->deg(), 300.0, 1e-9);

  host::reset();
  host::motors[3].type = kType5_5W;
  MotorGroup red({1}, Gearset::Red, {3});
  host::motors[3].position_deg = 200.0;
  CHECK_NEAR(red.getPositionsDeg()[1], 100.0, 1e-9);

  host::reset();
  host::motors[3].type = kType5_5W;
  MotorGroup green({1}, Gearset::Green, {3});
  host::motors[3].position_deg = 200.0;
  CHECK_NEAR(green.getPositionsDeg()[1], 200.0, 1e-9);
}

void aDeadFiveFiveWattMotorStaysDead() {
  host::reset();
  host::motors[3].type = kType5_5W;
  MotorGroup group({1}, Gearset::Blue, {3});
  host::motors[1].position_deg = 90.0;
  host::motors[3].position_deg = INFINITY;
  CHECK(std::isinf(group.getPositionsDeg()[1]));
  CHECK_NEAR(group.getAveragePositionDeg(), 90.0, 1e-9);
  CHECK(!group.averagePosition().has_value());
}

void everyMotorGetsTheSameFractionOfFullSpeed() {
  host::reset();
  host::motors[3].type = kType5_5W;
  MotorGroup group({-1, 2}, Gearset::Blue, {-3});
  group.setVoltage(6.0);
  CHECK_NEAR(host::motors[1].volts(), 6.0, 1e-9);
  CHECK_NEAR(host::motors[2].volts(), 6.0, 1e-9);
  CHECK_NEAR(host::motors[3].volts(), 4.0, 1e-9);
  CHECK(host::motors[3].reversed);
  group.setVoltage(12.0);
  CHECK_NEAR(host::motors[3].volts(), mclib::device::kMotor5_5WFullVolts, 1e-9);
  group.setVoltage(20.0);
  CHECK_NEAR(host::motors[1].volts(), 12.0, 1e-9);
  CHECK_NEAR(host::motors[3].volts(), mclib::device::kMotor5_5WFullVolts, 1e-9);
  group.setVoltage(-12.0);
  CHECK_NEAR(host::motors[3].volts(), -mclib::device::kMotor5_5WFullVolts, 1e-9);
  group.setVoltage(NAN);
  CHECK_NEAR(host::motors[1].volts(), 0.0, 1e-9);
  CHECK_NEAR(host::motors[3].volts(), 0.0, 1e-9);
  group.setPercent(0.5);
  CHECK_NEAR(host::motors[1].volts(), 64.0 / 127.0 * 12.0, 1e-9);
  CHECK_NEAR(host::motors[3].volts(), 64.0 / 127.0 * 8.0, 1e-3);
  group.setPercent(1.0);
  CHECK_NEAR(host::motors[3].volts(), 8.0, 1e-9);
  group.brake();
  CHECK(host::motors[1].braked && host::motors[2].braked && host::motors[3].braked);
}

void theConversionsAreMclibs() {
  CHECK_NEAR(mclib::device::voltsFor5_5WMotor(12.0), mclib::device::kMotor5_5WFullVolts, 1e-12);
  CHECK_NEAR(mclib::device::voltsFor5_5WMotor(-3.0), -2.0, 1e-12);
  CHECK_NEAR(mclib::device::readingScaleFor5_5WMotor(Gearset::Blue), 3.0, 1e-12);
  CHECK_NEAR(mclib::device::readingScaleFor5_5WMotor(Gearset::Green), 1.0, 1e-12);
  CHECK_NEAR(mclib::device::readingScaleFor5_5WMotor(Gearset::Red), 0.5, 1e-12);
}

void tareAndBrakeModeReachEveryMotor() {
  host::reset();
  MotorGroup group({1}, Gearset::Blue, {3});
  host::motors[1].position_deg = 50.0;
  host::motors[3].position_deg = 70.0;
  group.tarePosition();
  CHECK_NEAR(host::motors[1].position_deg, 0.0, 1e-9);
  CHECK_NEAR(host::motors[3].position_deg, 0.0, 1e-9);
  group.setBrakeMode(mclib::device::BrakeMode::Hold);
  CHECK_EQ(host::motors[1].brake_mode, static_cast<int>(pros::E_MOTOR_BRAKE_HOLD));
  CHECK_EQ(host::motors[3].brake_mode, static_cast<int>(pros::E_MOTOR_BRAKE_HOLD));
}

void theOneListGroupIsUnchanged() {
  host::reset();
  host::motors[1].type = kType5_5W;
  host::motors[2].type = kType5_5W;
  MotorGroup only_5_5w({1, 2});
  host::motors[1].position_deg = 100.0;
  host::motors[2].position_deg = 100.0;
  CHECK_NEAR(only_5_5w.getAveragePositionDeg(), 100.0, 1e-9);
  only_5_5w.setVoltage(6.0);
  CHECK_NEAR(host::motors[1].volts(), 6.0, 1e-9);
}

void aFiftyFiveWattChassisMeasuresTheWheels() {
  host::reset();
  host::motors[5].type = kType5_5W;
  host::motors[6].type = kType5_5W;
  const mclib::ChassisDimensions dimensions{mclib::units::Wheel::fromDiameter(3.25_in), 11.5_in,
                                            0.75};
  mclib::Chassis drive({-1, 2}, {3, -4}, Gearset::Blue, {-5}, {6}, dimensions);
  // One wheel turn: 480 deg of a blue motor through 36:48, 160 deg of a
  // 5.5 W motor geared to the same wheel speed.
  for (const int port : {1, 2, 3, 4}) host::motors[port].position_deg = 480.0;
  host::motors[5].position_deg = 160.0;
  host::motors[6].position_deg = 160.0;
  const double wheel_turn_in = 3.25 * M_PI;
  CHECK_NEAR(drive.leftDistanceIn(), wheel_turn_in, 1e-9);
  CHECK_NEAR(drive.rightDistanceIn(), wheel_turn_in, 1e-9);
  drive.setDriveVoltage(9.0, -9.0);
  CHECK_NEAR(host::motors[1].volts(), 9.0, 1e-9);
  CHECK_NEAR(host::motors[5].volts(), 6.0, 1e-9);
  CHECK_NEAR(host::motors[6].volts(), -6.0, 1e-9);
  CHECK_EQ(drive.driveVelocitiesRpm().size(), std::size_t{6});
}

void aMotorInTheWrongListIsReportedOnce() {
  const std::string mixed_in_one_list = printedBy([] {
    host::reset();
    host::motors[2].type = kType5_5W;
    MotorGroup group({1, 2});
    group.setVoltage(3.0);
    group.setVoltage(4.0);
  });
  CHECK(mixed_in_one_list.find("port 2 is a 5.5 W motor, listed with the 11 W motors") !=
        std::string::npos);
  CHECK(mixed_in_one_list.find("mclib:") == mixed_in_one_list.rfind("mclib:"));

  const std::string eleven_watt_in_the_5_5w_list = printedBy([] {
    host::reset();
    MotorGroup group({1}, Gearset::Blue, {-2});
    group.setPercent(0.2);
  });
  CHECK(eleven_watt_in_the_5_5w_list.find("port 2 is an 11 W motor, listed with the 5.5 W "
                                          "motors") != std::string::npos);

  const std::string waits_for_every_motor = printedBy([] {
    host::reset();
    host::motors[1].type = kTypeNotReporting;
    host::motors[2].type = kType5_5W;
    MotorGroup group({1, 2});
    group.setVoltage(3.0);
    std::printf("plugged in\n");
    host::motors[1].type = kType11W;
    group.setVoltage(3.0);
  });
  CHECK(waits_for_every_motor.find("plugged in\nmclib: the motor on port 2") != std::string::npos);

  const std::string listed_right = printedBy([] {
    host::reset();
    host::motors[3].type = kType5_5W;
    MotorGroup group({1, 2}, Gearset::Blue, {3});
    group.setVoltage(3.0);
  });
  CHECK(listed_right.empty());
}

}  // namespace

int main() {
  readingsAreOnTheElevenWattShaft();
  aDeadFiveFiveWattMotorStaysDead();
  everyMotorGetsTheSameFractionOfFullSpeed();
  theConversionsAreMclibs();
  tareAndBrakeModeReachEveryMotor();
  theOneListGroupIsUnchanged();
  aFiftyFiveWattChassisMeasuresTheWheels();
  aMotorInTheWrongListIsReportedOnce();
  return mclib::test::summary("motor_group_5_5w");
}
