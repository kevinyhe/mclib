// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// What the device wrappers return when PROS reports an error, and the
// rounding and wrapping fixes from the wrapper audit, against the stand-ins in
// tests/support/host_devices.cpp.

#include "mclib/device/controller.hpp"
#include "mclib/device/inertial.hpp"
#include "mclib/device/motor.hpp"
#include "mclib/device/motor_group.hpp"
#include "support/host_devices.hpp"
#include "test_assert.hpp"

#include <cmath>

namespace host = mclib::test::host_devices;
using mclib::device::Inertial;
using mclib::device::Motor;
using mclib::device::MotorGroup;

namespace {

void fullPercentReachesFullPower() {
  host::reset();
  Motor motor(1);
  // 0.999 * 127 = 126.9: truncating sent 126 (11.906 V).
  motor.setPercent(0.999);
  CHECK_NEAR(host::motors[1].millivolts, 12000.0, 1e-9);
  motor.setPercent(-0.999);
  CHECK_NEAR(host::motors[1].millivolts, -12000.0, 1e-9);
  motor.setPercent(0.5);  // 63.5 rounds to 64
  CHECK_NEAR(host::motors[1].millivolts, 64.0 / 127.0 * 12000.0, 1e-9);
}

void deadMotorReadsNan() {
  host::reset();
  Motor motor(2);
  host::motors[2].position_deg = INFINITY;
  host::motors[2].rpm = INFINITY;
  host::motors[2].current_ma = PROS_ERR;
  CHECK(std::isnan(motor.getPositionDeg()));
  CHECK(std::isnan(motor.getActualVelocity()));
  CHECK(std::isnan(motor.getCurrentDraw()));
  CHECK(!motor.current().has_value());
  // NaN fails the comparison a caller would write.
  CHECK(!(motor.getCurrentDraw() > 2500));
}

void groupAverageSkipsDeadMotors() {
  host::reset();
  MotorGroup group({3, 4, 5});
  host::motors[3].position_deg = 100;
  host::motors[4].position_deg = INFINITY;  // unplugged
  host::motors[5].position_deg = 200;
  CHECK_NEAR(group.getAveragePositionDeg(), 150.0, 1e-9);
  host::motors[3].current_ma = 1000;
  host::motors[4].current_ma = PROS_ERR;
  host::motors[5].current_ma = 2000;
  CHECK_NEAR(group.getAverageCurrentDraw(), 1500.0, 1e-9);
  // The strict typed average still refuses a group with a dead motor.
  CHECK(!group.averagePosition().has_value());
  // None reporting: NaN, not a believable 0.
  host::motors[3].position_deg = host::motors[5].position_deg = INFINITY;
  CHECK(std::isnan(group.getAveragePositionDeg()));
}

void headingStaysInRangeWithGain() {
  host::reset();
  Inertial fast(6, 1.01);
  host::imu_rotation_deg[6] = 359.0;
  const double heading = fast.getHeadingDeg();
  CHECK(heading >= 0.0 && heading < 360.0);
  CHECK_NEAR(heading, std::fmod(359.0 * 1.01, 360.0), 1e-9);
  // Rotation is not wrapped: it keeps counting.
  CHECK_NEAR(fast.getRotationDeg(), 359.0 * 1.01, 1e-9);

  CHECK(!fast.hasConfigurationError());

  // A gain that is not finite and > 0 reads NaN, like an unplugged IMU.
  for (const double gain : {0.0, -1.0, double{NAN}, double{INFINITY}}) {
    Inertial bad(7, gain);
    host::imu_rotation_deg[7] = 45.0;
    CHECK(bad.hasConfigurationError());
    CHECK(std::isnan(bad.getRotationDeg()));
    CHECK(std::isnan(bad.getHeadingDeg()));
    CHECK(!bad.rotation().has_value());
    bad.setRotationDeg(90.0);
    CHECK_NEAR(host::imu_rotation_deg[7], 45.0, 1e-9);
  }
}

void controllerRawStickErrorReadsZero() {
  host::reset();
  mclib::device::Controller master;
  host::analog[static_cast<int>(pros::E_CONTROLLER_ANALOG_LEFT_Y)] = PROS_ERR;
  CHECK_EQ(master.getAnalog(mclib::device::AnalogAxis::LeftY), 0);
}

}  // namespace

int main() {
  fullPercentReachesFullPower();
  deadMotorReadsNan();
  groupAverageSkipsDeadMotors();
  headingStaysInRangeWithGain();
  controllerRawStickErrorReadsZero();
  return mclib::test::summary("device_errors");
}
