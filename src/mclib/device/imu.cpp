// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/imu.hpp"

#include "pros/error.h"

#include <cmath>

MockIMU::MockIMU(int port, double gain)
    : pros::Imu(port), imu_gain(gain) {}

double MockIMU::get_rotation() const {
  const double raw = pros::Imu::get_rotation();
  if (raw == PROS_ERR_F) {
    return NAN;
  }
  return raw * imu_gain;
}

double MockIMU::get_heading() const {
  const double raw = pros::Imu::get_heading();
  if (raw == PROS_ERR_F) {
    return NAN;
  }
  // Wrap after the gain: PROS's heading is already [0, 360), and scaling
  // it moved 359 to 362.6 with a gain of 1.01.
  const double heading = std::fmod(raw * imu_gain, 360.0);
  return heading < 0 ? heading + 360.0 : heading;
}
