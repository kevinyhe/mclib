// mclib
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
  return raw * imu_gain;
}
