// mclib
#include "mclib/device/inertial.hpp"

#include "pros/error.h"

#include <cmath>

namespace mclib {
namespace device {

Inertial::Inertial(std::uint8_t port, double gain)
    : m_imu(port), m_gain(gain) {}

void Inertial::reset(bool blocking) {
  m_imu.reset(blocking);
}

double Inertial::getRotationDeg() const {
  const double raw = m_imu.get_rotation();
  if (raw == PROS_ERR_F) {
    return NAN;
  }
  return raw * m_gain;
}

double Inertial::getHeadingDeg() const {
  const double raw = m_imu.get_heading();
  if (raw == PROS_ERR_F) {
    return NAN;
  }
  return raw * m_gain;
}

void Inertial::setRotationDeg(double heading_deg) {
  m_imu.set_rotation(heading_deg / m_gain);
}

}  // namespace device
}  // namespace mclib
