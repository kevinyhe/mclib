// mclib
#pragma once

#include <cstdint>

#include "api.h"

namespace mclib {
namespace device {

class Inertial {
public:
  explicit Inertial(std::uint8_t port, double gain = 1.0);

  void reset(bool blocking = false);
  double getRotationDeg() const;
  double getHeadingDeg() const;
  void setRotationDeg(double heading_deg);

private:
  pros::Imu m_imu;
  double m_gain;
};

}  // namespace device
}  // namespace mclib
