// mclib
#include "mclib/device/distance.hpp"

#include "pros/error.h"

namespace mclib {
namespace device {

Distance::Distance(std::uint8_t port) : m_distance(port) {}

std::int32_t Distance::getDistanceMm() const {
  return m_distance.get_distance();
}

double Distance::getDistanceIn() const {
  return static_cast<double>(getDistanceMm()) / 25.4;
}

std::optional<units::QLength> Distance::distance() const {
  const std::int32_t mm = m_distance.get_distance();
  if (mm == PROS_ERR) {
    return std::nullopt;
  }
  return static_cast<double>(mm) * units::millimetre;
}

std::int32_t Distance::getConfidence() const {
  return m_distance.get_confidence();
}

}  // namespace device
}  // namespace mclib
