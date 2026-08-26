// mclib
#include "mclib/device/distance.hpp"

namespace mclib {
namespace device {

Distance::Distance(std::uint8_t port) : m_distance(port) {}

std::int32_t Distance::getDistanceMm() const {
  return m_distance.get_distance();
}

double Distance::getDistanceIn() const {
  return static_cast<double>(getDistanceMm()) / 25.4;
}

std::int32_t Distance::getConfidence() const {
  return m_distance.get_confidence();
}

}  // namespace device
}  // namespace mclib
