// mclib
#pragma once

#include <cstdint>

#include "api.h"

namespace mclib {
namespace device {

class Distance {
public:
  explicit Distance(std::uint8_t port);

  std::int32_t getDistanceMm() const;
  double getDistanceIn() const;
  std::int32_t getConfidence() const;

private:
  mutable pros::Distance m_distance;
};

}  // namespace device
}  // namespace mclib
