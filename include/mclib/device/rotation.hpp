// mclib
#pragma once

#include <cstdint>

#include "api.h"

namespace mclib {
namespace device {

class Rotation {
public:
  explicit Rotation(std::int8_t port, bool reversed = false);

  double getPositionDeg() const;
  void resetPosition();
  bool isInstalled() const;

private:
  mutable pros::Rotation m_rotation;
};

}  // namespace device
}  // namespace mclib
