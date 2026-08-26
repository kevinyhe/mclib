// mclib
#pragma once

#include "mclib/device/types.hpp"

#include <cstdint>

namespace mclib {
namespace device {

class Controller {
public:
  explicit Controller(ControllerId id = ControllerId::Master);

  std::int32_t getAnalog(AnalogAxis axis) const;
  bool getDigital(DigitalButton button) const;
  void setText(std::uint8_t line, std::uint8_t col, const char* text);

private:
  mutable pros::Controller m_controller;
};

}  // namespace device
}  // namespace mclib
