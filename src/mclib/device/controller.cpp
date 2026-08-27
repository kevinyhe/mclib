// mclib
#include "mclib/device/controller.hpp"

namespace mclib {
namespace device {

Controller::Controller(ControllerId id)
    : m_controller(toProsControllerId(id)) {}

std::int32_t Controller::getAnalog(AnalogAxis axis) const {
  return m_controller.get_analog(toProsAnalogAxis(axis));
}

units::QNumber Controller::analog(AnalogAxis axis) const {
  return static_cast<double>(getAnalog(axis)) / 127.0;
}

bool Controller::getDigital(DigitalButton button) const {
  return m_controller.get_digital(toProsDigitalButton(button));
}

void Controller::setText(std::uint8_t line, std::uint8_t col, const char* text) {
  m_controller.set_text(line, col, text);
}

}  // namespace device
}  // namespace mclib
