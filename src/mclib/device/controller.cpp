// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/controller.hpp"

#include <cstdarg>
#include <cstdio>

namespace mclib {
namespace device {

namespace {
// Every pros::Controller call returns PROS_ERR on failure.
bool ok(std::int32_t result) { return result != PROS_ERR; }

std::optional<std::int32_t> value(std::int32_t result) {
  if (result == PROS_ERR) return std::nullopt;
  return result;
}
}  // namespace

Controller::Controller(ControllerId id)
    : m_id(id), m_controller(toProsControllerId(id)) {}

bool Controller::isConnected() const {
  const std::int32_t result = m_controller.is_connected();
  return ok(result) && result != 0;
}

std::int32_t Controller::getAnalog(AnalogAxis axis) const {
  return m_controller.get_analog(toProsAnalogAxis(axis));
}

units::QNumber Controller::analog(AnalogAxis axis) const {
  const std::int32_t raw = getAnalog(axis);
  // A disconnected controller returns PROS_ERR, not a stick position.
  if (!ok(raw)) return 0.0;
  return static_cast<double>(raw) / 127.0;
}

bool Controller::getDigital(DigitalButton button) const {
  const std::int32_t result = m_controller.get_digital(toProsDigitalButton(button));
  return ok(result) && result != 0;
}

bool Controller::getNewPress(DigitalButton button) const {
  const std::int32_t result = m_controller.get_digital_new_press(toProsDigitalButton(button));
  return ok(result) && result != 0;
}

bool Controller::getNewRelease(DigitalButton button) const {
  const std::int32_t result = m_controller.get_digital_new_release(toProsDigitalButton(button));
  return ok(result) && result != 0;
}

bool Controller::setText(std::uint8_t line, std::uint8_t col, const char* text) {
  return text != nullptr && ok(m_controller.set_text(line, col, text));
}

bool Controller::setText(std::uint8_t line, std::uint8_t col, const std::string& text) {
  return setText(line, col, text.c_str());
}

bool Controller::print(std::uint8_t line, std::uint8_t col, const char* format, ...) {
  if (format == nullptr) return false;
  // 15 columns, plus room for the terminator.
  char buffer[16];
  va_list args;
  va_start(args, format);
  const int written = std::vsnprintf(buffer, sizeof buffer, format, args);
  va_end(args);
  if (written < 0) return false;
  return setText(line, col, buffer);
}

bool Controller::clearLine(std::uint8_t line) { return ok(m_controller.clear_line(line)); }

bool Controller::clear() { return ok(m_controller.clear()); }

bool Controller::rumble(const char* pattern) {
  return pattern != nullptr && ok(m_controller.rumble(pattern));
}

std::optional<std::int32_t> Controller::batteryCapacity() const {
  return value(m_controller.get_battery_capacity());
}

std::optional<std::int32_t> Controller::batteryLevel() const {
  return value(m_controller.get_battery_level());
}

}  // namespace device
}  // namespace mclib
