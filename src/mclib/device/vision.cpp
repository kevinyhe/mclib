// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/vision.hpp"

#include "pros/error.h"

namespace mclib {
namespace device {

Vision::Vision(std::uint8_t port) : m_vision(port, pros::E_VISION_ZERO_CENTER) {}

std::int32_t Vision::count() const {
  return m_vision.get_object_count();
}

std::optional<VisionObject> Vision::convert(const pros::vision_object_s_t& raw) {
  if (raw.signature == VISION_OBJECT_ERR_SIG) {
    return std::nullopt;
  }
  return VisionObject{raw.signature, raw.x_middle_coord, raw.y_middle_coord,
                      raw.width, raw.height};
}

std::optional<VisionObject> Vision::largest(std::uint8_t signature_id) const {
  return convert(m_vision.get_by_sig(0, signature_id));
}

std::optional<VisionObject> Vision::largest() const {
  return convert(m_vision.get_by_size(0));
}

std::vector<VisionObject> Vision::objects(std::uint8_t signature_id,
                                          std::size_t max_objects) const {
  std::vector<VisionObject> found;
  if (max_objects == 0) {
    return found;
  }
  std::vector<pros::vision_object_s_t> buffer(max_objects);
  const std::int32_t copied = m_vision.read_by_sig(
      0, signature_id, static_cast<std::uint32_t>(max_objects), buffer.data());
  if (copied == PROS_ERR || copied <= 0) {
    return found;
  }
  found.reserve(static_cast<std::size_t>(copied));
  for (std::int32_t i = 0; i < copied; ++i) {
    if (const auto object = convert(buffer[static_cast<std::size_t>(i)])) {
      found.push_back(*object);
    }
  }
  return found;
}

std::int32_t Vision::setSignature(std::uint8_t id, std::int32_t u_min,
                                  std::int32_t u_max, std::int32_t u_mean,
                                  std::int32_t v_min, std::int32_t v_max,
                                  std::int32_t v_mean, float range,
                                  std::int32_t type) {
  pros::vision_signature_s_t signature = pros::Vision::signature_from_utility(
      id, u_min, u_max, u_mean, v_min, v_max, v_mean, range, type);
  return m_vision.set_signature(id, &signature);
}

std::int32_t Vision::setSignature(std::uint8_t id,
                                  pros::vision_signature_s_t signature) {
  return m_vision.set_signature(id, &signature);
}

pros::vision_signature_s_t Vision::getSignature(std::uint8_t id) const {
  return m_vision.get_signature(id);
}

std::int32_t Vision::setExposure(std::uint8_t exposure) {
  return m_vision.set_exposure(exposure);
}

std::int32_t Vision::getExposure() const {
  return m_vision.get_exposure();
}

std::int32_t Vision::setWhiteBalance(std::uint32_t rgb) {
  return m_vision.set_white_balance(static_cast<std::int32_t>(rgb));
}

std::int32_t Vision::setAutoWhiteBalance(bool enable) {
  return m_vision.set_auto_white_balance(enable ? 1 : 0);
}

std::int32_t Vision::setLed(std::uint32_t rgb) {
  return m_vision.set_led(static_cast<std::int32_t>(rgb));
}

std::int32_t Vision::clearLed() {
  return m_vision.clear_led();
}

}  // namespace device
}  // namespace mclib
