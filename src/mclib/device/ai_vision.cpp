// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/ai_vision.hpp"

#include "pros/error.h"

#include <algorithm>

namespace mclib {
namespace device {

namespace {

pros::AivisionModeType toPros(AiVisionMode mode) {
  return static_cast<pros::AivisionModeType>(static_cast<std::uint8_t>(mode));
}

std::optional<AiVisionType> typeOf(std::uint8_t raw) {
  switch (raw) {
    case pros::E_AIVISION_DETECTED_COLOR:
      return AiVisionType::Color;
    case pros::E_AIVISION_DETECTED_CODE:
      return AiVisionType::Code;
    case pros::E_AIVISION_DETECTED_OBJECT:
      return AiVisionType::Object;
    case pros::E_AIVISION_DETECTED_TAG:
      return AiVisionType::Tag;
    default:
      // PROS hands back an object with a type outside the enum when the
      // read failed. There is nothing usable in it.
      return std::nullopt;
  }
}

}  // namespace

AiVision::AiVision(std::uint8_t port) : m_camera(port) {}

std::int32_t AiVision::enable(AiVisionMode mode) {
  return m_camera.enable_detection_types(toPros(mode));
}

std::int32_t AiVision::disable(AiVisionMode mode) {
  return m_camera.disable_detection_types(toPros(mode));
}

std::int32_t AiVision::enabledMask() const {
  return m_camera.get_enabled_detection_types();
}

std::int32_t AiVision::count() const {
  return m_camera.get_object_count();
}

std::optional<AiVisionObject> AiVision::convert(
    const pros::AIVision::Object& raw) {
  const std::optional<AiVisionType> type = typeOf(raw.type);
  if (!type) {
    return std::nullopt;
  }

  AiVisionObject out{};
  out.type = *type;
  out.id = raw.id;
  out.score = 0;
  out.angle_deg = 0.0;

  switch (*type) {
    case AiVisionType::Color:
    case AiVisionType::Code: {
      const auto& c = raw.object.color;
      out.left = c.xoffset;
      out.top = c.yoffset;
      out.width = c.width;
      out.height = c.height;
      out.angle_deg = c.angle / 10.0;
      break;
    }
    case AiVisionType::Object: {
      const auto& e = raw.object.element;
      out.left = e.xoffset;
      out.top = e.yoffset;
      out.width = e.width;
      out.height = e.height;
      out.score = e.score;
      break;
    }
    case AiVisionType::Tag: {
      const auto& t = raw.object.tag;
      const std::int32_t xs[4] = {t.x0, t.x1, t.x2, t.x3};
      const std::int32_t ys[4] = {t.y0, t.y1, t.y2, t.y3};
      const std::int32_t left = *std::min_element(xs, xs + 4);
      const std::int32_t right = *std::max_element(xs, xs + 4);
      const std::int32_t top = *std::min_element(ys, ys + 4);
      const std::int32_t bottom = *std::max_element(ys, ys + 4);
      out.left = left;
      out.top = top;
      out.width = right - left;
      out.height = bottom - top;
      break;
    }
  }

  out.center_x = out.left + out.width / 2;
  out.center_y = out.top + out.height / 2;
  return out;
}

std::vector<AiVisionObject> AiVision::objects() const {
  std::vector<AiVisionObject> found;
  const std::int32_t n = m_camera.get_object_count();
  if (n == PROS_ERR || n <= 0) {
    return found;
  }
  found.reserve(static_cast<std::size_t>(n));
  for (std::int32_t i = 0; i < n; ++i) {
    if (const auto object =
            convert(m_camera.get_object(static_cast<std::uint32_t>(i)))) {
      found.push_back(*object);
    }
  }
  return found;
}

std::vector<AiVisionObject> AiVision::objects(AiVisionType type) const {
  std::vector<AiVisionObject> all = objects();
  std::erase_if(all, [type](const AiVisionObject& o) { return o.type != type; });
  return all;
}

std::optional<AiVisionObject> AiVision::largest(AiVisionType type) const {
  std::optional<AiVisionObject> best;
  for (const AiVisionObject& o : objects()) {
    if (o.type == type && (!best || o.area() > best->area())) {
      best = o;
    }
  }
  return best;
}

std::optional<AiVisionObject> AiVision::largest(AiVisionType type,
                                                std::uint8_t id) const {
  std::optional<AiVisionObject> best;
  for (const AiVisionObject& o : objects()) {
    if (o.type == type && o.id == id && (!best || o.area() > best->area())) {
      best = o;
    }
  }
  return best;
}

std::int32_t AiVision::setColor(std::uint8_t id, std::uint8_t red,
                                std::uint8_t green, std::uint8_t blue,
                                float hue_range, float saturation_range) {
  pros::AIVision::Color color{};
  color.id = id;
  color.red = red;
  color.green = green;
  color.blue = blue;
  color.hue_range = hue_range;
  color.saturation_range = saturation_range;
  return m_camera.set_color(color);
}

std::int32_t AiVision::setTagFamily(pros::AivisionTagFamily family,
                                    bool override_existing) {
  return m_camera.set_tag_family(family, override_existing);
}

std::int32_t AiVision::startAutoWhiteBalance() {
  return m_camera.start_awb();
}

std::optional<std::string> AiVision::className(std::int32_t id) const {
  return m_camera.get_class_name(id);
}

std::int32_t AiVision::reset() {
  return m_camera.reset();
}

}  // namespace device
}  // namespace mclib
