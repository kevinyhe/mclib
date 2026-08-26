// mclib
#include "mclib/device/line.hpp"

#include <algorithm>

Line::Line(std::uint8_t adi_port,
           std::int32_t threshold,
           bool detect_dark_line)
    : sensor(adi_port),
      threshold_value(clamp_raw(threshold)),
      dark_line(detect_dark_line) {}

Line::Line(pros::ADIAnalogIn adi_port,
           std::int32_t threshold,
           bool detect_dark_line)
    : sensor(adi_port),
      threshold_value(clamp_raw(threshold)),
      dark_line(detect_dark_line) {}

bool Line::get() const {
  const std::int32_t value = get_raw();
  return dark_line ? (value >= threshold_value) : (value <= threshold_value);
}

std::int32_t Line::get_raw() const {
  return sensor.get_value();
}

void Line::set_threshold(std::int32_t threshold) {
  threshold_value = clamp_raw(threshold);
}

std::int32_t Line::get_threshold() const {
  return threshold_value;
}

void Line::set_detect_dark_line(bool detect_dark) {
  dark_line = detect_dark;
}

bool Line::detects_dark_line() const {
  return dark_line;
}

void Line::calibrate_midpoint(std::int32_t light_reading,
                              std::int32_t dark_reading) {
  const std::int32_t light = clamp_raw(light_reading);
  const std::int32_t dark = clamp_raw(dark_reading);
  threshold_value = (light + dark) / 2;
}

std::int32_t Line::clamp_raw(std::int32_t value) {
  return std::clamp<std::int32_t>(value, 0, 4095);
}
