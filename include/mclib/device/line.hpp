// mclib
#pragma once

#include "pros/adi.hpp"

#include <cstdint>

class Line {
public:
  /**
   * @param adi_port ADI port character
   * @param threshold Raw value threshold
   * @param detect_dark_line If true, returns true when value >= threshold
   *        If false, returns true when value <= threshold
   */
  explicit Line(std::uint8_t adi_port, std::int32_t threshold = 2000,
                bool detect_dark_line = true);

  /**
   * @param adi_port Pair form for ADI expander use
   */
  explicit Line(pros::ADIAnalogIn adi_port,
                std::int32_t threshold = 2000,
                bool detect_dark_line = true);

  /**
   * @brief Get whether a line is detected
   */
  bool get() const;

  /**
   * @brief Read raw sensor value in [0, 4095]
   */
  std::int32_t get_raw() const;

  /**
   * @brief Set line detection threshold
   */
  void set_threshold(std::int32_t threshold);

  /**
   * @brief Get configured threshold
   */
  std::int32_t get_threshold() const;

  /**
   * @brief Configure whether line is interpreted as dark or light
   */
  void set_detect_dark_line(bool detect_dark);

  /**
   * @brief True if wrapper treats higher values as line hits
   */
  bool detects_dark_line() const;

  /**
   * @brief Calibrate threshold from two known readings
   *
   * Uses midpoint between light and dark values
   */
  void calibrate_midpoint(std::int32_t light_reading,
                          std::int32_t dark_reading);

private:
  pros::adi::AnalogIn sensor;
  std::int32_t threshold_value;
  bool dark_line;

  static std::int32_t clamp_raw(std::int32_t value);
};
