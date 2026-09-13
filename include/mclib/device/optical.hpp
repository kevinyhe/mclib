// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "api.h"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

/**
 * @brief Wrapper for the V5 Optical sensor.
 *
 * The sensor reports hue (0..360), saturation (0..1), brightness (0..1) and
 * proximity (0..255, bigger means closer). The plain getters hand back
 * whatever PROS returns, including PROS_ERR_F / PROS_ERR when the port is
 * empty. The optional-returning siblings turn those into nullopt so a
 * disconnected sensor cannot pass as "hue 2147483647, must be red".
 *
 * Hue and proximity are the two values you use for game-piece detection.
 * `isColor()` does that check in one call and handles the 360 -> 0 wrap that
 * makes "red" awkward to express as a plain range.
 */
class Optical {
public:
  struct Rgb {
    double red;
    double green;
    double blue;
    double brightness;
  };

  enum class Gesture {
    None,
    Up,
    Down,
    Left,
    Right,
  };

  explicit Optical(std::uint8_t port);

  /// @brief Hue in degrees [0, 360). PROS_ERR_F on failure.
  double getHue() const;
  /// @brief Saturation [0, 1]. PROS_ERR_F on failure.
  double getSaturation() const;
  /// @brief Brightness [0, 1]. PROS_ERR_F on failure.
  double getBrightness() const;
  /// @brief Proximity [0, 255]; larger is closer. PROS_ERR on failure.
  std::int32_t getProximity() const;
  /// @brief Raw RGB as PROS reports it (components 0..255 scaled doubles).
  pros::c::optical_rgb_s_t getRgb() const;

  /// @brief Typed sibling of getHue(): nullopt when the sensor is not reporting.
  std::optional<double> hue() const;
  /// @brief Typed sibling of getSaturation().
  std::optional<double> saturation() const;
  /// @brief Typed sibling of getBrightness().
  std::optional<double> brightness() const;
  /// @brief Typed sibling of getProximity().
  std::optional<std::int32_t> proximity() const;
  /// @brief Typed sibling of getRgb(). nullopt if any component came back as
  ///        PROS_ERR_F.
  std::optional<Rgb> rgb() const;

  /**
   * @brief True when something is close enough and its hue sits in
   *        [hue_min, hue_max].
   *
   * The range may wrap through 360: `isColor(340, 20)` matches red hues on
   * both sides of zero. If hue_min <= hue_max it is a normal inclusive range.
   *
   * `min_proximity` rejects far-away colour noise: the sensor reads the field
   * tiles and lighting when nothing is in front of it, and those readings have
   * a hue too. 0 disables the check.
   *
   * Returns false when either reading failed - an unplugged sensor should
   * never claim it sees a ring.
   */
  bool isColor(double hue_min, double hue_max,
               std::int32_t min_proximity = 0) const;

  /**
   * @brief Drive the white LED, 0 (off) to 100 (full). Turn it on for
   *        colour detection: without it the hue reading swings with room
   *        lighting.
   */
  std::int32_t setLedPwm(std::uint8_t pwm);
  std::int32_t getLedPwm() const;

  /**
   * @brief Integration time in milliseconds (3..712). Shorter is faster but
   *        noisier; PROS defaults to about 100 ms, which is slow for a ring
   *        passing an intake.
   */
  std::int32_t setIntegrationTimeMs(double ms);
  double getIntegrationTimeMs() const;

  /**
   * @brief Gesture detection. While it is enabled, hue/saturation/brightness
   *        are not available, so leave it off unless you need it.
   */
  std::int32_t enableGesture();
  std::int32_t disableGesture();
  /// @brief Last detected gesture, or None (also None when PROS reports an error).
  Gesture gesture() const;

private:
  mutable pros::Optical m_optical;
};

}  // namespace device
}  // namespace mclib
