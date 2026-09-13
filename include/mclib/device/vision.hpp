// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "api.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mclib {
namespace device {

/**
 * @brief One detected blob, in pixels, with the origin at the centre of the
 *        image. +x is right, +y is DOWN (image convention, not field).
 *
 * The frame is 316 x 212, so x runs about -158..158 and y about -106..106.
 * `x` and `y` are the centre of the bounding box.
 */
struct VisionObject {
  std::int32_t signature;  ///< Signature id 1..7 that matched.
  std::int32_t x;          ///< Centre x, pixels from image centre, right positive.
  std::int32_t y;          ///< Centre y, pixels from image centre, down positive.
  std::int32_t width;      ///< Bounding box width, pixels.
  std::int32_t height;     ///< Bounding box height, pixels.

  std::int32_t area() const { return width * height; }
};

/**
 * @brief Wrapper for the V5 Vision sensor (the older colour-blob camera).
 *
 * The sensor matches colours against up to 7 signatures you configure with
 * the Vision Utility and paste in via setSignature(). It reports objects
 * sorted by size, so "the biggest thing of colour N" is one call:
 * largest(N).
 *
 * The wrapper puts the sensor in centre-origin mode at construction so
 * object x/y read as "how far off centre", which is what a turn-to-target
 * loop wants. PROS defaults to top-left origin; do not change the zero
 * point through the raw pros object or the coordinates below shift.
 */
class Vision {
public:
  static constexpr std::int32_t kFrameWidth = VISION_FOV_WIDTH;
  static constexpr std::int32_t kFrameHeight = VISION_FOV_HEIGHT;

  explicit Vision(std::uint8_t port);

  /// @brief Number of objects in the last frame. PROS_ERR on failure.
  std::int32_t count() const;

  /**
   * @brief Largest object matching `signature_id` (1..7), or nullopt when
   *        there is none or the sensor is not responding.
   *
   * PROS marks "no object" and "bad port" the same way, with signature 255,
   * so the two are not distinguishable here. Either way there is nothing to
   * aim at.
   */
  std::optional<VisionObject> largest(std::uint8_t signature_id) const;

  /// @brief Largest object of any signature.
  std::optional<VisionObject> largest() const;

  /**
   * @brief Up to `max_objects` objects matching `signature_id`, biggest
   *        first. Empty on error.
   */
  std::vector<VisionObject> objects(std::uint8_t signature_id,
                                    std::size_t max_objects = 8) const;

  /**
   * @brief Store a colour signature in slot `id` (1..7). The seven numbers
   *        are exactly what the Vision Utility prints for a signature, in
   *        the same order, so they can be pasted straight in.
   * @param range  Detection tolerance, typically 1..11 from the utility.
   * @param type   0 for a normal colour signature; leave it.
   */
  std::int32_t setSignature(std::uint8_t id, std::int32_t u_min,
                            std::int32_t u_max, std::int32_t u_mean,
                            std::int32_t v_min, std::int32_t v_max,
                            std::int32_t v_mean, float range,
                            std::int32_t type = 0);

  /// @brief Store an already-built PROS signature struct in slot `id`.
  std::int32_t setSignature(std::uint8_t id, pros::vision_signature_s_t signature);

  /// @brief Read back the signature stored in slot `id`.
  pros::vision_signature_s_t getSignature(std::uint8_t id) const;

  /**
   * @brief Exposure 0..150. Fix it rather than leaving auto: auto exposure
   *        changes what "the same colour" looks like as the robot drives
   *        under different lights.
   */
  std::int32_t setExposure(std::uint8_t exposure);
  std::int32_t getExposure() const;

  /// @brief Fix the white balance to a colour (0xRRGGBB) or hand it back to auto.
  std::int32_t setWhiteBalance(std::uint32_t rgb);
  std::int32_t setAutoWhiteBalance(bool enable);

  /// @brief Sensor LED colour (0xRRGGBB), or clearLed() for the default.
  std::int32_t setLed(std::uint32_t rgb);
  std::int32_t clearLed();

private:
  static std::optional<VisionObject> convert(const pros::vision_object_s_t& raw);

  pros::Vision m_vision;
};

}  // namespace device
}  // namespace mclib
