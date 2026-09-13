// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "api.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mclib {
namespace device {

/// @brief How an AI Vision object was found.
enum class AiVisionType {
  Color,   ///< Matched one of the colour descriptors (id = colour id 1..7).
  Code,    ///< Matched a colour code (id = code id 1..5).
  Object,  ///< Recognised by the built-in game-element model (id = element id).
  Tag,     ///< An AprilTag (id = tag id).
};

/// @brief Which detectors to run. Combine with `|`.
enum class AiVisionMode : std::uint8_t {
  Tags = 1 << 0,
  Colors = 1 << 1,
  Objects = 1 << 2,
  ColorMerge = 1 << 4,  ///< Merge touching blobs of the same colour into one.
  All = Tags | Colors | Objects,
};

constexpr AiVisionMode operator|(AiVisionMode a, AiVisionMode b) {
  return static_cast<AiVisionMode>(static_cast<std::uint8_t>(a) |
                                   static_cast<std::uint8_t>(b));
}

/**
 * @brief One detection, copied out of the PROS union into plain fields.
 *
 * Pixels, top-left origin, 320 x 240 frame: +x right, +y down. `center_x`
 * and `center_y` are the middle of the bounding box; subtract 160 / 120 to
 * get "how far off centre".
 *
 * For tags PROS gives four corners instead of a box; the box here is the
 * corners' bounding rectangle. `score` is only meaningful for Object
 * (model confidence, 0..100) and `angle_deg` only for Color / Code.
 */
struct AiVisionObject {
  AiVisionType type;
  std::uint8_t id;
  std::int32_t left;
  std::int32_t top;
  std::int32_t width;
  std::int32_t height;
  std::int32_t center_x;
  std::int32_t center_y;
  std::int32_t score;
  double angle_deg;

  std::int32_t area() const { return width * height; }
};

/**
 * @brief Thin wrapper over pros::AIVision.
 *
 * The sensor runs whichever detectors are enabled and returns every hit in
 * one list. objects() copies that list out as plain structs so callers do
 * not need to know which union member is live for which type - reading the
 * wrong one is not an error in C++, just a wrong number.
 */
class AiVision {
public:
  explicit AiVision(std::uint8_t port);

  /// @brief Turn on the detectors in `mode`, leaving others as they are.
  std::int32_t enable(AiVisionMode mode);
  /// @brief Turn off the detectors in `mode`.
  std::int32_t disable(AiVisionMode mode);
  /// @brief Bitmask of currently enabled detectors (AiVisionMode values). PROS_ERR on failure.
  std::int32_t enabledMask() const;

  /// @brief Objects in the last frame. PROS_ERR on failure.
  std::int32_t count() const;

  /// @brief Every detection in the last frame. Empty on error.
  std::vector<AiVisionObject> objects() const;

  /// @brief Only detections of `type`.
  std::vector<AiVisionObject> objects(AiVisionType type) const;

  /// @brief Biggest detection of `type`, by bounding-box area.
  std::optional<AiVisionObject> largest(AiVisionType type) const;

  /// @brief Biggest detection of `type` with this id (colour id, tag id, ...).
  std::optional<AiVisionObject> largest(AiVisionType type,
                                        std::uint8_t id) const;

  /**
   * @brief Define colour descriptor `id` (1..7) as the RGB the sensor
   *        should look for.
   * @param hue_range         How far hue may drift, 1..40 degrees.
   * @param saturation_range  How far saturation may drift, 0.1..1.
   */
  std::int32_t setColor(std::uint8_t id, std::uint8_t red, std::uint8_t green,
                        std::uint8_t blue, float hue_range = 10.0f,
                        float saturation_range = 0.2f);

  /// @brief Which AprilTag family to look for. `override_existing` replaces
  ///        the family already stored on the sensor.
  std::int32_t setTagFamily(pros::AivisionTagFamily family,
                            bool override_existing = false);

  /// @brief Start auto white balance. Takes a moment; do it in initialize().
  std::int32_t startAutoWhiteBalance();

  /// @brief Model class name for an Object id, if the sensor knows it.
  std::optional<std::string> className(std::int32_t id) const;

  /// @brief Reset the sensor to defaults (clears colours, codes, modes).
  std::int32_t reset();

private:
  static std::optional<AiVisionObject> convert(const pros::AIVision::Object& raw);

  mutable pros::AIVision m_camera;
};

}  // namespace device
}  // namespace mclib
