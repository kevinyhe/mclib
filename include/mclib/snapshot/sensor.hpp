// mclib
#pragma once

#include "mclib/device/distance.hpp"
#include "mclib/snapshot/types.hpp"

#include <cstdint>

/**
 * @file sensor.hpp
 * @brief The one snapshot header that names a real distance sensor.
 *
 * Everything else under `snapshot/` is pure geometry. Keeping the
 * `mclib::device::Distance*` here means `raycast.cpp` and the solver link on
 * the host, which is what makes the frame conventions testable.
 */

namespace snapshot {

/**
 * @brief A physical distance sensor: where it is bolted, and how to trust it.
 *
 * The mounting fields mirror `SensorGeometry` so existing setup code keeps
 * working; `geometry()` packages them for the solver.
 */
struct DistanceSensorConfig {
  mclib::device::Distance* dev = nullptr;
  float x_right_in = 0.0f;
  float y_fwd_in = 0.0f;
  float rel_deg = 0.0f;
  std::uint32_t field_mask_override = 0u;
  bool use_confidence_gate = false;
  std::int32_t min_confidence = 0;
  /// @copydoc SensorGeometry::max_range_in
  float max_range_in = 78.0f;

  /// @brief The mounting geometry alone, with no reference to the device.
  SensorGeometry geometry() const {
    SensorGeometry g{};
    g.x_right_in = x_right_in;
    g.y_fwd_in = y_fwd_in;
    g.rel_deg = rel_deg;
    g.field_mask = field_mask_override;
    g.max_range_in = max_range_in;
    return g;
  }
};

}  // namespace snapshot
