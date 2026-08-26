// mclib
#pragma once

#include "mclib/snapshot/raycast.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace snapshot {

template <typename Odom>
SnapshotResult snapshot_setpose(Odom& odom,
                                const std::vector<DistanceSensorConfig>& sensors,
                                const SnapshotConfig& cfg,
                                float heading_deg,
                                float forward_tracker_in,
                                float sideways_tracker_in,
                                float guess_x_in,
                                float guess_y_in) {
  SnapshotResult result{};
  result.x_in = guess_x_in;
  result.y_in = guess_y_in;
  result.heading_deg = heading_deg;

  if (sensors.empty()) {
    return result;
  }

  float total_error = 0.0f;
  std::size_t accepted = 0u;

  for (const DistanceSensorConfig& sensor : sensors) {
    if (sensor.dev == nullptr) {
      continue;
    }
    if (sensor.use_confidence_gate &&
        sensor.dev->getConfidence() < sensor.min_confidence) {
      continue;
    }

    const std::int32_t distance_mm = sensor.dev->getDistanceMm();
    if (distance_mm <= 0) {
      continue;
    }

    const float measured_in = static_cast<float>(distance_mm) / 25.4f;
    const Vec2 sensor_origin{
        guess_x_in + sensor.x_right_in,
        guess_y_in + sensor.y_fwd_in,
    };
    const Vec2 direction = unit_from_heading_deg(heading_deg + sensor.rel_deg);
    const std::uint32_t mask =
        sensor.field_mask_override == 0u ? cfg.field_mask : sensor.field_mask_override;
    const std::vector<RayHit> hits =
        raycast_all(sensor_origin, direction, sensor.max_range_in, mask);

    if (hits.empty()) {
      continue;
    }

    const float expected_in = hits.front().t_in;
    const float error = measured_in - expected_in;
    total_error += error * error;
    ++accepted;
  }

  if (accepted == 0u) {
    return result;
  }

  result.chi2 = total_error / static_cast<float>(accepted);
  result.used_samples = accepted;
  result.success = result.chi2 <= cfg.max_chi2_per_sensor;

  if (result.success) {
    odom.set_position(result.x_in,
                      result.y_in,
                      result.heading_deg,
                      forward_tracker_in,
                      sideways_tracker_in);
  }

  return result;
}

}  // namespace snapshot
