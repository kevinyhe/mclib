// mclib
#pragma once

#include "mclib/device/distance.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace snapshot {

struct Vec2 {
  float x;
  float y;
};

struct FieldSegment {
  Vec2 a;
  Vec2 b;
  std::uint32_t mask;
};

struct RayHit {
  float t_in;
  std::size_t segment_index;
  Vec2 point;
};

enum class Quadrant {
  ANY,
  BL,
  BR,
  TL,
  TR,
};

struct DistanceSensorConfig {
  mclib::device::Distance* dev = nullptr;
  float x_right_in = 0.0f;
  float y_fwd_in = 0.0f;
  float rel_deg = 0.0f;
  std::uint32_t field_mask_override = 0u;
  bool use_confidence_gate = false;
  std::int32_t min_confidence = 0;
  float max_range_in = 144.0f;
};

struct SnapshotConfig {
  std::uint32_t field_mask = 0u;
  std::size_t candidates_per_sensor = 1u;
  std::uint32_t samples = 1u;
  std::uint32_t sample_delay_ms = 20u;
  float max_chi2_per_sensor = 9.0f;
  float quadrant_margin_in = 0.0f;
};

struct SnapshotResult {
  bool success = false;
  float x_in = 0.0f;
  float y_in = 0.0f;
  float heading_deg = 0.0f;
  float chi2 = 0.0f;
  std::size_t used_samples = 0u;
};

struct SnapshotPoseRuntime {
  void* user_data = nullptr;
  float (*get_heading_deg)(void* user_data) = nullptr;
  float (*get_guess_x_in)(void* user_data) = nullptr;
  float (*get_guess_y_in)(void* user_data) = nullptr;
  float (*get_forward_tracker_in)(void* user_data) = nullptr;
  float (*get_sideways_tracker_in)(void* user_data) = nullptr;
  void (*apply_pose)(void* user_data,
                     float x_in,
                     float y_in,
                     float heading_deg,
                     float forward_tracker_in,
                     float sideways_tracker_in) = nullptr;
};

}  // namespace snapshot
