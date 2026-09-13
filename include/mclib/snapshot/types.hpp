// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @file types.hpp
 * @brief Plain data shared by the snapshot ray-caster and the pose solver.
 *
 * Nothing in this header touches PROS. The one PROS-dependent type,
 * `DistanceSensorConfig`, lives in `snapshot/sensor.hpp`, so the geometry
 * (`collision_map.hpp`, `raycast.hpp`, `solver.hpp`) builds and is testable on
 * the host.
 */

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

/**
 * @brief Where a distance sensor sits on the robot and what it may see.
 *
 * The offsets are in the robot frame: `x_right_in` is inches to the robot's
 * right, `y_fwd_in` is inches forward, `rel_deg` is the facing relative to
 * robot forward (0 = forward, +90 = right, 180 = back, -90 = left). They are
 * rotated into the field frame by the robot heading before ray-casting - see
 * `sensor_origin_field()` in `solver.hpp`.
 */
struct SensorGeometry {
  float x_right_in = 0.0f;
  float y_fwd_in = 0.0f;
  float rel_deg = 0.0f;
  /// Bitmask of map objects this sensor may hit. 0 falls back to
  /// `SnapshotConfig::field_mask`.
  std::uint32_t field_mask = 0u;
  /// How far this sensor can actually measure, inches. It both truncates the
  /// predicted range and rejects a measurement past it.
  ///
  /// The default is the V5 distance sensor's usable ceiling, not the field
  /// diagonal. Past about 2000 mm the sensor stops reporting a range and starts
  /// reporting 9999 mm, so a larger value lets the model predict distances the
  /// hardware cannot measure and quietly biases every residual against a
  /// saturated reading.
  float max_range_in = 78.0f;
};

/// @brief One sensor's geometry paired with the range it actually reported.
struct SensorMeasurement {
  SensorGeometry geometry{};
  float measured_in = 0.0f;
};

/**
 * @brief Knobs for the snapshot solve and its accept/reject gate.
 *
 * Distances are inches, chi-squared is inches squared.
 */
struct SnapshotConfig {
  /// Map objects any sensor may hit unless it overrides the mask itself.
  /// 0 allows nothing, so every reading is dropped - set it to at least
  /// `MAP_PERIMETER`. `snapshot_config_reset_defaults()` does.
  std::uint32_t field_mask = 0u;
  /// Readings taken per sensor per snapshot; the median is used. 0 and 1 both
  /// mean a single read.
  std::uint32_t samples = 1u;
  /// Milliseconds between the reads of one sensor when `samples > 1`.
  std::uint32_t sample_delay_ms = 20u;
  /// Sensors that must survive the prefilters before a solve is attempted.
  /// Two is the minimum that makes (x, y) observable.
  std::size_t min_sensors = 2u;
  /// Gauss-Newton iteration cap.
  std::uint32_t max_iterations = 6u;
  /// Stop iterating once a step moves the estimate less than this.
  float convergence_in = 0.01f;
  /// Gross-outlier prefilter: a reading whose residual against the *guess*
  /// exceeds this is dropped before the solve. Sized for "the sensor is
  /// looking at a robot, not the field", not for normal odometry drift.
  float max_residual_in = 24.0f;
  /// Post-fit gate: mean squared residual per sensor, inches squared.
  float max_chi2_per_sensor = 9.0f;
  /// Post-fit gate: refuse a correction that moves the pose further than this,
  /// and cap any single Gauss-Newton step at the same length. 0 disables both.
  float max_correction_in = 12.0f;
  /// Levenberg-Marquardt damping on the 2x2 normal equations. Keeps the
  /// unobservable direction pinned to the guess when the walls a snapshot sees
  /// are close to parallel.
  float damping = 1e-3f;
  /// Post-fit gate: how far outside the field the solved position may land.
  float field_margin_in = 6.0f;
  /// Quadrant bias applied to the guess by `snapshot_setpose_quadrant()`.
  float quadrant_margin_in = 0.0f;
};

/// @brief Why a snapshot did not correct the pose.
enum class SnapshotReject {
  NONE,                  ///< The correction was accepted and applied.
  NO_SENSORS,            ///< No sensor produced a reading to solve with.
  TOO_FEW_SENSORS,       ///< Fewer usable readings than `min_sensors`.
  DEGENERATE,            ///< The walls seen do not pin down a position.
  CHI2,                  ///< Post-fit residuals too large to trust.
  CORRECTION_TOO_LARGE,  ///< The solve wanted to teleport the robot.
  OFF_FIELD,             ///< The solved position is off the field.
  NO_RUNTIME,            ///< No runtime vtable installed.
};

struct SnapshotResult {
  bool success = false;
  float x_in = 0.0f;
  float y_in = 0.0f;
  float heading_deg = 0.0f;
  /// Mean squared residual per sensor after the solve, inches squared.
  float chi2 = 0.0f;
  /// Mean squared residual per sensor at the guess, before the solve.
  float chi2_initial = 0.0f;
  /// Distance from the guess to the solved position, inches.
  float correction_in = 0.0f;
  /// Sensors that survived the prefilters and fed the solve.
  std::size_t used_samples = 0u;
  std::uint32_t iterations = 0u;
  SnapshotReject reject = SnapshotReject::NO_SENSORS;
};

/**
 * @brief C function-pointer seam between the solver and whatever owns the pose.
 *
 * The solver never names an odometry type. Install one of these and
 * `snapshot_setpose_quadrant()` reads the guess through it and writes the
 * correction back through it.
 */
struct SnapshotPoseRuntime {
  void* user_data = nullptr;
  /**
   * @brief Read x, y and heading in one shot. Preferred when the pose source
   *        has a consistent read.
   *
   * The three scalar getters below are three separate reads, so they can pair
   * an x from one odometry tick with a y from the next - exactly the tearing
   * `RobotState::pose()` exists to prevent. When this is non-null it is used
   * instead of them.
   */
  void (*get_pose)(void* user_data,
                   float* x_in,
                   float* y_in,
                   float* heading_deg) = nullptr;
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
