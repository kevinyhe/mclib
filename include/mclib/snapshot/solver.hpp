// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/snapshot/raycast.hpp"

#include <vector>

/**
 * @file solver.hpp
 * @brief Turns distance readings into a corrected field position.
 *
 * ## What this does
 *
 * Given a guessed pose and a set of range readings, it finds the position that
 * best explains the readings by ray-casting against the static field map. The
 * heading is an input, not an unknown: the IMU is far more trustworthy than a
 * heading inferred from two range readings, and with only two sensors letting
 * the heading float makes the problem underdetermined. The solve is over
 * `(x, y)` only.
 *
 * ## Frame
 *
 * The compass frame from `mclib/math.hpp`: theta = 0 along +Y, clockwise
 * positive, heading vector `(sin, cos)`. Sensor mounting offsets are in the
 * robot frame (`x_right_in` right, `y_fwd_in` forward) and are rotated into the
 * field by `mclib::robotToField()`. Do NOT use `mclib::rotate()` here - it is a
 * standard-frame primitive and on a compass heading it silently performs the
 * inverse rotation.
 *
 * ## Method
 *
 * Gauss-Newton with an analytic Jacobian, Levenberg-Marquardt damping, and a
 * fresh ray-cast per iteration.
 *
 * For a ray from `o` along unit `d` hitting a wall through point `a` with unit
 * normal `n`, the range satisfies `n . (o + t d) = n . a`, so
 * `t = n . (a - o) / (n . d)` and `dt/do = -n / (n . d)`. The residual is
 * `r = measured - t`, so `dr/d(x, y) = n / (n . d)` - exact, closed form, and
 * free once the ray-cast has told us which segment was hit. No finite
 * differences, no extra casts per parameter.
 *
 * Because the map is piecewise linear, the model is exact as long as the same
 * segments stay hit, so the solve typically converges in two iterations. Cost
 * is `iterations * sensors` ray-casts, each a loop over the 12 field segments:
 * a few hundred float operations total, microseconds on a V5 brain. The
 * expensive part of a snapshot is reading the sensors, not solving.
 *
 * ## Why not a multi-candidate search
 *
 * `candidates_per_sensor` in the old config hinted at enumerating every
 * segment each sensor could have hit and scoring the combinations. That is
 * `segments^sensors` scored poses for the same answer, and it still needs a
 * refinement step at the end. Gauss-Newton from the odometry guess is cheaper
 * and the guess is already good - a snapshot corrects drift, it does not
 * recover a lost robot. The field is small enough that the wrong-wall failure
 * mode is caught by the gate rather than searched around.
 */

namespace snapshot {

/**
 * @brief One sensor's predicted range at a candidate pose.
 */
struct RayPrediction {
  bool valid = false;         ///< False when the ray hits nothing in range.
  float expected_in = 0.0f;   ///< Predicted range along the ray, inches.
  Vec2 origin{0.0f, 0.0f};    ///< Ray start in field coordinates, inches.
  Vec2 direction{0.0f, 0.0f}; ///< Unit ray direction in field coordinates.
  Vec2 normal{0.0f, 0.0f};    ///< Unit normal of the hit segment, facing the ray.
  std::size_t segment_index = 0u;
};

/**
 * @brief Where a sensor's ray starts, in field coordinates.
 *
 * Rotates the robot-frame mounting offset by the robot heading, then
 * translates. This is the step the original code skipped: it added
 * `x_right_in` straight onto the field X, so a sensor 7 in to the robot's
 * right was treated as 7 in field-east no matter which way the robot faced.
 */
Vec2 sensor_origin_field(const SensorGeometry& geometry,
                         float x_in,
                         float y_in,
                         float heading_deg);

/**
 * @brief Unit vector the sensor looks along, in field coordinates.
 * @return `(sin, cos)` of `heading_deg + geometry.rel_deg`.
 */
Vec2 sensor_direction_field(const SensorGeometry& geometry, float heading_deg);

/**
 * @brief Ray-cast one sensor from a candidate pose.
 * @param default_mask Used when `geometry.field_mask` is 0.
 * @return The nearest hit, with the surface normal needed by the Jacobian.
 */
RayPrediction predict_range(const SensorGeometry& geometry,
                            float x_in,
                            float y_in,
                            float heading_deg,
                            std::uint32_t default_mask);

/**
 * @brief Solve for the position that best explains the readings.
 *
 * Runs the prefilters, the Gauss-Newton solve, and the accept/reject gate. It
 * does not touch any odometry - the caller applies `result` only when
 * `result.success` is true.
 *
 * A refusal fills in `result.reject` and leaves `result.x_in` / `result.y_in`
 * at the guess, so writing the result back unconditionally is a no-op rather
 * than a corruption. Refusals happen when: fewer than `min_sensors` readings
 * survive the prefilters (`TOO_FEW_SENSORS`), the walls seen leave the
 * position unobservable (`DEGENERATE`), the fit still disagrees with the
 * readings (`CHI2`), the fit wants to move further than `max_correction_in`
 * (`CORRECTION_TOO_LARGE`), or the answer is off the field (`OFF_FIELD`).
 *
 * @param measurements Sensors that reported a usable range this snapshot.
 * @param cfg          Gate thresholds and iteration limits.
 * @param guess_x_in   Odometry's current field X, inches.
 * @param guess_y_in   Odometry's current field Y, inches.
 * @param heading_deg  Robot heading, compass degrees. Held fixed.
 */
SnapshotResult solve_snapshot(const std::vector<SensorMeasurement>& measurements,
                              const SnapshotConfig& cfg,
                              float guess_x_in,
                              float guess_y_in,
                              float heading_deg);

}  // namespace snapshot
