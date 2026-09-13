// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/snapshot/sensor.hpp"
#include "mclib/snapshot/solver.hpp"

// pros::delay. sensor.hpp already drags in api.h, but this file calls into
// PROS directly and should say so rather than rely on that.
#include "api.h"

#include <algorithm>
#include <cstdint>
#include <vector>

/**
 * @file snapshot.hpp
 * @brief Reads the distance sensors, solves for the pose, writes it back.
 *
 * This is the only layer that talks to hardware. The geometry and the solve
 * live in `solver.hpp` / `snapshot_pose.cpp` and know nothing about PROS.
 */

namespace snapshot {

namespace detail {

/// @brief Caps on the sampling loop. Together they bound one call to
///        `read_measurements()` at 16 * 100 ms, so a mistyped config costs a
///        second and a half rather than a minute of the autonomous period.
inline constexpr std::uint32_t kMaxSamples = 16u;
/// @brief @copydoc kMaxSamples
inline constexpr std::uint32_t kMaxSampleDelayMs = 100u;

/**
 * @brief One reading in inches, or a negative value when unusable.
 *
 * Goes through `Distance::distance()`, not `getDistanceMm()`. A port with no
 * distance sensor on it makes PROS return PROS_ERR (INT32_MAX), which
 * `getDistanceMm()` hands straight back as 84.5 million inches - a number no
 * range check catches - and `getConfidence()` fails the same way, so the
 * confidence gate does not save you either. The optional accessor is the one
 * that reports the bad port. See the note on `Distance::distance()`.
 */
inline float read_once_in(const DistanceSensorConfig& sensor) {
  if (sensor.dev == nullptr) {
    return -1.0f;
  }
  const std::optional<mclib::units::QLength> measured = sensor.dev->distance();
  if (!measured.has_value()) {
    return -1.0f;  // Bad port. Confidence would be PROS_ERR too.
  }
  if (sensor.use_confidence_gate &&
      sensor.dev->getConfidence() < sensor.min_confidence) {
    return -1.0f;
  }
  const float inches = static_cast<float>(measured->convert(mclib::units::inch));
  if (inches <= 0.0f) {
    return -1.0f;
  }
  // The sensor reports 9999 mm when nothing is in range. That is not an error,
  // but it is not a range either, and letting it into the fit as a 393 in
  // measurement biases every residual. max_range_in is the sensor's real
  // ceiling; anything past it is "saw nothing".
  if (sensor.max_range_in > 0.0f && inches > sensor.max_range_in) {
    return -1.0f;
  }
  return inches;
}

}  // namespace detail

/**
 * @brief Collect one median-filtered reading per sensor.
 *
 * A distance sensor occasionally returns a single wild value - a reflection off
 * the edge of a field element, or a frame of noise at long range. The median
 * throws those away where a mean would let them through. Readings that fail the
 * confidence gate or come back non-positive are not counted, and a sensor that
 * never produces a usable reading is left out of the result entirely.
 *
 * The sample loop is outside the sensor loop, so all the sensors share one
 * delay per round. Total block time is `(samples - 1) * sample_delay_ms`
 * regardless of how many sensors there are - 140 ms at the defaults - instead
 * of that much per sensor. Call it between motions, not inside a control loop.
 */
inline std::vector<SensorMeasurement> read_measurements(
    const std::vector<DistanceSensorConfig>& sensors, const SnapshotConfig& cfg) {
  std::vector<SensorMeasurement> measurements;
  if (sensors.empty()) {
    return measurements;
  }

  std::uint32_t samples = cfg.samples < 1u ? 1u : cfg.samples;
  if (samples > detail::kMaxSamples) {
    samples = detail::kMaxSamples;
  }
  const std::uint32_t sample_delay_ms =
      cfg.sample_delay_ms > detail::kMaxSampleDelayMs ? detail::kMaxSampleDelayMs
                                                      : cfg.sample_delay_ms;

  std::vector<float> readings(sensors.size() * samples, -1.0f);
  std::vector<std::uint32_t> counts(sensors.size(), 0u);

  for (std::uint32_t round = 0u; round < samples; ++round) {
    if (round > 0u && sample_delay_ms > 0u) {
      pros::delay(sample_delay_ms);
    }
    for (std::size_t i = 0u; i < sensors.size(); ++i) {
      const float value = detail::read_once_in(sensors[i]);
      if (value > 0.0f) {
        readings[i * samples + counts[i]] = value;
        ++counts[i];
      }
    }
  }

  measurements.reserve(sensors.size());
  for (std::size_t i = 0u; i < sensors.size(); ++i) {
    if (counts[i] == 0u) {
      continue;
    }
    float* const first = readings.data() + i * samples;
    std::sort(first, first + counts[i]);
    measurements.push_back(
        SensorMeasurement{sensors[i].geometry(), first[counts[i] / 2u]});
  }
  return measurements;
}

/**
 * @brief Read the sensors, solve for the position, and correct `odom` if the
 *        gate accepts the answer.
 *
 * The heading is taken as given and never corrected - see `solver.hpp`. On a
 * refusal the pose is left exactly as it was and `result.reject` says why.
 *
 * @param odom Anything with
 *        `set_position(x_in, y_in, heading_deg, forward_tracker_in, sideways_tracker_in)`.
 */
template <typename Odom>
SnapshotResult snapshot_setpose(Odom& odom,
                                const std::vector<DistanceSensorConfig>& sensors,
                                const SnapshotConfig& cfg,
                                float heading_deg,
                                float forward_tracker_in,
                                float sideways_tracker_in,
                                float guess_x_in,
                                float guess_y_in) {
  if (sensors.empty()) {
    SnapshotResult result{};
    result.x_in = guess_x_in;
    result.y_in = guess_y_in;
    result.heading_deg = heading_deg;
    result.reject = SnapshotReject::NO_SENSORS;
    return result;
  }

  const std::vector<SensorMeasurement> measurements =
      read_measurements(sensors, cfg);
  SnapshotResult result =
      solve_snapshot(measurements, cfg, guess_x_in, guess_y_in, heading_deg);
  if (measurements.empty()) {
    // Sensors are configured; none of them produced a reading. That is a
    // hardware problem, not an empty sensor list.
    result.reject = SnapshotReject::TOO_FEW_SENSORS;
  }

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
