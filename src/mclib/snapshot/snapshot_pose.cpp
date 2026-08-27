// mclib
#include "mclib/snapshot/solver.hpp"

#include "mclib/math.hpp"

#include <cmath>
#include <cstddef>

namespace snapshot {
namespace {

/// @brief Unit normal of a field segment, flipped to face into the ray.
Vec2 segment_normal_facing(const FieldSegment& seg, const Vec2& dir_unit) {
  const Vec2 along = sub(seg.b, seg.a);
  Vec2 n{along.y, -along.x};
  const float len = norm(n);
  if (len <= 0.0f) {
    return Vec2{0.0f, 0.0f};
  }
  n = mul(n, 1.0f / len);
  // The Jacobian only needs a consistent sign, but facing the normal into the
  // incoming ray keeps `n . d` negative and the sign of the step obvious.
  if (dot(n, dir_unit) > 0.0f) {
    n = mul(n, -1.0f);
  }
  return n;
}

bool inside_field(float x_in, float y_in, float margin_in) {
  return x_in >= -margin_in && x_in <= FIELD_SIZE_IN + margin_in &&
         y_in >= -margin_in && y_in <= FIELD_SIZE_IN + margin_in;
}

}  // namespace

Vec2 sensor_origin_field(const SensorGeometry& geometry,
                         float x_in,
                         float y_in,
                         float heading_deg) {
  // Robot frame is (right, forward); robotToField() rotates it by the compass
  // heading. mclib::rotate() would rotate the wrong way - see solver.hpp.
  const mclib::Vec2 offset = mclib::robotToField(
      mclib::Vec2{static_cast<double>(geometry.x_right_in),
                  static_cast<double>(geometry.y_fwd_in)},
      static_cast<double>(heading_deg) * mclib::kPi / 180.0);
  return Vec2{x_in + static_cast<float>(offset.x()),
              y_in + static_cast<float>(offset.y())};
}

Vec2 sensor_direction_field(const SensorGeometry& geometry, float heading_deg) {
  const mclib::Vec2 dir = mclib::headingVector(
      static_cast<double>(heading_deg + geometry.rel_deg) * mclib::kPi / 180.0);
  return Vec2{static_cast<float>(dir.x()), static_cast<float>(dir.y())};
}

RayPrediction predict_range(const SensorGeometry& geometry,
                            float x_in,
                            float y_in,
                            float heading_deg,
                            std::uint32_t default_mask) {
  RayPrediction prediction{};
  prediction.origin = sensor_origin_field(geometry, x_in, y_in, heading_deg);
  prediction.direction = sensor_direction_field(geometry, heading_deg);

  const std::uint32_t mask =
      geometry.field_mask == 0u ? default_mask : geometry.field_mask;
  const std::vector<RayHit> hits = raycast_all(
      prediction.origin, prediction.direction, geometry.max_range_in, mask);
  if (hits.empty()) {
    return prediction;
  }

  const RayHit& nearest = hits.front();
  prediction.valid = true;
  prediction.expected_in = nearest.t_in;
  prediction.segment_index = nearest.segment_index;
  prediction.normal = segment_normal_facing(
      TERMINAL_FIELD_SEGMENTS[nearest.segment_index], prediction.direction);
  return prediction;
}

SnapshotResult solve_snapshot(const std::vector<SensorMeasurement>& measurements,
                              const SnapshotConfig& cfg,
                              float guess_x_in,
                              float guess_y_in,
                              float heading_deg) {
  SnapshotResult result{};
  result.x_in = guess_x_in;
  result.y_in = guess_y_in;
  result.heading_deg = heading_deg;

  if (measurements.empty()) {
    result.reject = SnapshotReject::NO_SENSORS;
    return result;
  }

  // Prefilter: drop readings that disagree grossly with the guess. A sensor
  // staring at another robot reports a range the map cannot explain from
  // anywhere near here, and one of those in the fit drags the whole solve.
  std::vector<SensorMeasurement> kept;
  kept.reserve(measurements.size());
  float initial_error = 0.0f;
  for (const SensorMeasurement& m : measurements) {
    if (!(m.measured_in > 0.0f)) {
      continue;
    }
    // Past the sensor's ceiling the reading is "saw nothing", not a range.
    if (m.geometry.max_range_in > 0.0f &&
        m.measured_in > m.geometry.max_range_in) {
      continue;
    }
    const RayPrediction prediction = predict_range(
        m.geometry, guess_x_in, guess_y_in, heading_deg, cfg.field_mask);
    if (!prediction.valid) {
      continue;
    }
    const float residual = m.measured_in - prediction.expected_in;
    if (std::fabs(residual) > cfg.max_residual_in) {
      continue;
    }
    initial_error += residual * residual;
    kept.push_back(m);
  }

  result.used_samples = kept.size();
  if (kept.empty()) {
    result.reject = SnapshotReject::TOO_FEW_SENSORS;
    return result;
  }
  result.chi2_initial = initial_error / static_cast<float>(kept.size());
  result.chi2 = result.chi2_initial;

  const std::size_t min_sensors = cfg.min_sensors < 1u ? 1u : cfg.min_sensors;
  if (kept.size() < min_sensors) {
    result.reject = SnapshotReject::TOO_FEW_SENSORS;
    return result;
  }

  float x = guess_x_in;
  float y = guess_y_in;
  float chi2 = result.chi2_initial;
  bool degenerate = false;

  const std::uint32_t max_iterations =
      cfg.max_iterations == 0u ? 1u : cfg.max_iterations;
  for (std::uint32_t iter = 0u; iter < max_iterations; ++iter) {
    // Normal equations for the 2x2 Gauss-Newton step, plus LM damping.
    float jtj_xx = cfg.damping;
    float jtj_xy = 0.0f;
    float jtj_yy = cfg.damping;
    float jtr_x = 0.0f;
    float jtr_y = 0.0f;
    float error = 0.0f;
    std::size_t rows = 0u;

    for (const SensorMeasurement& m : kept) {
      const RayPrediction prediction =
          predict_range(m.geometry, x, y, heading_deg, cfg.field_mask);
      if (!prediction.valid) {
        continue;
      }
      const float ndotd = dot(prediction.normal, prediction.direction);
      if (std::fabs(ndotd) < 1e-4f) {
        continue;  // Grazing hit: the range is not differentiable here.
      }
      const float residual = m.measured_in - prediction.expected_in;
      // d(residual)/d(x, y) = n / (n . d). See the derivation in solver.hpp.
      const float jx = prediction.normal.x / ndotd;
      const float jy = prediction.normal.y / ndotd;

      jtj_xx += jx * jx;
      jtj_xy += jx * jy;
      jtj_yy += jy * jy;
      // Gauss-Newton minimises ||r + J d||^2, so the normal equations are
      // (J^T J) d = -J^T r. The negation lives here.
      jtr_x -= jx * residual;
      jtr_y -= jy * residual;
      error += residual * residual;
      ++rows;
    }

    if (rows == 0u) {
      degenerate = true;
      break;
    }

    chi2 = error / static_cast<float>(rows);
    result.iterations = iter + 1u;

    const float det = jtj_xx * jtj_yy - jtj_xy * jtj_xy;
    if (std::fabs(det) < 1e-9f) {
      degenerate = true;
      break;
    }

    const float dx = (jtj_yy * jtr_x - jtj_xy * jtr_y) / det;
    const float dy = (jtj_xx * jtr_y - jtj_xy * jtr_x) / det;
    if (!std::isfinite(dx) || !std::isfinite(dy)) {
      degenerate = true;
      break;
    }

    // Cap a single step so a bad linearisation cannot fling the estimate
    // across the field and land it on the wrong wall.
    float step_x = dx;
    float step_y = dy;
    const float step = std::sqrt(dx * dx + dy * dy);
    if (cfg.max_correction_in > 0.0f && step > cfg.max_correction_in) {
      const float scale = cfg.max_correction_in / step;
      step_x *= scale;
      step_y *= scale;
    }

    x += step_x;
    y += step_y;

    if (std::sqrt(step_x * step_x + step_y * step_y) < cfg.convergence_in) {
      break;
    }
  }

  // Final residuals at the solved position, so the gate scores what would
  // actually be written back rather than the last mid-iteration estimate.
  float final_error = 0.0f;
  std::size_t final_rows = 0u;
  for (const SensorMeasurement& m : kept) {
    const RayPrediction prediction =
        predict_range(m.geometry, x, y, heading_deg, cfg.field_mask);
    if (!prediction.valid) {
      continue;
    }
    const float residual = m.measured_in - prediction.expected_in;
    final_error += residual * residual;
    ++final_rows;
  }
  if (final_rows > 0u) {
    chi2 = final_error / static_cast<float>(final_rows);
  }
  // Every kept reading had a hit at the guess. If one has lost its hit at the
  // solved position, chi2 would be averaged over a smaller set and look better
  // than it is, so treat the missing row as a failed solve instead.
  const bool lost_a_row = final_rows != kept.size();

  const float move_x = x - guess_x_in;
  const float move_y = y - guess_y_in;
  const float correction = std::sqrt(move_x * move_x + move_y * move_y);
  result.chi2 = chi2;
  result.correction_in = correction;

  // The gate. A wrong correction mid-auton is worse than no correction, so
  // every one of these leaves the pose at the guess.
  if (degenerate || lost_a_row || !std::isfinite(x) || !std::isfinite(y)) {
    result.reject = SnapshotReject::DEGENERATE;
    return result;
  }
  if (chi2 > cfg.max_chi2_per_sensor) {
    result.reject = SnapshotReject::CHI2;
    return result;
  }
  if (cfg.max_correction_in > 0.0f && correction > cfg.max_correction_in) {
    result.reject = SnapshotReject::CORRECTION_TOO_LARGE;
    return result;
  }
  if (!inside_field(x, y, cfg.field_margin_in)) {
    result.reject = SnapshotReject::OFF_FIELD;
    return result;
  }

  result.x_in = x;
  result.y_in = y;
  result.success = true;
  result.reject = SnapshotReject::NONE;
  return result;
}

}  // namespace snapshot
