// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/path/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mclib {
namespace path {

namespace {

// Below this, a curvature is treated as straight. Matches pure_pursuit.cpp.
constexpr double kStraightCurvature = 1e-9;

bool positiveFinite(double value) { return std::isfinite(value) && value > 0.0; }

// Everything below runs on raw SI base values, as pure_pursuit.cpp does.
double speedCap(double curvature, const TrajectoryConstraints& c) {
  const double k = std::fabs(curvature);
  double cap = c.max_velocity.raw();
  if (k > kStraightCurvature && c.max_lateral_acceleration.raw() > 0.0) {
    cap = std::min(cap, std::sqrt(c.max_lateral_acceleration.raw() / k));
  }
  if (c.track_width.raw() > 0.0) {
    cap = std::min(cap, c.max_velocity.raw() / (1.0 + k * c.track_width.raw() / 2.0));
  }
  return cap;
}

}  // namespace

Trajectory Trajectory::generate(const Path& path,
                                const TrajectoryConstraints& constraints) {
  Trajectory out;
  const double accel = constraints.max_acceleration.raw();
  const double decel = constraints.max_deceleration.raw() > 0.0
                           ? constraints.max_deceleration.raw()
                           : accel;
  if (!path.valid() || !positiveFinite(constraints.max_velocity.raw()) ||
      !positiveFinite(accel) || !positiveFinite(decel) ||
      !std::isfinite(constraints.max_lateral_acceleration.raw()) ||
      !std::isfinite(constraints.track_width.raw()) ||
      !std::isfinite(constraints.start_velocity.raw()) ||
      !std::isfinite(constraints.end_velocity.raw())) {
    return out;
  }

  const double length = path.length().raw();
  double spacing = constraints.spacing.raw();
  if (!positiveFinite(spacing)) spacing = (0.5 * units::inch).raw();
  // At least one segment, and the last sample lands exactly on the end.
  const std::size_t segments =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(length / spacing)));
  const double step = length / static_cast<double>(segments);

  std::vector<double> s(segments + 1), v(segments + 1);
  std::vector<PathPoint> points(segments + 1);
  for (std::size_t i = 0; i <= segments; ++i) {
    s[i] = (i == segments) ? length : step * static_cast<double>(i);
    points[i] = path.atDistance(QLength::fromBase(s[i]));
    // Take the worst curvature within half a step either side, so a sharp
    // spot between two samples still slows the robot down.
    const double half = step / 2.0;
    const double worst =
        path.maxAbsCurvature(QLength::fromBase(std::max(0.0, s[i] - half)),
                             QLength::fromBase(std::min(length, s[i] + half)))
            .raw();
    v[i] = speedCap(worst, constraints);
  }

  v.front() = std::min(v.front(), std::fabs(constraints.start_velocity.raw()));
  v.back() = std::min(v.back(), std::fabs(constraints.end_velocity.raw()));
  for (std::size_t i = 1; i <= segments; ++i) {
    const double ds = s[i] - s[i - 1];
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2.0 * accel * ds));
  }
  for (std::size_t i = segments; i-- > 0;) {
    const double ds = s[i + 1] - s[i];
    v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * decel * ds));
  }

  const double sign = constraints.reversed ? -1.0 : 1.0;
  out.m_states.resize(segments + 1);
  double t = 0.0;
  for (std::size_t i = 0; i <= segments; ++i) {
    TrajectoryState& state = out.m_states[i];
    const PathPoint& p = points[i];
    double a = 0.0;
    if (i < segments) {
      const double ds = s[i + 1] - s[i];
      a = ds > 0.0 ? (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * ds) : 0.0;
    }
    state.time = QTime::fromBase(t);
    state.distance = QLength::fromBase(s[i]);
    state.x = p.x;
    state.y = p.y;
    state.heading = constraints.reversed
                        ? QAngle::fromBase(wrapAngle(p.heading.raw() + M_PI))
                        : p.heading;
    state.velocity = QVelocity::fromBase(sign * v[i]);
    state.acceleration = QAcceleration::fromBase(sign * a);
    state.curvature = units::QCurvature::fromBase(sign * p.curvature.raw());
    if (i < segments) {
      const double ds = s[i + 1] - s[i];
      const double sum = v[i] + v[i + 1];
      // Both ends at zero speed only happens on a zero-length step; the
      // limits above are strictly positive everywhere else.
      t += sum > 0.0 ? 2.0 * ds / sum : 0.0;
    }
  }
  return out;
}

QTime Trajectory::duration() const {
  return m_states.empty() ? QTime{} : m_states.back().time;
}

QLength Trajectory::length() const {
  return m_states.empty() ? QLength{} : m_states.back().distance;
}

TrajectoryState Trajectory::sample(QTime t) const {
  if (m_states.empty()) return TrajectoryState{};
  if (!(t > m_states.front().time)) return m_states.front();
  if (!(t < m_states.back().time)) return m_states.back();

  // Last state at or before t.
  const auto after = std::upper_bound(
      m_states.begin(), m_states.end(), t,
      [](QTime value, const TrajectoryState& s) { return value < s.time; });
  const TrajectoryState& a = *(after - 1);
  const TrajectoryState& b = *after;

  const double tau = (t - a.time).raw();
  const double ds_total = (b.distance - a.distance).raw();
  // Work in speed magnitudes, then put the sign back.
  const double sign = a.velocity.raw() < 0.0 || b.velocity.raw() < 0.0 ? -1.0 : 1.0;
  const double v0 = std::fabs(a.velocity.raw());
  const double acc = sign * a.acceleration.raw();
  const double ds = std::clamp(v0 * tau + 0.5 * acc * tau * tau, 0.0, ds_total);
  const double frac = ds_total > 0.0 ? ds / ds_total : 0.0;

  TrajectoryState out;
  out.time = t;
  out.distance = a.distance + QLength::fromBase(ds);
  out.x = a.x + (b.x - a.x) * frac;
  out.y = a.y + (b.y - a.y) * frac;
  out.heading = QAngle::fromBase(
      wrapAngle(a.heading.raw() + wrapAngle(b.heading.raw() - a.heading.raw()) * frac));
  out.velocity = QVelocity::fromBase(sign * std::max(0.0, v0 + acc * tau));
  out.acceleration = a.acceleration;
  out.curvature = a.curvature + (b.curvature - a.curvature) * frac;
  return out;
}

}  // namespace path
}  // namespace mclib
