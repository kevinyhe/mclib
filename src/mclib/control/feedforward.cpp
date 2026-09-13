// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mclib/control/feedforward.hpp"

#include "mclib/time.hpp"

#include <cmath>
#include <cstdio>

/**
 * @file feedforward.cpp
 * @brief The two least-squares fits and the profile follower.
 *
 * The model arithmetic itself is constexpr and lives in the header; what is
 * here is the parts with state or loops - identifying the gains from samples,
 * and driving a profile against a real clock.
 */

namespace mclib {
namespace control {

namespace {

/// @brief Below this a divisor is treated as zero. See profile.cpp.
constexpr double kEps = 1e-12;

/// @brief Longest telemetry column name this file builds, including the null.
constexpr std::size_t kNameBufferSize = 40;

}  // namespace

// ---------------------------------------------------------------------------
// Characterisation
// ---------------------------------------------------------------------------

VelocityFit fitVelocityGains(const VelocitySample* samples, std::size_t count,
                             units::QVelocity min_speed) {
  VelocityFit fit;
  if (samples == nullptr) {
    return fit;
  }
  const double floor_speed = std::fabs(min_speed.raw());

  double n = 0.0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;

  for (std::size_t i = 0; i < count; ++i) {
    const double velocity = samples[i].velocity.raw();
    // Fold the reverse half of the run onto the forward half. The model is
    // odd-symmetric - V(-v) = -V(v) - so multiplying both coordinates of a
    // reverse sample by -1 puts it on the same line as the forward ones, and
    // one fit covers both directions.
    const double direction = velocity > 0.0 ? 1.0 : (velocity < 0.0 ? -1.0 : 0.0);
    if (direction == 0.0) {
      continue;
    }
    const double x = velocity * direction;
    if (x < floor_speed) {
      continue;
    }
    const double y = samples[i].voltage.raw() * direction;

    n += 1.0;
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
    sum_yy += y * y;
  }

  fit.used = static_cast<std::size_t>(n);
  if (n < 2.0) {
    return fit;
  }

  const double denominator = n * sum_xx - sum_x * sum_x;
  if (std::fabs(denominator) <= kEps) {
    // Every surviving sample sat at the same speed. There is a point but no
    // line; refusing beats reporting an arbitrary slope.
    return fit;
  }
  const double numerator = n * sum_xy - sum_x * sum_y;
  const double slope = numerator / denominator;

  fit.kV = QVoltagePerVelocity::fromBase(slope);
  fit.kS = units::QVoltage::fromBase((sum_y - slope * sum_x) / n);

  const double y_spread = n * sum_yy - sum_y * sum_y;
  // A zero spread in y means every sample wanted the same voltage, so the fit
  // is exact by construction and r^2 is 1 rather than 0/0.
  double r_squared = y_spread > kEps ? (numerator * numerator) / (denominator * y_spread) : 1.0;
  if (r_squared < 0.0) {
    r_squared = 0.0;
  }
  if (r_squared > 1.0) {
    r_squared = 1.0;
  }
  fit.r_squared = r_squared;
  fit.valid = true;
  return fit;
}

AccelerationFit fitAccelerationGain(const AccelerationSample* samples, std::size_t count,
                                    const FeedforwardGains& known,
                                    units::QAcceleration min_acceleration) {
  AccelerationFit fit;
  if (samples == nullptr) {
    return fit;
  }
  const double floor_acceleration = std::fabs(min_acceleration.raw());
  const double ks = known.kS.raw();
  const double kv = known.kV.raw();

  double sum_ra = 0.0;
  double sum_aa = 0.0;
  std::size_t used = 0;

  for (std::size_t i = 0; i < count; ++i) {
    const double acceleration = samples[i].acceleration.raw();
    if (std::fabs(acceleration) < floor_acceleration) {
      continue;
    }
    const double velocity = samples[i].velocity.raw();
    // Exactly the rule SimpleMotorFeedforward::calculate() applies, including
    // the fallback to the sign of the acceleration at zero velocity. A fit
    // that inverted a slightly different model would push the mismatch into
    // kA, which is the one gain nobody can sanity-check by eye.
    double static_sign = velocity > 0.0 ? 1.0 : (velocity < 0.0 ? -1.0 : 0.0);
    if (static_sign == 0.0) {
      static_sign = acceleration > 0.0 ? 1.0 : -1.0;
    }
    // What the already-known half of the model cannot explain. The model says
    // this leftover is exactly kA * a, so the fit is a line through the
    // origin - an intercept here would quietly absorb an error in kS.
    const double residual = samples[i].voltage.raw() - ks * static_sign - kv * velocity;

    sum_ra += residual * acceleration;
    sum_aa += acceleration * acceleration;
    ++used;
  }

  fit.used = used;
  if (used == 0 || sum_aa <= kEps) {
    return fit;
  }
  fit.kA = QVoltagePerAcceleration::fromBase(sum_ra / sum_aa);
  fit.valid = true;
  return fit;
}

// ---------------------------------------------------------------------------
// Follower
// ---------------------------------------------------------------------------

ProfileFollower::ProfileFollower(const ProfileFollowerConfig& config)
    : m_feedforward(config.gains),
      m_pid(config.kp, config.ki, config.kd),
      m_max_voltage(units::abs(config.max_voltage)) {
  // Real rates, not raw per-call deltas. This is new code with no gains fitted
  // against the historical numerics, so there is nothing to preserve and every
  // reason for kd to mean volts per (inch per second).
  m_pid.setUseDt(true);
  // A profile ends when the profile ends. Arrival detection would latch during
  // the cruise phase of any motion whose tracking is good, and with
  // hold_output false that zeroes the output mid-motion.
  m_pid.setArrive(false);
  // Keeps ki alive: PID zeroes its integral whenever |error| is inside the
  // small tolerance, and a well-tracked profile lives there permanently.
  m_pid.setSmallBigErrorTolerance(units::QLength{}, units::QLength{});
  m_pid.setIntegralMax(units::abs(config.integral_max));
  m_pid.setHoldOutput(true);
}

void ProfileFollower::follow(const MotionProfile& profile, units::QTime start_time) {
  m_profile = profile;
  m_start_time = start_time;
  m_last_tick = start_time;
  m_running = true;
  m_first_tick = true;
  m_pid.reset();
}

void ProfileFollower::follow(const MotionProfile& profile) {
  follow(profile, time::now());
}

void ProfileFollower::reset() {
  m_profile = MotionProfile{};
  m_running = false;
  m_first_tick = true;
  m_start_time = units::QTime{};
  m_last_tick = units::QTime{};
  m_setpoint = ProfileState{};
  m_last_feedforward = units::QVoltage{};
  m_last_feedback = units::QVoltage{};
  m_last_output = units::QVoltage{};
  m_last_error = units::QLength{};
  m_pid.reset();
}

units::QTime ProfileFollower::elapsed(units::QTime now) const {
  const units::QTime since = now - m_start_time;
  return since.raw() > 0.0 ? since : units::QTime{};
}

bool ProfileFollower::isFinished(units::QTime now) const {
  return !m_running || m_profile.isFinished(elapsed(now));
}

bool ProfileFollower::isFinished() const { return isFinished(time::now()); }

units::QVoltage ProfileFollower::update(units::QLength measured_position, units::QTime now) {
  if (!m_running) {
    m_last_feedforward = units::QVoltage{};
    m_last_feedback = units::QVoltage{};
    m_last_output = units::QVoltage{};
    return m_last_output;
  }

  // The first tick has no previous one to measure against. Zero is handed to
  // the PID deliberately: it clamps a non-positive dt up to its own floor,
  // which is a better first derivative than a made-up interval.
  const units::QTime dt = m_first_tick ? units::QTime{} : now - m_last_tick;
  m_first_tick = false;
  m_last_tick = now;

  return calculate(m_profile.sample(elapsed(now)), measured_position, dt);
}

units::QVoltage ProfileFollower::update(units::QLength measured_position) {
  return update(measured_position, time::now());
}

units::QVoltage ProfileFollower::calculate(const ProfileState& setpoint,
                                           units::QLength measured_position, units::QTime dt) {
  m_setpoint = setpoint;
  m_last_error = setpoint.position - measured_position;
  m_last_feedforward = m_feedforward.calculate(setpoint.velocity, setpoint.acceleration);

  m_pid.setTarget(setpoint.position);
  m_last_feedback = m_pid.update(measured_position, dt);

  m_last_output =
      units::clamp(m_last_feedforward + m_last_feedback, -m_max_voltage, m_max_voltage);

  if (m_logger != nullptr) {
    m_logger->set(m_ch_setpoint_position, setpoint.position);
    m_logger->set(m_ch_setpoint_velocity, setpoint.velocity);
    m_logger->set(m_ch_measured_position, measured_position);
    m_logger->set(m_ch_error, m_last_error);
    m_logger->set(m_ch_feedforward, m_last_feedforward);
    m_logger->set(m_ch_feedback, m_last_feedback);
  }
  return m_last_output;
}

void ProfileFollower::attachTelemetry(telemetry::Logger* logger, const char* prefix) {
  m_logger = logger;
  if (logger == nullptr) {
    return;
  }
  const char* base = prefix != nullptr ? prefix : "profile";
  char name[kNameBufferSize];
  const auto column = [&](const char* suffix) -> const char* {
    std::snprintf(name, sizeof(name), "%s_%s", base, suffix);
    return name;
  };

  m_ch_setpoint_position = logger->addLength(column("sp_pos"));
  m_ch_setpoint_velocity = logger->addVelocity(column("sp_vel"));
  m_ch_measured_position = logger->addLength(column("pos"));
  m_ch_error = logger->addLength(column("err"));
  m_ch_feedforward = logger->addVoltage(column("ff"));
  m_ch_feedback = logger->addVoltage(column("fb"));
}

}  // namespace control
}  // namespace mclib
