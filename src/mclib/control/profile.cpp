// mclib

#include "mclib/control/profile.hpp"

#include <cmath>

/**
 * @file profile.cpp
 * @brief Profile generation. Raw doubles inside, typed quantities at the edge.
 *
 * The generators below work in a "travel frame": every distance, velocity and
 * acceleration is a plain SI double whose positive direction is the direction
 * of travel. That is what makes the algebra readable - there is exactly one
 * place, at the moment a segment is emitted, where the direction is multiplied
 * back in. Everything the caller sees is typed and signed in the caller's own
 * frame.
 */

namespace mclib {
namespace control {

namespace {

/**
 * @brief Below this, a duration or a limit is treated as zero.
 *
 * @details All quantities here are in SI base units, so 1e-12 is a picometre
 * or a picosecond - far below anything a drivetrain can express, and far above
 * the rounding noise of the square roots and divisions in the generators.
 */
constexpr double kEps = 1e-12;

/// @brief The state of @p segment at local time @p u seconds into it.
ProfileState evaluate(const profile_detail::Segment& segment, double u) {
  const double x0 = segment.start_position.raw();
  const double v0 = segment.start_velocity.raw();
  const double a0 = segment.start_acceleration.raw();
  const double j = segment.jerk.raw();
  ProfileState state;
  state.position = units::QLength::fromBase(x0 + v0 * u + 0.5 * a0 * u * u + j * u * u * u / 6.0);
  state.velocity = units::QVelocity::fromBase(v0 + a0 * u + 0.5 * j * u * u);
  state.acceleration = units::QAcceleration::fromBase(a0 + j * u);
  return state;
}

/// @brief The state at the very end of @p segment.
ProfileState endOf(const profile_detail::Segment& segment) {
  return evaluate(segment, segment.duration.raw());
}

/**
 * @brief Sanitised limits, in SI base units, all strictly positive.
 *
 * @details `ok` is false when the constraints cannot produce any motion at
 * all, which is the one case where a caller gets an empty profile back rather
 * than a shorter one.
 */
struct Limits {
  double v_max = 0.0;
  double acc = 0.0;
  double dec = 0.0;
  double jerk = 0.0;
  bool ok = false;
};

Limits sanitise(const ProfileConstraints& constraints) {
  Limits limits;
  limits.v_max = std::fabs(constraints.max_velocity.raw());
  limits.acc = std::fabs(constraints.max_acceleration.raw());
  limits.dec = std::fabs(constraints.effectiveDeceleration().raw());
  limits.jerk = std::fabs(constraints.max_jerk.raw());
  if (limits.dec <= kEps) {
    limits.dec = limits.acc;
  }
  limits.ok = limits.v_max > kEps && limits.acc > kEps && limits.dec > kEps;
  return limits;
}

/// @brief +1, -1, or 0 when both the distance and the initial velocity are zero.
double travelDirection(double distance, double initial_velocity) {
  if (distance > 0.0) {
    return 1.0;
  }
  if (distance < 0.0) {
    return -1.0;
  }
  if (initial_velocity > 0.0) {
    return 1.0;
  }
  if (initial_velocity < 0.0) {
    return -1.0;
  }
  return 0.0;
}

}  // namespace

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

units::QAcceleration ProfileConstraints::effectiveDeceleration() const {
  return units::abs(max_deceleration).raw() > kEps ? max_deceleration : max_acceleration;
}

bool ProfileConstraints::jerkLimited() const { return units::abs(max_jerk).raw() > kEps; }

const ProfileConstraints& DirectionalConstraints::select(
    units::QLength distance, units::QVelocity initial_velocity) const {
  // Same tiebreak travelDirection() uses. A zero-distance request while
  // already rolling is a "stop from where you are" motion, and it travels in
  // whatever direction it is already going - so it has to get that
  // direction's deceleration limit, not the forward one.
  return travelDirection(distance.raw(), initial_velocity.raw()) < 0.0 ? reverse : forward;
}

// ---------------------------------------------------------------------------
// Segment chaining
// ---------------------------------------------------------------------------

void MotionProfile::beginProfile(units::QVelocity initial_velocity) {
  m_segment_count = 0;
  m_duration = units::QTime{};
  m_initial_velocity = initial_velocity;
  m_final_velocity = initial_velocity;
  m_net_displacement = units::QLength{};
  m_peak_velocity = units::abs(initial_velocity);
}

void MotionProfile::appendSegment(units::QTime duration, units::QAcceleration start_acceleration,
                                  units::QJerk jerk, bool chain_acceleration) {
  if (m_segment_count >= profile_detail::kMaxSegments) {
    return;
  }
  if (!(duration.raw() > kEps)) {
    return;
  }

  profile_detail::Segment& segment = m_segments[m_segment_count];
  if (m_segment_count == 0) {
    segment.start_time = units::QTime{};
    segment.start_position = units::QLength{};
    segment.start_velocity = m_initial_velocity;
    // Nothing to chain from on the first segment, so a chained acceleration
    // starts at zero. Every S-curve here starts from rest with zero
    // acceleration, which is exactly that.
    segment.start_acceleration =
        chain_acceleration ? units::QAcceleration{} : start_acceleration;
  } else {
    const profile_detail::Segment& previous = m_segments[m_segment_count - 1];
    const ProfileState end = endOf(previous);
    segment.start_time = previous.start_time + previous.duration;
    segment.start_position = end.position;
    segment.start_velocity = end.velocity;
    segment.start_acceleration = chain_acceleration ? end.acceleration : start_acceleration;
  }
  segment.duration = duration;
  segment.jerk = jerk;
  ++m_segment_count;
}

void MotionProfile::finalise() {
  if (m_segment_count == 0) {
    m_duration = units::QTime{};
    m_net_displacement = units::QLength{};
    m_final_velocity = units::QVelocity{};
    m_peak_velocity = units::QVelocity{};
    return;
  }

  const profile_detail::Segment& last = m_segments[m_segment_count - 1];
  m_duration = last.start_time + last.duration;
  const ProfileState end = endOf(last);
  m_net_displacement = end.position;
  m_final_velocity = end.velocity;

  double peak = std::fabs(m_initial_velocity.raw());
  for (std::size_t i = 0; i < m_segment_count; ++i) {
    const profile_detail::Segment& segment = m_segments[i];
    peak = std::fmax(peak, std::fabs(segment.start_velocity.raw()));
    peak = std::fmax(peak, std::fabs(endOf(segment).velocity.raw()));
    // On a constant-jerk segment velocity is quadratic, so its extremum sits
    // wherever the acceleration crosses zero. The S-curve shapes built here
    // never do that mid-segment, but a caller-built profile could, and a peak
    // that under-reports is worse than one line of arithmetic.
    const double j = segment.jerk.raw();
    if (std::fabs(j) > kEps) {
      const double u = -segment.start_acceleration.raw() / j;
      if (u > 0.0 && u < segment.duration.raw()) {
        peak = std::fmax(peak, std::fabs(evaluate(segment, u).velocity.raw()));
      }
    }
  }
  m_peak_velocity = units::QVelocity::fromBase(peak);
}

// ---------------------------------------------------------------------------
// Trapezoid
// ---------------------------------------------------------------------------

MotionProfile MotionProfile::trapezoidal(units::QLength distance,
                                         const ProfileConstraints& constraints,
                                         units::QVelocity initial_velocity,
                                         units::QVelocity final_velocity) {
  MotionProfile profile;
  profile.m_requested_distance = distance;

  const Limits limits = sanitise(constraints);
  const double direction = travelDirection(distance.raw(), initial_velocity.raw());
  if (!limits.ok || direction == 0.0) {
    return profile;
  }
  profile.m_direction = direction;
  profile.beginProfile(initial_velocity);

  // Travel frame: distance is non-negative, and a negative v0 means "currently
  // moving away from the target".
  const double d = distance.raw() * direction;
  const double v0 = initial_velocity.raw() * direction;
  double vf = final_velocity.raw() * direction;
  if (vf < 0.0) {
    vf = 0.0;
  }
  if (vf > limits.v_max) {
    vf = limits.v_max;
  }

  // Braking from v0 to vf already needs more room than we have. There is no
  // profile that both respects the deceleration limit and stops on target, so
  // emit the honest one - brake at the limit and overshoot - and say so.
  const double stop_distance =
      v0 > vf ? (v0 * v0 - vf * vf) / (2.0 * limits.dec) : 0.0;
  if (stop_distance > d + kEps) {
    profile.m_overshoots = true;
    profile.appendSegment(units::QTime::fromBase((v0 - vf) / limits.dec),
                          units::QAcceleration::fromBase(-limits.dec * direction),
                          units::QJerk{}, false);
    profile.finalise();
    return profile;
  }

  // The mirror image: too short to reach the requested final velocity. Same
  // reasoning as the braking case above - accelerate at the limit, run past
  // the target, and say so.
  const double reach_distance =
      vf > v0 ? (vf * vf - v0 * v0) / (2.0 * limits.acc) : 0.0;
  if (reach_distance > d + kEps) {
    profile.m_overshoots = true;
    profile.appendSegment(units::QTime::fromBase((vf - v0) / limits.acc),
                          units::QAcceleration::fromBase(limits.acc * direction),
                          units::QJerk{}, false);
    profile.finalise();
    return profile;
  }

  // Peak velocity if there were no cruise phase at all: solve
  //   d = (v^2 - v0^2) / (2*acc) + (v^2 - vf^2) / (2*dec)
  // for v. Note (v^2 - v0^2) / (2*a) is the signed displacement of a constant
  // acceleration between two velocities whatever their signs, so this stays
  // correct when v0 points the wrong way.
  const double numerator =
      d + v0 * v0 / (2.0 * limits.acc) + vf * vf / (2.0 * limits.dec);
  const double denominator = 1.0 / (2.0 * limits.acc) + 1.0 / (2.0 * limits.dec);
  double v_peak = std::sqrt(std::fmax(numerator / denominator, 0.0));
  if (v_peak > limits.v_max) {
    v_peak = limits.v_max;
  }
  if (v_peak < vf) {
    v_peak = vf;
  }

  double ramp_up_rate = 0.0;
  double ramp_up_time = 0.0;
  if (std::fabs(v_peak - v0) > kEps) {
    // A peak below the current speed means the first phase is a deceleration,
    // which is the right limit to apply to it.
    ramp_up_rate = v_peak > v0 ? limits.acc : -limits.dec;
    ramp_up_time = (v_peak - v0) / ramp_up_rate;
  }
  const double ramp_up_distance =
      v0 * ramp_up_time + 0.5 * ramp_up_rate * ramp_up_time * ramp_up_time;

  const double ramp_down_time = v_peak - vf > kEps ? (v_peak - vf) / limits.dec : 0.0;
  const double ramp_down_distance =
      v_peak * ramp_down_time - 0.5 * limits.dec * ramp_down_time * ramp_down_time;

  double cruise_distance = d - ramp_up_distance - ramp_down_distance;
  if (cruise_distance < 0.0) {
    cruise_distance = 0.0;
  }
  const double cruise_time = v_peak > kEps ? cruise_distance / v_peak : 0.0;

  profile.appendSegment(units::QTime::fromBase(ramp_up_time),
                        units::QAcceleration::fromBase(ramp_up_rate * direction),
                        units::QJerk{}, false);
  profile.appendSegment(units::QTime::fromBase(cruise_time), units::QAcceleration{},
                        units::QJerk{}, false);
  profile.appendSegment(units::QTime::fromBase(ramp_down_time),
                        units::QAcceleration::fromBase(-limits.dec * direction),
                        units::QJerk{}, false);
  profile.finalise();
  return profile;
}

// ---------------------------------------------------------------------------
// S-curve
// ---------------------------------------------------------------------------

MotionProfile MotionProfile::sCurve(units::QLength distance,
                                    const ProfileConstraints& constraints,
                                    units::QVelocity initial_velocity,
                                    units::QVelocity final_velocity) {
  const Limits limits = sanitise(constraints);
  // See the header warning: jerk limiting is only applied to a rest-to-rest
  // move. Anything else is handed to the trapezoid rather than refused.
  if (limits.jerk <= kEps || std::fabs(initial_velocity.raw()) > kEps ||
      std::fabs(final_velocity.raw()) > kEps) {
    return trapezoidal(distance, constraints, initial_velocity, final_velocity);
  }

  MotionProfile profile;
  profile.m_requested_distance = distance;
  const double direction = travelDirection(distance.raw(), 0.0);
  if (!limits.ok || direction == 0.0) {
    return profile;
  }
  profile.m_direction = direction;
  profile.beginProfile(units::QVelocity{});

  const double d = distance.raw() * direction;
  const double jerk = limits.jerk;

  // Peak acceleration actually reached while ramping to speed `v` under an
  // acceleration limit `a`: the ramp is triangular in acceleration, and never
  // reaches `a`, once v < a^2/jerk.
  const auto peakAccel = [jerk](double v, double a) {
    return v >= a * a / jerk ? a : std::sqrt(v * jerk);
  };
  // Time to go from rest to `v`. The velocity curve is antisymmetric about its
  // midpoint, so the mean speed over the ramp is exactly v/2 whatever shape it
  // took - which is what makes the distance below a single multiplication.
  const auto rampTime = [jerk, &peakAccel](double v, double a) {
    const double a_peak = peakAccel(v, a);
    return a_peak > kEps ? a_peak / jerk + v / a_peak : 0.0;
  };
  const auto rampDistance = [&rampTime](double v, double a) { return 0.5 * v * rampTime(v, a); };

  double v_peak = limits.v_max;
  if (rampDistance(limits.v_max, limits.acc) + rampDistance(limits.v_max, limits.dec) > d) {
    // rampDistance is continuous and strictly increasing in v, so bisection is
    // both correct and unconditionally convergent. Sixty halvings of the
    // velocity range is far below double precision; there is no closed form
    // worth the case analysis it would cost.
    double low = 0.0;
    double high = limits.v_max;
    for (int i = 0; i < 60; ++i) {
      const double mid = 0.5 * (low + high);
      if (rampDistance(mid, limits.acc) + rampDistance(mid, limits.dec) > d) {
        high = mid;
      } else {
        low = mid;
      }
    }
    v_peak = 0.5 * (low + high);
  }

  double cruise_distance =
      d - rampDistance(v_peak, limits.acc) - rampDistance(v_peak, limits.dec);
  if (cruise_distance < 0.0) {
    cruise_distance = 0.0;
  }
  const double cruise_time = v_peak > kEps ? cruise_distance / v_peak : 0.0;

  // One ramp is three segments: jerk the acceleration up, hold it, jerk it
  // back to zero. `polarity` is +1 for the speed-up half and -1 for the
  // slow-down half; `direction` maps both into the caller's frame.
  const auto emitRamp = [&](double a_limit, double polarity) {
    const double a_peak = peakAccel(v_peak, a_limit);
    if (a_peak <= kEps) {
      return;
    }
    const double jerk_time = a_peak / jerk;
    const double hold_time = std::fmax(v_peak / a_peak - jerk_time, 0.0);
    profile.appendSegment(units::QTime::fromBase(jerk_time), units::QAcceleration{},
                          units::QJerk::fromBase(polarity * jerk * direction), true);
    profile.appendSegment(units::QTime::fromBase(hold_time),
                          units::QAcceleration::fromBase(polarity * a_peak * direction),
                          units::QJerk{}, false);
    profile.appendSegment(units::QTime::fromBase(jerk_time), units::QAcceleration{},
                          units::QJerk::fromBase(-polarity * jerk * direction), true);
  };

  emitRamp(limits.acc, 1.0);
  profile.appendSegment(units::QTime::fromBase(cruise_time), units::QAcceleration{},
                        units::QJerk{}, false);
  emitRamp(limits.dec, -1.0);
  profile.finalise();
  return profile;
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

MotionProfile MotionProfile::generate(units::QLength distance,
                                      const ProfileConstraints& constraints,
                                      units::QVelocity initial_velocity,
                                      units::QVelocity final_velocity) {
  return constraints.jerkLimited()
             ? sCurve(distance, constraints, initial_velocity, final_velocity)
             : trapezoidal(distance, constraints, initial_velocity, final_velocity);
}

MotionProfile MotionProfile::generate(units::QLength distance,
                                      const DirectionalConstraints& constraints,
                                      units::QVelocity initial_velocity,
                                      units::QVelocity final_velocity) {
  return generate(distance, constraints.select(distance, initial_velocity), initial_velocity,
                  final_velocity);
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

std::size_t MotionProfile::segmentAt(units::QTime t) const {
  for (std::size_t i = 0; i + 1 < m_segment_count; ++i) {
    if (t < m_segments[i].start_time + m_segments[i].duration) {
      return i;
    }
  }
  return m_segment_count == 0 ? 0 : m_segment_count - 1;
}

ProfileState MotionProfile::sample(units::QTime t) const {
  if (m_segment_count == 0) {
    return ProfileState{};
  }
  if (t.raw() <= 0.0) {
    const profile_detail::Segment& first = m_segments[0];
    return ProfileState{first.start_position, first.start_velocity, first.start_acceleration};
  }
  if (t >= m_duration) {
    // Zero acceleration past the end, not the last segment's: the motion is
    // over, and a follower that keeps sampling should stop being told to push.
    return ProfileState{m_net_displacement, m_final_velocity, units::QAcceleration{}};
  }
  const std::size_t index = segmentAt(t);
  return evaluate(m_segments[index], (t - m_segments[index].start_time).raw());
}

units::QTime MotionProfile::timeAtDistance(units::QLength travelled) const {
  if (m_segment_count == 0 || m_direction == 0.0) {
    return units::QTime{};
  }
  const double target = travelled.raw() * m_direction;
  const double total = m_net_displacement.raw() * m_direction;
  if (target <= 0.0) {
    return units::QTime{};
  }
  if (target >= total) {
    return m_duration;
  }

  double low = 0.0;
  double high = m_duration.raw();
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (low + high);
    if (sample(units::QTime::fromBase(mid)).position.raw() * m_direction > target) {
      high = mid;
    } else {
      low = mid;
    }
  }
  return units::QTime::fromBase(0.5 * (low + high));
}

units::QVelocity MotionProfile::velocityAtDistance(units::QLength travelled) const {
  return sample(timeAtDistance(travelled)).velocity;
}

}  // namespace control
}  // namespace mclib
