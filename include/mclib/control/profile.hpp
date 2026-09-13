// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/units.hpp"

#include <cstddef>

/**
 * @file profile.hpp
 * @brief Time-parameterised velocity profiles: trapezoidal and S-curve.
 *
 * A profile answers one question: "at time t into this motion, where should
 * the robot be, how fast should it be going, and how hard should it be
 * accelerating?" Nothing here reads a sensor, touches a motor, or includes a
 * PROS header - it is pure arithmetic over `mclib::units` quantities, so every
 * number in it is checked by `tests/profile_test.cpp` on the host.
 *
 * The output feeds `mclib::control::ProfileFollower` in `feedforward.hpp`,
 * which turns a ProfileState into a voltage. A path follower can also ask for
 * a velocity at a given distance along the path with velocityAtDistance().
 *
 * @code
 * using namespace mclib::control;
 * ProfileConstraints limits{
 *     .max_velocity = 48 * mclib::units::inps,
 *     .max_acceleration = 96 * inps2,
 * };
 * const MotionProfile profile = MotionProfile::generate(48_in, limits);
 * // profile.duration() == 1.5 s, profile.netDisplacement() == 48 in
 * const ProfileState at_half = profile.sample(750_ms);
 * @endcode
 *
 * Internally a profile is a short list of constant-jerk segments. A trapezoid
 * is three segments with zero jerk; an S-curve is up to seven with a non-zero
 * jerk on four of them. Sampling evaluates one cubic, so it is cheap enough to
 * call on every 10 ms control tick.
 */

namespace mclib {
namespace control {

/// @brief One inch per second squared, the unit VEX drivetrain accel is tuned in.
inline constexpr units::QAcceleration inps2 = units::inps / units::second;

/// @brief One inch per second cubed, the unit S-curve jerk is tuned in.
inline constexpr units::QJerk inps3 = inps2 / units::second;

/**
 * @brief A single point on a profile: where to be, how fast, how hard.
 *
 * @details Position is measured along the direction of travel from wherever
 * the motion started, so it is signed: a profile for -24 inches ends at
 * -24 inches. Velocity and acceleration carry the same sign convention.
 */
struct ProfileState {
  /// @brief Displacement from the start of the motion.
  units::QLength position{};
  /// @brief Commanded velocity at this instant.
  units::QVelocity velocity{};
  /// @brief Commanded acceleration at this instant.
  units::QAcceleration acceleration{};
};

/**
 * @brief The limits a profile is generated against.
 *
 * @details All four are magnitudes; the generator applies whichever sign the
 * direction of travel calls for. Two of them have a "0 means something else"
 * convention, which is what keeps the common case a two-field aggregate:
 *
 * - `max_deceleration` of 0 means "same as max_acceleration", i.e. symmetric.
 * - `max_jerk` of 0 means "unbounded", i.e. a trapezoid rather than an
 *   S-curve. `MotionProfile::generate()` reads this to pick a shape.
 *
 * This drivetrain has separate forward and reverse accel/decel constants
 * (`max_slew_accel_fwd` and friends in config.cpp). Build one
 * ProfileConstraints per direction and hand generate() the right one, or use
 * DirectionalConstraints below to let it choose.
 */
struct ProfileConstraints {
  /// @brief Cruise speed cap. Must be > 0 or the profile comes out empty.
  units::QVelocity max_velocity = 48 * units::inps;
  /// @brief Speeding-up limit. Must be > 0 or the profile comes out empty.
  units::QAcceleration max_acceleration = 96 * inps2;
  /// @brief Slowing-down limit. 0 means "same as max_acceleration".
  units::QAcceleration max_deceleration = units::QAcceleration{0.0};
  /// @brief Rate-of-change-of-acceleration limit. 0 means unbounded.
  units::QJerk max_jerk = units::QJerk{0.0};

  /// @brief The deceleration actually used: max_deceleration, or accel when 0.
  units::QAcceleration effectiveDeceleration() const;

  /// @brief Whether these limits describe an S-curve (max_jerk > 0).
  bool jerkLimited() const;
};

/**
 * @brief A forward set and a reverse set of limits, chosen by travel direction.
 *
 * @details Mirrors the shape of the existing slew constants, which are already
 * split four ways. `select()` picks by the sign of the distance so a caller
 * does not have to.
 */
struct DirectionalConstraints {
  /// @brief Limits applied when the commanded distance is positive.
  ProfileConstraints forward{};
  /// @brief Limits applied when the commanded distance is negative.
  ProfileConstraints reverse{};

  /**
   * @brief The half matching the direction the motion will actually travel in.
   *
   * @details `forward` unless the motion goes backwards. @p initial_velocity
   * is the tiebreak for a zero-distance request - "stop from where you are"
   * travels in whatever direction it is already rolling, and must get that
   * direction's deceleration limit.
   */
  const ProfileConstraints& select(units::QLength distance,
                                   units::QVelocity initial_velocity = units::QVelocity{}) const;
};

namespace profile_detail {

/**
 * @brief One piece of a profile: constant jerk over a fixed duration.
 *
 * @details Sampling a segment at local time `u` is
 * `x = x0 + v0*u + a0*u^2/2 + j*u^3/6`, with the derivatives falling out of
 * the same polynomial. Storing the profile this way means a trapezoid and an
 * S-curve share one sampler; the only difference is whether `jerk` is zero.
 *
 * Every field is already signed for the direction of travel, so the sampler
 * needs no direction handling at all.
 */
struct Segment {
  /// @brief Profile time at which this segment begins.
  units::QTime start_time{};
  /// @brief How long this segment lasts.
  units::QTime duration{};
  /// @brief Displacement at the start of the segment.
  units::QLength start_position{};
  /// @brief Velocity at the start of the segment.
  units::QVelocity start_velocity{};
  /// @brief Acceleration at the start of the segment.
  units::QAcceleration start_acceleration{};
  /// @brief Constant jerk over the segment. Zero for a trapezoid.
  units::QJerk jerk{};
};

/**
 * @brief Cap on segments per profile.
 *
 * @details Seven is the worst case (a jerk-limited S-curve with a cruise
 * phase); eight leaves a slot spare and keeps a MotionProfile a fixed few
 * hundred bytes with no allocation, which is what makes it safe to hold by
 * value in a follower on the V5 brain.
 */
inline constexpr std::size_t kMaxSegments = 8;

}  // namespace profile_detail

/**
 * @brief A velocity profile over a fixed distance, sampled by time.
 *
 * @details Build one with generate(), trapezoidal() or sCurve(), then call
 * sample() with the time since the motion started. Copyable, no allocation,
 * no clock of its own - a profile is a pure function of time and knows nothing
 * about when it started.
 *
 * **Sign.** A profile is generated in a travel frame whose positive direction
 * is the sign of the commanded distance, then mapped back. Everything sample()
 * returns is in the caller's frame, so a -24 inch profile has negative
 * velocity throughout.
 *
 * **Degenerate cases, all of which are defined behaviour:**
 * - Zero distance with zero initial velocity: an empty profile. duration() is
 *   0 and sample() returns all zeros at any time.
 * - Negative distance: a normal profile running the other way.
 * - Non-zero initial velocity: the first segment accelerates or decelerates
 *   from it. An initial velocity pointing away from the target is handled by
 *   accelerating through zero at `max_acceleration`.
 * - Initial velocity too high to stop in the distance given: the profile
 *   becomes a single deceleration segment at `max_deceleration` and
 *   overshoots. overshoots() reports this, and netDisplacement() is then
 *   larger in magnitude than requestedDistance(). Refusing to build anything
 *   would be worse - the robot is moving either way and the best available
 *   command is "brake at the limit".
 * - Non-positive `max_velocity` or `max_acceleration`: an empty profile.
 */
class MotionProfile {
 public:
  /// @brief An empty profile: zero duration, all-zero samples.
  MotionProfile() = default;

  /**
   * @brief Build a profile, choosing the shape from the constraints.
   *
   * @details An S-curve when `constraints.max_jerk` is positive, a trapezoid
   * otherwise. This is the entry point most callers want.
   *
   * @param distance Signed displacement to cover.
   * @param constraints Velocity, acceleration and jerk limits.
   * @param initial_velocity Velocity at t = 0, signed in the caller's frame.
   * @param final_velocity Velocity to arrive with. Clamped to be non-negative
   *   in the travel frame; a "final velocity" pointing backwards is not a
   *   thing a single profile can express.
   * @return The profile. Empty when the constraints or the request are.
   */
  static MotionProfile generate(units::QLength distance, const ProfileConstraints& constraints,
                                units::QVelocity initial_velocity = units::QVelocity{},
                                units::QVelocity final_velocity = units::QVelocity{});

  /// @brief generate() against the direction-appropriate half of @p constraints.
  static MotionProfile generate(units::QLength distance, const DirectionalConstraints& constraints,
                                units::QVelocity initial_velocity = units::QVelocity{},
                                units::QVelocity final_velocity = units::QVelocity{});

  /**
   * @brief Build a trapezoidal profile, ignoring `max_jerk`.
   *
   * @details Three phases: ramp at `max_acceleration` to a peak velocity,
   * cruise, ramp down at `max_deceleration`. When the distance is too short to
   * reach `max_velocity` the cruise phase has zero length and the result is a
   * triangle that still lands exactly on the target.
   */
  static MotionProfile trapezoidal(units::QLength distance, const ProfileConstraints& constraints,
                                   units::QVelocity initial_velocity = units::QVelocity{},
                                   units::QVelocity final_velocity = units::QVelocity{});

  /**
   * @brief Build a jerk-limited (S-curve) profile.
   *
   * @details Seven phases: jerk up, hold acceleration, jerk down, cruise, then
   * the mirror image for the deceleration. Acceleration is continuous
   * everywhere, so the voltage command has no steps in it - which is the whole
   * reason to pay the extra time an S-curve costs.
   *
   * @warning Jerk limiting is only applied when the motion starts and ends at
   * rest. With a non-zero initial or final velocity this falls back to
   * trapezoidal(), because a jerk-limited profile through an arbitrary
   * non-zero boundary velocity has no closed form worth the code. The
   * fallback is silent by design: a path follower stitching segments together
   * gets a valid profile rather than an empty one.
   */
  static MotionProfile sCurve(units::QLength distance, const ProfileConstraints& constraints,
                              units::QVelocity initial_velocity = units::QVelocity{},
                              units::QVelocity final_velocity = units::QVelocity{});

  /// @brief Total time the profile takes. Zero for an empty profile.
  units::QTime duration() const { return m_duration; }

  /**
   * @brief The state at time @p t into the motion.
   *
   * @details Clamped at both ends: a negative @p t returns the initial state,
   * and any @p t past duration() returns the final state with the final
   * velocity and zero acceleration. That makes it safe to keep calling after a
   * motion has run long, which control loops do.
   */
  ProfileState sample(units::QTime t) const;

  /// @brief Whether @p t is at or past the end of the profile.
  bool isFinished(units::QTime t) const { return t >= m_duration; }

  /// @brief The state at duration(), i.e. where the profile ends up.
  ProfileState endState() const { return sample(m_duration); }

  /// @brief The distance the caller asked for.
  units::QLength requestedDistance() const { return m_requested_distance; }

  /**
   * @brief The displacement the profile actually produces.
   *
   * @details Equal to requestedDistance() to within floating-point noise in
   * every case except an overshoot, where it is larger.
   */
  units::QLength netDisplacement() const { return m_net_displacement; }

  /// @brief The highest speed reached, as a magnitude.
  units::QVelocity peakVelocity() const { return m_peak_velocity; }

  /// @brief +1, -1 or 0: the direction of travel this profile was built in.
  double direction() const { return m_direction; }

  /// @brief Whether the profile could not stop in the distance requested.
  bool overshoots() const { return m_overshoots; }

  /// @brief How many segments the profile is made of. 0 when empty.
  std::size_t segmentCount() const { return m_segment_count; }

  /**
   * @brief The profile time at which @p travelled has been covered.
   *
   * @details For a path follower that tracks progress by distance rather than
   * by a stopwatch. @p travelled is signed in the caller's frame and clamped
   * into the range the profile actually covers.
   *
   * @warning Only meaningful when displacement is monotonic, which it is
   * unless the initial velocity points away from the target. Solved by
   * bisection over the duration, so it is not free - cache the result rather
   * than calling it in a tight inner loop.
   */
  units::QTime timeAtDistance(units::QLength travelled) const;

  /**
   * @brief The commanded velocity once @p travelled has been covered.
   *
   * @details The velocity setpoint a path follower wants: "I am 18 inches
   * along, how fast should I be going?" Equivalent to
   * `sample(timeAtDistance(travelled)).velocity` and carries the same
   * monotonicity caveat.
   */
  units::QVelocity velocityAtDistance(units::QLength travelled) const;

 private:
  /// @brief The segment index covering profile time @p t.
  std::size_t segmentAt(units::QTime t) const;

  /// @brief Drop every segment and seed the chain with an initial velocity.
  void beginProfile(units::QVelocity initial_velocity);

  /**
   * @brief Append one segment, chaining position and velocity from the last.
   *
   * @details Silently drops a segment of non-positive duration, and one that
   * would exceed kMaxSegments. Both are "this phase does not exist", which is
   * exactly what a triangular profile's missing cruise phase is.
   *
   * @param duration How long the segment lasts.
   * @param start_acceleration Acceleration at the start of the segment, used
   *   only when @p chain_acceleration is false.
   * @param jerk Constant jerk over the segment.
   * @param chain_acceleration True to continue from the previous segment's
   *   final acceleration - what makes an S-curve's acceleration continuous.
   *   False to step the acceleration, which is what a trapezoid does.
   */
  void appendSegment(units::QTime duration, units::QAcceleration start_acceleration,
                     units::QJerk jerk, bool chain_acceleration);

  /// @brief Stamp duration, net displacement, peak and final velocity.
  void finalise();

  profile_detail::Segment m_segments[profile_detail::kMaxSegments]{};
  std::size_t m_segment_count = 0;
  units::QTime m_duration{};
  units::QLength m_requested_distance{};
  units::QLength m_net_displacement{};
  units::QVelocity m_peak_velocity{};
  units::QVelocity m_initial_velocity{};
  units::QVelocity m_final_velocity{};
  double m_direction = 0.0;
  bool m_overshoots = false;
};

}  // namespace control
}  // namespace mclib
