// mclib
//
// Numeric assertions on the profile generator. This unit is pure arithmetic,
// so the numbers below are the entire verification: every expected value is
// worked out by hand in the comment above it, not read back out of the
// implementation.

#include "mclib/control/profile.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>

using mclib::control::inps2;
using mclib::control::inps3;
using mclib::control::MotionProfile;
using mclib::control::ProfileConstraints;
using mclib::control::ProfileState;
using mclib::units::QAcceleration;
using mclib::units::QLength;
using mclib::units::QTime;
using mclib::units::QVelocity;
using mclib::units::inps;

namespace {

/// @brief 48 in/s, 96 in/s^2 both ways. The reference drivetrain for this file.
ProfileConstraints symmetricLimits() {
  ProfileConstraints limits;
  limits.max_velocity = 48 * inps;
  limits.max_acceleration = 96 * inps2;
  return limits;
}

/**
 * @brief Integrate the sampled velocity with the trapezoid rule.
 *
 * @details Deliberately independent of the profile's own position output: if
 * the position polynomial and the velocity polynomial ever disagree, this is
 * what catches it.
 */
double integratedInches(const MotionProfile& profile, int steps) {
  const double total = profile.duration().raw();
  const double h = total / steps;
  double sum = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double v0 = profile.sample(QTime::fromBase(i * h)).velocity.inps();
    const double v1 = profile.sample(QTime::fromBase((i + 1) * h)).velocity.inps();
    sum += 0.5 * (v0 + v1) * h;
  }
  return sum;
}

/// @brief Largest |velocity| seen over `steps` uniform samples, in in/s.
double sampledPeakSpeed(const MotionProfile& profile, int steps) {
  double peak = 0.0;
  for (int i = 0; i <= steps; ++i) {
    const double t = profile.duration().raw() * i / steps;
    peak = std::fmax(peak, std::fabs(profile.sample(QTime::fromBase(t)).velocity.inps()));
  }
  return peak;
}

/// @brief Largest |acceleration| seen over `steps` uniform samples, in in/s^2.
double sampledPeakAccel(const MotionProfile& profile, int steps) {
  double peak = 0.0;
  for (int i = 0; i <= steps; ++i) {
    // Sampled just inside the profile: at exactly duration() the acceleration
    // is reported as zero by contract, which would not be a fair maximum.
    const double t = profile.duration().raw() * i / (steps + 1);
    peak = std::fmax(peak, std::fabs(profile.sample(QTime::fromBase(t)).acceleration.raw()));
  }
  return peak / inps2.raw();
}

/// @brief Largest position step between adjacent samples, in inches.
double largestPositionStep(const MotionProfile& profile, int steps) {
  double worst = 0.0;
  double previous = profile.sample(QTime{}).position.in();
  for (int i = 1; i <= steps; ++i) {
    const double t = profile.duration().raw() * i / steps;
    const double next = profile.sample(QTime::fromBase(t)).position.in();
    worst = std::fmax(worst, std::fabs(next - previous));
    previous = next;
  }
  return worst;
}

// -------------------------------------------------------------------------
// Trapezoid: the worked example.
//
// 48 in at 48 in/s with 96 in/s^2 both ways.
//   accel: 48 / 96          = 0.5 s, covering 48^2 / (2*96) = 12 in
//   decel: same             = 0.5 s, 12 in
//   cruise: 48 - 12 - 12    = 24 in at 48 in/s = 0.5 s
//   total                   = 1.5 s, ending at 48 in.
// -------------------------------------------------------------------------
void testWorkedTrapezoid() {
  const MotionProfile profile = MotionProfile::generate(48 * mclib::units::inch, symmetricLimits());

  CHECK_NEAR(profile.duration().s(), 1.5, 1e-12);
  CHECK_NEAR(profile.netDisplacement().in(), 48.0, 1e-9);
  CHECK_NEAR(profile.peakVelocity().inps(), 48.0, 1e-9);
  CHECK_EQ(profile.direction(), 1.0);
  CHECK(!profile.overshoots());
  CHECK_EQ(static_cast<double>(profile.segmentCount()), 3.0);

  // Phase boundaries, by hand.
  CHECK_NEAR(profile.sample(QTime::fromBase(0.5)).position.in(), 12.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.5)).velocity.inps(), 48.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(1.0)).position.in(), 36.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(1.0)).velocity.inps(), 48.0, 1e-9);
  // Midpoint of the cruise: 12 + 12 = 24 in at 0.75 s.
  CHECK_NEAR(profile.sample(QTime::fromBase(0.75)).position.in(), 24.0, 1e-9);
  // Quarter of the way up the accel ramp: t = 0.25 -> v = 24, x = 3.
  CHECK_NEAR(profile.sample(QTime::fromBase(0.25)).velocity.inps(), 24.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.25)).position.in(), 3.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.25)).acceleration.raw() / inps2.raw(), 96.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(1.25)).acceleration.raw() / inps2.raw(), -96.0, 1e-9);

  // End state, and the clamp past the end.
  CHECK_NEAR(profile.endState().position.in(), 48.0, 1e-9);
  CHECK_NEAR(profile.endState().velocity.inps(), 0.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(5.0)).position.in(), 48.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(5.0)).velocity.inps(), 0.0, 1e-12);
  CHECK_NEAR(profile.sample(QTime::fromBase(-1.0)).position.in(), 0.0, 1e-12);
  CHECK(profile.isFinished(QTime::fromBase(1.5)));
  CHECK(!profile.isFinished(QTime::fromBase(1.4999)));

  // Independent integration of the velocity curve, 2000 steps. The velocity is
  // piecewise linear, so the trapezoid rule is exact apart from the two steps
  // that straddle a corner - hence 1e-3 in rather than 1e-12.
  CHECK_NEAR(integratedInches(profile, 2000), 48.0, 1e-3);

  // Limits are never exceeded at any sample.
  CHECK(sampledPeakSpeed(profile, 4000) <= 48.0 + 1e-9);
  CHECK(sampledPeakAccel(profile, 4000) <= 96.0 + 1e-9);

  std::printf("  worked trapezoid: 48 in, 48 in/s, 96 in/s^2 -> %.6f s, ends at %.9f in\n",
              profile.duration().s(), profile.netDisplacement().in());
}

// -------------------------------------------------------------------------
// Triangular: too short to cruise.
//
// 6 in with acc = dec = 96 in/s^2. Peak velocity solves
//   6 = v^2/(2*96) + v^2/(2*96) = v^2/96  ->  v = sqrt(576) = 24 in/s.
// Each ramp is 24/96 = 0.25 s, so 0.5 s total, and it still lands on 6 in.
// -------------------------------------------------------------------------
void testTriangular() {
  const MotionProfile profile = MotionProfile::generate(6 * mclib::units::inch, symmetricLimits());

  CHECK_NEAR(profile.peakVelocity().inps(), 24.0, 1e-9);
  CHECK_NEAR(profile.duration().s(), 0.5, 1e-12);
  CHECK_NEAR(profile.netDisplacement().in(), 6.0, 1e-9);
  // Cruise phase does not exist, so only two segments were emitted.
  CHECK_EQ(static_cast<double>(profile.segmentCount()), 2.0);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.25)).velocity.inps(), 24.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.25)).position.in(), 3.0, 1e-9);
  CHECK(sampledPeakSpeed(profile, 2000) <= 24.0 + 1e-9);
  CHECK(sampledPeakAccel(profile, 2000) <= 96.0 + 1e-9);
  CHECK_NEAR(integratedInches(profile, 2000), 6.0, 1e-4);
}

// -------------------------------------------------------------------------
// Asymmetric accel / decel, with the time split predicted up front.
//
// 48 in, v_max 48 in/s, acc 96 in/s^2, dec 48 in/s^2.
//   accel: 48/96 = 0.50 s over 48^2/(2*96) = 12 in
//   decel: 48/48 = 1.00 s over 48^2/(2*48) = 24 in
//   cruise: 48 - 12 - 24 = 12 in at 48 in/s = 0.25 s
//   total = 1.75 s.
// The reverse profile is the mirror image: same times, negated everything.
// -------------------------------------------------------------------------
void testAsymmetric() {
  ProfileConstraints limits = symmetricLimits();
  limits.max_deceleration = 48 * inps2;

  const MotionProfile profile = MotionProfile::generate(48 * mclib::units::inch, limits);
  CHECK_NEAR(profile.duration().s(), 1.75, 1e-12);
  CHECK_NEAR(profile.netDisplacement().in(), 48.0, 1e-9);

  // The predicted split, read back off the phase boundaries.
  CHECK_NEAR(profile.sample(QTime::fromBase(0.50)).position.in(), 12.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.50)).velocity.inps(), 48.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.75)).position.in(), 24.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.75)).velocity.inps(), 48.0, 1e-9);
  // Halfway down the 1.0 s decel ramp: v = 24 in/s.
  CHECK_NEAR(profile.sample(QTime::fromBase(1.25)).velocity.inps(), 24.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(1.25)).acceleration.raw() / inps2.raw(), -48.0, 1e-9);
  CHECK(sampledPeakAccel(profile, 4000) <= 96.0 + 1e-9);

  // Same request the other way round.
  const MotionProfile reverse = MotionProfile::generate(-48 * mclib::units::inch, limits);
  CHECK_NEAR(reverse.duration().s(), 1.75, 1e-12);
  CHECK_NEAR(reverse.netDisplacement().in(), -48.0, 1e-9);
  CHECK_EQ(reverse.direction(), -1.0);
  CHECK_NEAR(reverse.sample(QTime::fromBase(0.50)).velocity.inps(), -48.0, 1e-9);
  CHECK_NEAR(reverse.sample(QTime::fromBase(1.25)).acceleration.raw() / inps2.raw(), 48.0, 1e-9);
  CHECK_NEAR(reverse.peakVelocity().inps(), 48.0, 1e-9);

  // DirectionalConstraints picks the reverse half for a negative distance.
  mclib::control::DirectionalConstraints split;
  split.forward = symmetricLimits();
  split.reverse = limits;
  const MotionProfile chosen = MotionProfile::generate(-48 * mclib::units::inch, split);
  CHECK_NEAR(chosen.duration().s(), 1.75, 1e-12);
  const MotionProfile chosen_fwd = MotionProfile::generate(48 * mclib::units::inch, split);
  CHECK_NEAR(chosen_fwd.duration().s(), 1.5, 1e-12);
}

// -------------------------------------------------------------------------
// Degenerate requests.
// -------------------------------------------------------------------------
void testDegenerate() {
  // Zero distance from rest: nothing to do.
  const MotionProfile empty = MotionProfile::generate(QLength{}, symmetricLimits());
  CHECK_EQ(empty.duration().s(), 0.0);
  CHECK_EQ(empty.netDisplacement().in(), 0.0);
  CHECK_EQ(static_cast<double>(empty.segmentCount()), 0.0);
  CHECK_EQ(empty.sample(QTime::fromBase(1.0)).velocity.inps(), 0.0);
  CHECK(empty.isFinished(QTime{}));

  // Non-positive limits: also empty, rather than a divide by zero.
  ProfileConstraints broken;
  broken.max_velocity = QVelocity{};
  broken.max_acceleration = 96 * inps2;
  CHECK_EQ(MotionProfile::generate(24 * mclib::units::inch, broken).duration().s(), 0.0);
  broken.max_velocity = 48 * inps;
  broken.max_acceleration = QAcceleration{};
  CHECK_EQ(MotionProfile::generate(24 * mclib::units::inch, broken).duration().s(), 0.0);

  // Negative distance: mirror image of the positive one.
  const MotionProfile back = MotionProfile::generate(-48 * mclib::units::inch, symmetricLimits());
  CHECK_NEAR(back.duration().s(), 1.5, 1e-12);
  CHECK_NEAR(back.netDisplacement().in(), -48.0, 1e-9);
  CHECK_NEAR(back.sample(QTime::fromBase(0.75)).position.in(), -24.0, 1e-9);
  CHECK_NEAR(integratedInches(back, 2000), -48.0, 1e-3);
}

// -------------------------------------------------------------------------
// Non-zero initial velocity.
//
// Case 1 - already cruising. 48 in, starting at 48 in/s, acc = dec = 96.
//   No accel phase. Decel from 48 to 0 takes 0.5 s over 12 in, leaving 36 in
//   of cruise at 48 in/s = 0.75 s. Total 1.25 s.
//
// Case 2 - moving backwards. 24 in, starting at -24 in/s.
//   Phase 1 accelerates at +96 from -24 through 0 up to the peak. Solving
//   24 = (v^2 - 24^2)/(2*96) + v^2/(2*96) gives v^2 = (24*192 + 576)/2 = 2592,
//   v = 50.91 in/s, above the 48 cap, so it is a real trapezoid at 48 in/s.
//   Net displacement still lands on 24 in even though the first 0.25 s go the
//   wrong way.
//
// Case 3 - too fast to stop. 2 in, starting at 48 in/s, dec 96.
//   Stopping needs 48^2/(2*96) = 12 in > 2 in, so the profile is a single
//   0.5 s brake covering 12 in and overshoots() is true.
// -------------------------------------------------------------------------
void testInitialVelocity() {
  const MotionProfile cruising =
      MotionProfile::generate(48 * mclib::units::inch, symmetricLimits(), 48 * inps);
  CHECK_NEAR(cruising.duration().s(), 1.25, 1e-12);
  CHECK_NEAR(cruising.netDisplacement().in(), 48.0, 1e-9);
  CHECK_NEAR(cruising.sample(QTime{}).velocity.inps(), 48.0, 1e-12);
  CHECK_EQ(static_cast<double>(cruising.segmentCount()), 2.0);
  CHECK(!cruising.overshoots());
  CHECK_NEAR(integratedInches(cruising, 2000), 48.0, 1e-3);

  const MotionProfile backwards =
      MotionProfile::generate(24 * mclib::units::inch, symmetricLimits(), -24 * inps);
  CHECK_NEAR(backwards.sample(QTime{}).velocity.inps(), -24.0, 1e-12);
  CHECK_NEAR(backwards.netDisplacement().in(), 24.0, 1e-9);
  // 0.25 s to come back to a standstill, having gone 3 in the wrong way.
  CHECK_NEAR(backwards.sample(QTime::fromBase(0.25)).velocity.inps(), 0.0, 1e-9);
  CHECK_NEAR(backwards.sample(QTime::fromBase(0.25)).position.in(), -3.0, 1e-9);
  CHECK(sampledPeakSpeed(backwards, 4000) <= 48.0 + 1e-9);
  CHECK_NEAR(integratedInches(backwards, 4000), 24.0, 1e-3);

  const MotionProfile overshoot =
      MotionProfile::generate(2 * mclib::units::inch, symmetricLimits(), 48 * inps);
  CHECK(overshoot.overshoots());
  CHECK_NEAR(overshoot.duration().s(), 0.5, 1e-12);
  CHECK_NEAR(overshoot.netDisplacement().in(), 12.0, 1e-9);
  CHECK_NEAR(overshoot.requestedDistance().in(), 2.0, 1e-12);
  CHECK_NEAR(overshoot.endState().velocity.inps(), 0.0, 1e-9);
  CHECK_EQ(static_cast<double>(overshoot.segmentCount()), 1.0);

  // Zero distance while already moving is the same story: brake at the limit.
  const MotionProfile braking = MotionProfile::generate(QLength{}, symmetricLimits(), 24 * inps);
  CHECK(braking.overshoots());
  CHECK_NEAR(braking.duration().s(), 0.25, 1e-12);
  CHECK_NEAR(braking.netDisplacement().in(), 3.0, 1e-9);

  // A non-zero final velocity: 48 in ending at 24 in/s. The decel ramp is
  // (48-24)/96 = 0.25 s over (48^2-24^2)/(2*96) = 9 in.
  const MotionProfile flying = MotionProfile::generate(48 * mclib::units::inch, symmetricLimits(),
                                                      QVelocity{}, 24 * inps);
  CHECK_NEAR(flying.endState().velocity.inps(), 24.0, 1e-9);
  CHECK_NEAR(flying.netDisplacement().in(), 48.0, 1e-9);
  // 12 in accel + 9 in decel leaves 27 in of cruise = 0.5625 s.
  CHECK_NEAR(flying.duration().s(), 0.5 + 0.5625 + 0.25, 1e-12);

  // The mirror of case 3: too short to *reach* the requested final velocity.
  // Getting from rest to 48 in/s at 96 in/s^2 needs 12 in; asking for it in
  // 4 in has to overshoot, and has to say so.
  const MotionProfile too_short = MotionProfile::generate(4 * mclib::units::inch,
                                                          symmetricLimits(), QVelocity{},
                                                          48 * inps);
  CHECK(too_short.overshoots());
  CHECK_NEAR(too_short.duration().s(), 0.5, 1e-12);
  CHECK_NEAR(too_short.netDisplacement().in(), 12.0, 1e-9);
  CHECK_NEAR(too_short.endState().velocity.inps(), 48.0, 1e-9);
  CHECK_EQ(static_cast<double>(too_short.segmentCount()), 1.0);
  // One inch past the threshold it fits again, and no longer overshoots.
  const MotionProfile just_fits = MotionProfile::generate(13 * mclib::units::inch,
                                                          symmetricLimits(), QVelocity{},
                                                          48 * inps);
  CHECK(!just_fits.overshoots());
  CHECK_NEAR(just_fits.netDisplacement().in(), 13.0, 1e-9);
}

// -------------------------------------------------------------------------
// DirectionalConstraints has to pick by the direction of travel, which for a
// zero-distance "stop from where you are" is the sign of the initial
// velocity, not the sign of the distance.
// -------------------------------------------------------------------------
void testDirectionalTiebreak() {
  mclib::control::DirectionalConstraints split;
  split.forward = symmetricLimits();
  split.reverse = symmetricLimits();
  split.reverse.max_deceleration = 12 * inps2;

  // Rolling backwards at 24 in/s with nowhere left to go: the reverse decel
  // limit of 12 in/s^2 gives 2.0 s, not the forward 96 in/s^2's 0.25 s.
  const MotionProfile braking = MotionProfile::generate(QLength{}, split, -24 * inps);
  CHECK_EQ(braking.direction(), -1.0);
  CHECK_NEAR(braking.duration().s(), 2.0, 1e-12);
  CHECK_NEAR(braking.netDisplacement().in(), -24.0, 1e-9);

  // Rolling forwards, same request: the forward half, 0.25 s.
  const MotionProfile forward_braking = MotionProfile::generate(QLength{}, split, 24 * inps);
  CHECK_EQ(forward_braking.direction(), 1.0);
  CHECK_NEAR(forward_braking.duration().s(), 0.25, 1e-12);

  // select() agrees with what generate() used.
  CHECK_NEAR(split.select(QLength{}, -24 * inps).max_deceleration.raw() / inps2.raw(), 12.0, 1e-9);
  CHECK_EQ(split.select(QLength{}, 24 * inps).max_deceleration.raw() / inps2.raw(), 0.0);
  CHECK_NEAR(split.select(-5 * mclib::units::inch).max_deceleration.raw() / inps2.raw(), 12.0, 1e-9);
}

// -------------------------------------------------------------------------
// S-curve.
//
// 48 in, v_max 48 in/s, acc 96 in/s^2, jerk 384 in/s^3.
//   jerk time to reach 96 in/s^2: 96/384 = 0.25 s, gaining 96^2/384 = 24 in/s
//   -> the acceleration limit is reached, so the ramp holds 96 in/s^2 for
//   48/96 - 0.25 = 0.25 s.
//   Ramp time = 0.25 + 0.25 + 0.25 = 0.75 s, over 48 * 0.75 / 2 = 18 in.
//   Two ramps = 36 in, leaving 12 in of cruise = 0.25 s.
//   Total = 0.75 + 0.25 + 0.75 = 1.75 s.
// -------------------------------------------------------------------------
void testSCurve() {
  ProfileConstraints limits = symmetricLimits();
  limits.max_jerk = 384 * inps3;
  CHECK(limits.jerkLimited());

  const MotionProfile profile = MotionProfile::generate(48 * mclib::units::inch, limits);
  CHECK_NEAR(profile.duration().s(), 1.75, 1e-12);
  CHECK_NEAR(profile.netDisplacement().in(), 48.0, 1e-9);
  CHECK_NEAR(profile.peakVelocity().inps(), 48.0, 1e-9);
  CHECK_EQ(static_cast<double>(profile.segmentCount()), 7.0);

  // Acceleration starts and ends at zero, and peaks at exactly the limit.
  CHECK_NEAR(profile.sample(QTime{}).acceleration.raw() / inps2.raw(), 0.0, 1e-12);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.25)).acceleration.raw() / inps2.raw(), 96.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.375)).acceleration.raw() / inps2.raw(), 96.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.75)).acceleration.raw() / inps2.raw(), 0.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.75)).velocity.inps(), 48.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(0.75)).position.in(), 18.0, 1e-9);
  CHECK_NEAR(profile.sample(QTime::fromBase(1.0)).position.in(), 30.0, 1e-9);
  CHECK(sampledPeakAccel(profile, 6000) <= 96.0 + 1e-9);
  CHECK(sampledPeakSpeed(profile, 6000) <= 48.0 + 1e-9);
  CHECK_NEAR(integratedInches(profile, 6000), 48.0, 1e-4);

  // Jerk is bounded: numerically differentiate the sampled acceleration.
  // The four jerk segments run at exactly +/-384 and the rest at 0, so the
  // finite difference lands on the limit and never past it.
  const int steps = 20000;
  const double h = profile.duration().raw() / steps;
  double worst_jerk = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double a0 = profile.sample(QTime::fromBase(i * h)).acceleration.raw();
    const double a1 = profile.sample(QTime::fromBase((i + 1) * h)).acceleration.raw();
    worst_jerk = std::fmax(worst_jerk, std::fabs(a1 - a0) / h);
  }
  CHECK(worst_jerk / inps3.raw() <= 384.0 + 1e-6);
  CHECK_NEAR(worst_jerk / inps3.raw(), 384.0, 1.0);

  // Position is continuous: no sample step exceeds what the peak velocity
  // could cover in one step. A discontinuity would blow straight past this.
  const int continuity_steps = 20000;
  const double bound = 48.0 * (profile.duration().s() / continuity_steps) * 1.0001;
  CHECK(largestPositionStep(profile, continuity_steps) <= bound);

  // Short S-curve: 3 in, too short to reach either the velocity or the
  // acceleration limit. It still lands exactly on target.
  const MotionProfile tiny = MotionProfile::generate(3 * mclib::units::inch, limits);
  CHECK_NEAR(tiny.netDisplacement().in(), 3.0, 1e-9);
  CHECK(tiny.peakVelocity().inps() < 48.0);
  CHECK(sampledPeakAccel(tiny, 4000) <= 96.0 + 1e-9);
  CHECK_NEAR(integratedInches(tiny, 4000), 3.0, 1e-4);

  // Documented fallback: a non-zero boundary velocity gives a trapezoid, not
  // an empty profile.
  const MotionProfile fallback =
      MotionProfile::generate(48 * mclib::units::inch, limits, 48 * inps);
  CHECK_NEAR(fallback.duration().s(), 1.25, 1e-12);
  CHECK_NEAR(fallback.netDisplacement().in(), 48.0, 1e-9);

  // An S-curve takes longer than the trapezoid over the same distance - that
  // is what the smoothness costs.
  CHECK(profile.duration() > MotionProfile::generate(48 * mclib::units::inch,
                                                     symmetricLimits())
                                 .duration());
}

// -------------------------------------------------------------------------
// The distance-parameterised seam a path follower uses.
// -------------------------------------------------------------------------
void testDistanceLookup() {
  const MotionProfile profile = MotionProfile::generate(48 * mclib::units::inch, symmetricLimits());

  // 12 in is the top of the accel ramp: t = 0.5 s, v = 48 in/s.
  CHECK_NEAR(profile.timeAtDistance(12 * mclib::units::inch).s(), 0.5, 1e-9);
  CHECK_NEAR(profile.velocityAtDistance(12 * mclib::units::inch).inps(), 48.0, 1e-6);
  // 3 in is a quarter of the way up: t = 0.25 s, v = 24 in/s.
  CHECK_NEAR(profile.timeAtDistance(3 * mclib::units::inch).s(), 0.25, 1e-9);
  CHECK_NEAR(profile.velocityAtDistance(3 * mclib::units::inch).inps(), 24.0, 1e-6);
  // Clamped at both ends.
  CHECK_EQ(profile.timeAtDistance(-5 * mclib::units::inch).s(), 0.0);
  CHECK_NEAR(profile.timeAtDistance(100 * mclib::units::inch).s(), 1.5, 1e-12);
  CHECK_NEAR(profile.velocityAtDistance(100 * mclib::units::inch).inps(), 0.0, 1e-9);

  // Same, mirrored, for a reverse profile.
  const MotionProfile back = MotionProfile::generate(-48 * mclib::units::inch, symmetricLimits());
  CHECK_NEAR(back.timeAtDistance(-12 * mclib::units::inch).s(), 0.5, 1e-9);
  CHECK_NEAR(back.velocityAtDistance(-12 * mclib::units::inch).inps(), -48.0, 1e-6);
}

}  // namespace

int main() {
  testWorkedTrapezoid();
  testTriangular();
  testAsymmetric();
  testDegenerate();
  testInitialVelocity();
  testDirectionalTiebreak();
  testSCurve();
  testDistanceLookup();
  return mclib::test::summary("profile");
}
