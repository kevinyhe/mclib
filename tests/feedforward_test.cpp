// mclib
//
// Numeric assertions on the kS/kV/kA model, the two characterisation fits, and
// the profile follower. Synthetic samples are generated from known gains and
// the fits have to recover those gains back.

#include "mclib/control/feedforward.hpp"
#include "mclib/control/profile.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

using mclib::control::AccelerationFit;
using mclib::control::AccelerationSample;
using mclib::control::FeedforwardGains;
using mclib::control::fitAccelerationGain;
using mclib::control::fitVelocityGains;
using mclib::control::inps2;
using mclib::control::MotionProfile;
using mclib::control::ProfileConstraints;
using mclib::control::ProfileFollower;
using mclib::control::ProfileFollowerConfig;
using mclib::control::ProfileState;
using mclib::control::QVoltagePerAcceleration;
using mclib::control::QVoltagePerLength;
using mclib::control::QVoltagePerVelocity;
using mclib::control::SimpleMotorFeedforward;
using mclib::control::VelocityFit;
using mclib::control::VelocitySample;
using mclib::units::inch;
using mclib::units::inps;
using mclib::units::QAcceleration;
using mclib::units::QLength;
using mclib::units::QTime;
using mclib::units::QVelocity;
using mclib::units::QVoltage;
using mclib::units::volt;

namespace {

/// @brief The model this file identifies: 0.8 V, 0.2 V per in/s, 0.02 V per in/s^2.
constexpr double kKsVolts = 0.8;
constexpr double kKvVoltsPerInps = 0.2;
constexpr double kKaVoltsPerInps2 = 0.02;

FeedforwardGains knownGains() {
  FeedforwardGains gains;
  gains.kS = kKsVolts * volt;
  gains.kV = kKvVoltsPerInps * volt / inps;
  gains.kA = kKaVoltsPerInps2 * volt / inps2;
  return gains;
}

/// @brief A fake millisecond clock the follower test drives by hand.
std::uint32_t g_fake_ms = 0;
std::uint32_t fakeClock() { return g_fake_ms; }

// -------------------------------------------------------------------------
// The model itself.
// -------------------------------------------------------------------------
void testModel() {
  const SimpleMotorFeedforward feedforward(knownGains());

  // Steady state with kA irrelevant: V = kS + kV * v, exactly.
  // 0.8 + 0.2 * 30 = 6.8 V.
  CHECK_NEAR(feedforward.calculate(30 * inps).volts(), 6.8, 1e-12);
  // Reverse flips only the static term: -0.8 + 0.2 * -30 = -6.8 V.
  CHECK_NEAR(feedforward.calculate(-30 * inps).volts(), -6.8, 1e-12);

  // With kA = 0 the two-argument form is identical to the one-argument form
  // at any acceleration - the definition of "steady state, kA = 0".
  FeedforwardGains no_ka = knownGains();
  no_ka.kA = QVoltagePerAcceleration{};
  const SimpleMotorFeedforward without_ka(no_ka);
  CHECK_EQ(without_ka.calculate(30 * inps, 500 * inps2).volts(),
           without_ka.calculate(30 * inps).volts());
  CHECK_NEAR(without_ka.calculate(30 * inps, 500 * inps2).volts(), 6.8, 1e-12);

  // Full model: 0.8 + 0.2*30 + 0.02*100 = 8.8 V.
  CHECK_NEAR(feedforward.calculate(30 * inps, 100 * inps2).volts(), 8.8, 1e-12);

  // Stationary and not accelerating means 0 V, not +kS.
  CHECK_EQ(feedforward.calculate(QVelocity{}, QAcceleration{}).volts(), 0.0);
  // Stationary but about to move takes the sign from the acceleration:
  // 0.8 + 0 + 0.02*100 = 2.8 V.
  CHECK_NEAR(feedforward.calculate(QVelocity{}, 100 * inps2).volts(), 2.8, 1e-12);
  CHECK_NEAR(feedforward.calculate(QVelocity{}, -100 * inps2).volts(), -2.8, 1e-12);

  // Sampling a profile state goes straight through.
  ProfileState state;
  state.velocity = 30 * inps;
  state.acceleration = 100 * inps2;
  CHECK_NEAR(feedforward.calculate(state).volts(), 8.8, 1e-12);

  // Inverting the model. At 12 V and zero acceleration:
  // (12 - 0.8) / 0.2 = 56 in/s.
  CHECK_NEAR(feedforward.maxAchievableVelocity(12 * volt, QAcceleration{}).inps(), 56.0, 1e-9);
  // At 12 V while already doing 30 in/s: (12 - 0.8 - 6) / 0.02 = 260 in/s^2.
  CHECK_NEAR(feedforward.maxAchievableAcceleration(12 * volt, 30 * inps).raw() / inps2.raw(),
             260.0, 1e-9);

  // Reverse: the model is odd-symmetric, so kS has to flip with the supply.
  // (-12 + 0.8) / 0.2 = -56 in/s, not (-12 - 0.8)/0.2 = -64.
  CHECK_NEAR(feedforward.maxAchievableVelocity(-12 * volt, QAcceleration{}).inps(), -56.0, 1e-9);
  // At rest the static term is signed by the supply: (12 - 0.8)/0.02 = 560.
  CHECK_NEAR(feedforward.maxAchievableAcceleration(12 * volt, QVelocity{}).raw() / inps2.raw(),
             560.0, 1e-9);
  CHECK_NEAR(feedforward.maxAchievableAcceleration(-12 * volt, QVelocity{}).raw() / inps2.raw(),
             -560.0, 1e-9);

  // A default-constructed model is inert, not accidentally biased.
  const SimpleMotorFeedforward inert;
  CHECK_EQ(inert.calculate(30 * inps, 100 * inps2).volts(), 0.0);
  // An unidentified gain gives zero, not infinity. These answers feed straight
  // into ProfileConstraints, where an infinity would become an unbounded limit
  // and the profile would command whatever it liked.
  CHECK_EQ(inert.maxAchievableVelocity(12 * volt, QAcceleration{}).inps(), 0.0);
  CHECK_EQ(inert.maxAchievableAcceleration(12 * volt, QVelocity{}).raw(), 0.0);
  // kA unidentified is the documented common case, so it gets its own check.
  CHECK_EQ(without_ka.maxAchievableAcceleration(12 * volt, 30 * inps).raw(), 0.0);
  CHECK_NEAR(without_ka.maxAchievableVelocity(12 * volt, QAcceleration{}).inps(), 56.0, 1e-9);
}

// -------------------------------------------------------------------------
// kS / kV recovery from synthetic steady-state samples.
// -------------------------------------------------------------------------
void testVelocityFit() {
  // Nine forward points at 4, 8, ... 36 in/s, generated from the known model.
  std::vector<VelocitySample> samples;
  for (int i = 1; i <= 9; ++i) {
    const double v = 4.0 * i;
    samples.push_back({(kKsVolts + kKvVoltsPerInps * v) * volt, v * inps});
  }

  const VelocityFit fit = fitVelocityGains(samples.data(), samples.size());
  CHECK(fit.valid);
  CHECK_EQ(static_cast<double>(fit.used), 9.0);
  CHECK_NEAR(fit.kS.volts(), kKsVolts, 1e-9);
  CHECK_NEAR((fit.kV * inps).volts(), kKvVoltsPerInps, 1e-9);
  CHECK_NEAR(fit.r_squared, 1.0, 1e-9);

  // Reverse samples fold onto the same line: appending them must not move the
  // answer at all.
  std::vector<VelocitySample> both = samples;
  for (int i = 1; i <= 9; ++i) {
    const double v = 4.0 * i;
    both.push_back({-(kKsVolts + kKvVoltsPerInps * v) * volt, -v * inps});
  }
  const VelocityFit bidirectional = fitVelocityGains(both.data(), both.size());
  CHECK(bidirectional.valid);
  CHECK_EQ(static_cast<double>(bidirectional.used), 18.0);
  CHECK_NEAR(bidirectional.kS.volts(), kKsVolts, 1e-9);
  CHECK_NEAR((bidirectional.kV * inps).volts(), kKvVoltsPerInps, 1e-9);

  // Stalled samples - voltage applied, robot did not move - are dropped by the
  // minimum-speed filter, so they do not drag the intercept down.
  std::vector<VelocitySample> with_stalls = samples;
  with_stalls.push_back({0.4 * volt, QVelocity{}});
  with_stalls.push_back({0.7 * volt, 0.1 * inps});
  const VelocityFit filtered = fitVelocityGains(with_stalls.data(), with_stalls.size());
  CHECK_EQ(static_cast<double>(filtered.used), 9.0);
  CHECK_NEAR(filtered.kS.volts(), kKsVolts, 1e-9);

  // Noise degrades r^2 but the gains stay close. The perturbation is a fixed
  // deterministic zig-zag so this test never flakes.
  std::vector<VelocitySample> noisy;
  for (int i = 1; i <= 9; ++i) {
    const double v = 4.0 * i;
    const double wobble = (i % 2 == 0) ? 0.05 : -0.05;
    noisy.push_back({(kKsVolts + kKvVoltsPerInps * v + wobble) * volt, v * inps});
  }
  const VelocityFit noisy_fit = fitVelocityGains(noisy.data(), noisy.size());
  CHECK(noisy_fit.valid);
  CHECK_NEAR(noisy_fit.kS.volts(), kKsVolts, 0.05);
  CHECK(noisy_fit.r_squared < 1.0);
  CHECK(noisy_fit.r_squared > 0.99);

  // Too few usable points, and every point at the same speed: both refused.
  const VelocitySample one = {2.0 * volt, 10 * inps};
  CHECK(!fitVelocityGains(&one, 1).valid);
  const VelocitySample flat[3] = {
      {2.0 * volt, 10 * inps}, {2.0 * volt, 10 * inps}, {2.0 * volt, 10 * inps}};
  CHECK(!fitVelocityGains(flat, 3).valid);
  CHECK(!fitVelocityGains(nullptr, 4).valid);

  std::printf("  kS/kV fit: recovered kS = %.9f V, kV = %.9f V per in/s (r^2 = %.9f)\n",
              fit.kS.volts(), (fit.kV * inps).volts(), fit.r_squared);
}

// -------------------------------------------------------------------------
// kA recovery, given kS and kV.
// -------------------------------------------------------------------------
void testAccelerationFit() {
  FeedforwardGains known = knownGains();
  const QVoltagePerAcceleration true_ka = known.kA;
  known.kA = QVoltagePerAcceleration{};

  // A synthetic ramp: velocity climbing, acceleration falling, voltages
  // generated from the full model.
  const SimpleMotorFeedforward truth(knownGains());
  std::vector<AccelerationSample> ramp;
  for (int i = 0; i < 40; ++i) {
    const QVelocity v = (2.0 * i) * inps;
    const QAcceleration a = (200.0 - 3.0 * i) * inps2;
    ramp.push_back({truth.calculate(v, a), v, a});
  }

  const AccelerationFit fit = fitAccelerationGain(ramp.data(), ramp.size(), known);
  CHECK(fit.valid);
  CHECK_EQ(static_cast<double>(fit.used), 40.0);
  CHECK_NEAR((fit.kA * inps2).volts(), kKaVoltsPerInps2, 1e-9);
  CHECK_NEAR(fit.kA.raw(), true_ka.raw(), 1e-9);

  // Every sample below the acceleration floor: nothing usable, so invalid
  // rather than a slope fitted through noise.
  std::vector<AccelerationSample> flat;
  for (int i = 0; i < 10; ++i) {
    flat.push_back({6.0 * volt, 26 * inps, QAcceleration{}});
  }
  CHECK(!fitAccelerationGain(flat.data(), flat.size(), known).valid);
  CHECK(!fitAccelerationGain(nullptr, 4, known).valid);

  std::printf("  kA fit: recovered kA = %.9f V per in/s^2 from %zu ramp samples\n",
              (fit.kA * inps2).volts(), fit.used);
}

// -------------------------------------------------------------------------
// The follower.
// -------------------------------------------------------------------------
void testFollower() {
  mclib::time::ScopedClock clock(&fakeClock);
  g_fake_ms = 0;

  ProfileFollowerConfig config;
  config.gains = knownGains();
  config.kp = 1.0 * volt / inch;
  const ProfileFollower probe(config);
  CHECK(probe.pid().usingDt());

  ProfileFollower follower(config);
  ProfileConstraints limits;
  limits.max_velocity = 48 * inps;
  limits.max_acceleration = 96 * inps2;
  const MotionProfile profile = MotionProfile::generate(48 * inch, limits);

  follower.follow(profile, QTime{});
  CHECK(!follower.isFinished(QTime{}));
  CHECK(follower.isFinished(QTime::fromBase(1.5)));

  // Perfect tracking: the position error is zero every tick, so the whole
  // command is feedforward and the feedback term is exactly 0 V.
  double worst_feedback = 0.0;
  double worst_command = 0.0;
  for (int step = 0; step <= 150; ++step) {
    const QTime now = QTime::fromBase(step * 0.01);
    const ProfileState setpoint = profile.sample(now);
    const QVoltage command = follower.update(setpoint.position, now);
    worst_feedback = std::fmax(worst_feedback, std::fabs(follower.lastFeedback().volts()));
    worst_command = std::fmax(worst_command, std::fabs(command.volts()));
    CHECK_EQ(follower.lastError().in(), 0.0);
  }
  CHECK_EQ(worst_feedback, 0.0);
  // Peak command is the top of the accel ramp at t just under 0.5 s:
  // 0.8 + 0.2*48 + 0.02*96 = 12.32 V, clamped by the follower to 12 V.
  CHECK_NEAR(worst_command, 12.0, 1e-12);

  // Cruise phase, no acceleration: 0.8 + 0.2 * 48 = 10.4 V exactly.
  follower.follow(profile, QTime{});
  const QVoltage cruise = follower.update(profile.sample(QTime::fromBase(0.75)).position,
                                          QTime::fromBase(0.75));
  CHECK_NEAR(cruise.volts(), 10.4, 1e-9);
  CHECK_NEAR(follower.lastFeedforward().volts(), 10.4, 1e-9);

  // Lagging by an inch adds exactly kp * 1 in of feedback on top.
  follower.follow(profile, QTime{});
  const QLength setpoint_position = profile.sample(QTime::fromBase(0.75)).position;
  const QVoltage lagging = follower.update(setpoint_position - 1 * inch, QTime::fromBase(0.75));
  CHECK_NEAR(follower.lastError().in(), 1.0, 1e-9);
  CHECK_NEAR(follower.lastFeedback().volts(), 1.0, 1e-9);
  CHECK_NEAR(lagging.volts(), 11.4, 1e-9);

  // The clamp holds in both directions.
  follower.setMaxVoltage(6 * volt);
  const QVoltage clamped = follower.update(setpoint_position - 1 * inch, QTime::fromBase(0.76));
  CHECK_NEAR(clamped.volts(), 6.0, 1e-12);
  follower.setMaxVoltage(12 * volt);

  // reset() stops it commanding anything.
  follower.reset();
  CHECK(follower.isFinished(QTime{}));
  CHECK_EQ(follower.update(10 * inch, QTime::fromBase(0.5)).volts(), 0.0);

  // update() with no explicit time reads the installed clock.
  g_fake_ms = 4000;
  follower.follow(profile);
  g_fake_ms = 4750;
  const QVoltage from_clock = follower.update(profile.sample(QTime::fromBase(0.75)).position);
  CHECK_NEAR(from_clock.volts(), 10.4, 1e-9);
  CHECK_NEAR(follower.elapsed(QTime::fromBase(4.75)).s(), 0.75, 1e-12);
  g_fake_ms = 6000;
  CHECK(follower.isFinished());

  // calculate() is the seam a path follower uses: it brings its own setpoint
  // and its own dt, and never touches the stopwatch.
  ProfileFollower direct(config);
  ProfileState wanted;
  wanted.velocity = 24 * inps;
  const QVoltage from_setpoint = direct.calculate(wanted, QLength{}, 10 * mclib::units::millisecond);
  // 0.8 + 0.2*24 = 5.6 V feedforward, zero error so zero feedback.
  CHECK_NEAR(from_setpoint.volts(), 5.6, 1e-9);
}

}  // namespace

int main() {
  testModel();
  testVelocityFit();
  testAccelerationFit();
  testFollower();
  return mclib::test::summary("feedforward");
}
