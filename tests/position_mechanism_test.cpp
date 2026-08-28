// mclib
//
// Behaviour of PositionMechanismConfig::hold_output. Everything runs on a
// mclib::time::ScopedClock so the arrival dwell is exact and reproducible.

#include "mclib/mechanism/position_mechanism.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>

using mclib::mechanism::PositionMechanism;
using mclib::mechanism::PositionMechanismConfig;

namespace {

std::uint32_t g_fake_ms = 0;

constexpr double kTickMs = 10.0;

/// kp = 0.5 V per unit of error, ki live but expected to stay dead inside the
/// settle band, kd off so the held voltage is exactly the P term.
PositionMechanismConfig baseConfig(bool hold_output) {
  PositionMechanismConfig config;
  config.kp = 0.5;
  config.ki = 0.1;
  config.kd = 0.0;
  config.max_voltage = 12.0;
  config.small_error = 2.0;
  config.big_error = 5.0;
  config.small_duration_ms = 50.0;
  config.big_duration_ms = 250.0;
  config.derivative_tolerance = 5.0;
  config.hold_output = hold_output;
  return config;
}

/// One mechanism plus the position and voltage it is wired to.
struct Rig {
  double position = 0.0;
  double volts = 0.0;
  PositionMechanism mechanism;

  explicit Rig(bool hold_output)
      : mechanism([this]() { return position; },
                  [this](double v) { volts = v; },
                  baseConfig(hold_output)) {}

  /// Advance the fake clock one tick, then run one scheduler pass.
  void tick() {
    g_fake_ms += static_cast<std::uint32_t>(kTickMs);
    mechanism.runPeriodic();
  }
};

/// Sit exactly on the target until the loop latches arrival. Returns the tick
/// on which atTarget() first became true, or -1 if it never did.
int settle(Rig& rig, int max_ticks) {
  for (int i = 1; i <= max_ticks; ++i) {
    rig.tick();
    if (rig.mechanism.atTarget()) {
      return i;
    }
  }
  return -1;
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });

  // The default is hold_output true: a positional mechanism exists to hold.
  CHECK(PositionMechanismConfig{}.hold_output);

  // --- Both modes settle on the same tick -----------------------------------
  Rig off(false);
  Rig on(true);

  off.mechanism.moveTo(10.0);
  on.mechanism.moveTo(10.0);
  off.position = 10.0;
  on.position = 10.0;

  const int off_tick = settle(off, 50);
  const int on_tick = settle(on, 50);
  std::printf("arrival tick: hold_output off = %d, on = %d (%.0f ms dwell at "
              "%.0f ms/tick)\n",
              off_tick, on_tick, 50.0, kTickMs);
  CHECK(off_tick > 0);
  CHECK_EQ(on_tick, off_tick);

  // --- A sustained error after arrival --------------------------------------
  // Drop 1.5 units below the target: still inside small_error, so the loop is
  // still "arrived" and the integral is still being zeroed every tick.
  off.position = 8.5;
  on.position = 8.5;
  for (int i = 0; i < 20; ++i) {
    off.tick();
    on.tick();
  }
  std::printf("held at error 1.5: hold_output off = %.4f V, on = %.4f V\n",
              off.volts, on.volts);

  // Historical behaviour, unchanged: 0 V, and the drift is inside the band so
  // the re-arm does not fire either.
  CHECK_NEAR(off.volts, 0.0, 1e-12);
  // kp * error, with no integral contribution after 20 held ticks.
  CHECK_NEAR(on.volts, 0.5 * 1.5, 1e-12);

  // Arrival did not move.
  CHECK(off.mechanism.atTarget());
  CHECK(on.mechanism.atTarget());

  // --- The droop bound is real ----------------------------------------------
  // At the very edge of the settle band the held output is exactly
  // kp * small_error. That is the most the loop can push back with, so
  // small_error is a hard bound on steady-state droop, not a zero-error hold.
  on.position = 10.0 - 2.0;
  for (int i = 0; i < 30; ++i) {
    on.tick();
  }
  const double edge_expected = 0.5 * 2.0;
  std::printf("held at the band edge (error = small_error = 2.0): %.4f V, "
              "kp * small_error = %.4f V\n",
              on.volts, edge_expected);
  CHECK_NEAR(on.volts, edge_expected, 1e-12);

  // ki is 0.1 and 30 ticks of error 2.0 would be worth 6.0 V of integral if it
  // accumulated. It does not: update() zeroes the accumulator over exactly the
  // arrival band.
  CHECK(on.volts < 1.0 + 1e-12);

  // --- Outside the band -----------------------------------------------------
  // Past small_error the integral is live again, so the output climbs above
  // the pure P term instead of sitting on it.
  on.position = 10.0 - 3.0;
  on.tick();
  const double first_out_of_band = on.volts;
  for (int i = 0; i < 10; ++i) {
    on.tick();
  }
  std::printf("outside the band (error 3.0): first tick %.4f V, after 10 more "
              "%.4f V (kp * 3.0 = %.4f V)\n",
              first_out_of_band, on.volts, 1.5);
  CHECK_NEAR(first_out_of_band, 0.5 * 3.0 + 0.1 * 3.0, 1e-12);
  CHECK(on.volts > first_out_of_band);

  // With hold_output off the same drift re-arms the PID, which is the old
  // workaround. It still works, and it is the only thing driving in that mode.
  off.position = 10.0 - 3.0;
  off.tick();
  std::printf("hold_output off, drift 3.0 past the band: %.4f V (re-arm)\n",
              off.volts);
  CHECK(off.volts > 0.0);
  // atTarget() is sticky across the re-arm.
  CHECK(off.mechanism.atTarget());

  // --- The voltage clamp still applies to a held output ---------------------
  Rig clamped(true);
  clamped.mechanism.moveTo(0.0);
  clamped.position = 0.0;
  CHECK(settle(clamped, 50) > 0);
  clamped.position = -1000.0;
  clamped.tick();
  std::printf("held output past the clamp: %.4f V\n", clamped.volts);
  CHECK_NEAR(clamped.volts, 12.0, 1e-12);

  // --- Integral windup is bounded -------------------------------------------
  // The hazard hold_output introduces: nothing clears the accumulator while
  // the mechanism sits outside small_error, because PID::update() only throws
  // it away once |error| falls back inside that band. A stuck mechanism would
  // walk the command to max_voltage and park a motor there. integral_max_volts
  // is what stops it.
  {
    Rig capped(true);
    capped.mechanism.moveTo(10.0);
    capped.position = 10.0;
    CHECK(settle(capped, 50) > 0);

    // Hold it at 5: error 5, inside big_error, outside small_error. 3 s at
    // 10 ms/tick is 300 ticks of uncleared accumulation.
    capped.position = 5.0;
    for (int i = 0; i < 300; ++i) {
      capped.tick();
    }
    std::printf("stuck outside the band for 3 s: %.4f V (max_voltage = %.1f, "
                "integral_max_volts = %.1f)\n",
                capped.volts, 12.0, PositionMechanismConfig{}.integral_max_volts);
    // kp * 5 = 2.5, plus the integral pinned at its 2.0 V cap.
    CHECK_NEAR(capped.volts, 0.5 * 5.0 + 2.0, 1e-9);
    // The point of the assertion: nowhere near the rail.
    CHECK(capped.volts < 12.0);
  }

  // --- A manual override still wins -----------------------------------------
  on.mechanism.setManualVoltage(0.0);
  on.tick();
  CHECK_NEAR(on.volts, 0.0, 1e-12);
  CHECK(!on.mechanism.atTarget());

  return mclib::test::summary("position_mechanism");
}
