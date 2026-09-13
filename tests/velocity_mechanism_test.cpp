// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// VelocityMechanism: the RPM spaces, the sign clamp, the atSpeed() dwell, the
// integral gate, and the three command factories. Everything runs on a
// mclib::time::ScopedClock at 10 ms per tick so every dwell is exact.
//
// The velocity source and the voltage sink are lambdas, so no hardware and no
// PROS symbol is involved. The numbers below are the ones the README's
// VelocityMechanism section quotes.

#include "mclib/command/command.h"
#include "mclib/mechanism/velocity_mechanism.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>

using mclib::mechanism::VelocityMechanism;
using mclib::mechanism::VelocityMechanismConfig;

namespace {

std::uint32_t g_fake_ms = 0;

constexpr double kTickMs = 10.0;

/// One mechanism plus the motor speed it reads and the volts it writes.
struct Rig {
  double motor_rpm = 0.0;
  double volts = 0.0;
  int writes = 0;
  VelocityMechanism mechanism;

  explicit Rig(const VelocityMechanismConfig& config)
      : mechanism([this]() { return motor_rpm; },
                  [this](double v) {
                    volts = v;
                    ++writes;
                  },
                  config) {}

  void tick() {
    g_fake_ms += static_cast<std::uint32_t>(kTickMs);
    mechanism.runPeriodic();
  }

  /// Ticks until atSpeed() first reads true. Returns the tick number, or -1.
  int ticksUntilAtSpeed(int max_ticks) {
    for (int i = 1; i <= max_ticks; ++i) {
      tick();
      if (mechanism.atSpeed()) {
        return i;
      }
    }
    return -1;
  }
};

/// The README's 1:2 worked example: blue motor, 600 free RPM, wheel at 2x.
VelocityMechanismConfig readmeConfig() {
  VelocityMechanismConfig config;
  config.ratio = 2.0;
  config.kv = 12.0 / 600.0;
  config.kp = 0.02;
  config.ki = 0.0;
  config.kd = 0.0;
  config.tolerance_rpm = 10.0;
  config.dwell_ms = 150.0;
  config.integral_range_rpm = 100.0;
  return config;
}

/// Ungeared, P only, no feedforward. The simplest loop to reason about.
VelocityMechanismConfig plainConfig() {
  VelocityMechanismConfig config;
  config.ratio = 1.0;
  config.kv = 0.0;
  config.kp = 0.02;
  config.ki = 0.0;
  config.kd = 0.0;
  config.tolerance_rpm = 10.0;
  config.dwell_ms = 150.0;
  return config;
}

// ---------------------------------------------------------------------------
// 1. Defaults, and nothing reaches the sink until periodic() runs.
// ---------------------------------------------------------------------------
void defaultsAndPeriodic() {
  std::printf("-- defaults, and periodic() is the only thing that writes\n");
  const VelocityMechanismConfig defaults;
  CHECK_EQ(defaults.kp, 0.02);
  CHECK_EQ(defaults.kv, 0.0);
  CHECK_EQ(defaults.ratio, 1.0);
  CHECK_EQ(defaults.tolerance_rpm, 100.0);
  CHECK_EQ(defaults.dwell_ms, 150.0);
  CHECK_EQ(defaults.integral_range_rpm, 200.0);
  CHECK_EQ(defaults.integral_max_volts, 2.0);
  CHECK_EQ(defaults.max_voltage, 12.0);

  Rig rig(plainConfig());
  CHECK_EQ(rig.mechanism.getTargetRpm(), 0.0);
  CHECK(!rig.mechanism.atSpeed());
  CHECK(!rig.mechanism.isSpinningUp());

  // setTargetRpm() only caches the state. The README says the mechanism has
  // to be registered with the scheduler or nothing moves.
  rig.mechanism.setTargetRpm(500.0);
  CHECK_EQ(rig.mechanism.getTargetRpm(), 500.0);
  CHECK(rig.mechanism.isSpinningUp());
  CHECK_EQ(static_cast<double>(rig.writes), 0.0);

  rig.tick();
  CHECK_EQ(static_cast<double>(rig.writes), 1.0);
  // Error 500, kp 0.02: 10 V.
  CHECK_NEAR(rig.volts, 10.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 2. A zero target coasts: 0 V, never a braking voltage.
// ---------------------------------------------------------------------------
void zeroTargetCoasts() {
  std::printf("-- zero target coasts\n");
  Rig rig(plainConfig());
  rig.motor_rpm = 400.0;  // still spinning from before
  rig.tick();
  CHECK_EQ(rig.volts, 0.0);
  CHECK(!rig.mechanism.atSpeed());
  CHECK(!rig.mechanism.isSpinningUp());

  // Spinning at target, then stop(): the output goes to 0, not negative.
  rig.mechanism.setTargetRpm(400.0);
  rig.tick();
  rig.mechanism.stop();
  CHECK_EQ(rig.mechanism.getTargetRpm(), 0.0);
  rig.tick();
  CHECK_EQ(rig.volts, 0.0);
  CHECK(!rig.mechanism.isSpinningUp());
}

// ---------------------------------------------------------------------------
// 3. The README's worked 1:2 example, number for number.
// ---------------------------------------------------------------------------
void readmeWorkedExample() {
  std::printf("-- README 1:2 example\n");
  Rig rig(readmeConfig());
  rig.mechanism.setTargetRpm(600.0);  // 600 wheel RPM = 300 motor RPM
  rig.motor_rpm = 280.0;

  CHECK_EQ(rig.mechanism.getCurrentRpm(), 560.0);
  CHECK_EQ(rig.mechanism.getCurrentMotorRpm(), 280.0);

  rig.tick();
  // P: 0.02 * (300 - 280) = 0.4 V. Feedforward: kv * 300 = 6.0 V, not kv * 600.
  std::printf("   motor 280 of 300, first tick: %.4f V (expected 6.4)\n", rig.volts);
  CHECK_NEAR(rig.volts, 6.4, 1e-12);
}

// ---------------------------------------------------------------------------
// 4. Output is clamped to [0, max] or [-max, 0] by the sign of the target.
// ---------------------------------------------------------------------------
void clampToTargetSign() {
  std::printf("-- sign clamp\n");
  VelocityMechanismConfig config = plainConfig();
  config.kv = 0.02;  // 12 V at 600 RPM
  Rig rig(config);

  // Standing start: 12 V feedforward plus 12 V of P is 24 V, capped at 12.
  rig.mechanism.setTargetRpm(600.0);
  rig.motor_rpm = 0.0;
  rig.tick();
  CHECK_NEAR(rig.volts, 12.0, 1e-12);

  // Twice the target: 12 V feedforward minus 24 V of P. Clamped to 0, the
  // mechanism is never driven backwards to brake.
  rig.motor_rpm = 1200.0;
  rig.tick();
  CHECK_EQ(rig.volts, 0.0);
  rig.motor_rpm = 5000.0;
  rig.tick();
  CHECK_EQ(rig.volts, 0.0);

  // Negative target: the clamp flips with it.
  rig.mechanism.setTargetRpm(-600.0);
  rig.motor_rpm = 0.0;
  rig.tick();
  CHECK_NEAR(rig.volts, -12.0, 1e-12);
  rig.motor_rpm = -5000.0;
  rig.tick();
  CHECK_EQ(rig.volts, 0.0);

  // max_voltage is honoured, not a hard-coded 12.
  VelocityMechanismConfig low = config;
  low.max_voltage = 5.0;
  Rig capped(low);
  capped.mechanism.setTargetRpm(600.0);
  capped.tick();
  CHECK_NEAR(capped.volts, 5.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 5. atSpeed() latches after dwell_ms inside tolerance, and drops instantly.
// ---------------------------------------------------------------------------
void atSpeedDwell() {
  std::printf("-- atSpeed dwell\n");
  Rig rig(plainConfig());
  rig.mechanism.setTargetRpm(500.0);
  rig.motor_rpm = 500.0;

  // In tolerance from tick 1 (t = 10 ms); 150 ms of dwell lands on tick 16.
  const int latched = rig.ticksUntilAtSpeed(40);
  std::printf("   dwell %.0f ms at %.0f ms/tick: latched on tick %d\n", 150.0, kTickMs, latched);
  CHECK_EQ(static_cast<double>(latched), 16.0);
  CHECK(!rig.mechanism.isSpinningUp());

  // Exactly on the tolerance edge still counts as inside.
  rig.motor_rpm = 490.0;
  rig.tick();
  CHECK(rig.mechanism.atSpeed());

  // One tick outside and it is gone, with no dwell on the way out. A flywheel
  // drained by a shot must report not-at-speed at once.
  rig.motor_rpm = 480.0;
  rig.tick();
  CHECK(!rig.mechanism.atSpeed());
  CHECK(rig.mechanism.isSpinningUp());

  // Coming back restarts the full dwell.
  rig.motor_rpm = 500.0;
  CHECK_EQ(static_cast<double>(rig.ticksUntilAtSpeed(40)), 16.0);

  // Flicker never accumulates a dwell.
  Rig flicker(plainConfig());
  flicker.mechanism.setTargetRpm(500.0);
  for (int i = 1; i <= 60; ++i) {
    flicker.motor_rpm = (i % 2 == 0) ? 500.0 : 400.0;
    flicker.tick();
  }
  CHECK(!flicker.mechanism.atSpeed());
}

// ---------------------------------------------------------------------------
// 6. tolerance_rpm is output RPM: a motor error under tolerance can still be
//    an output error over it.
// ---------------------------------------------------------------------------
void toleranceIsOutputSpace() {
  std::printf("-- tolerance judged in output RPM\n");
  Rig rig(readmeConfig());  // ratio 2, tolerance 10 wheel RPM
  rig.mechanism.setTargetRpm(600.0);

  // Motor 294 of 300: a motor error of 6 is a wheel error of 12. Not at speed,
  // ever, even though 6 < 10.
  rig.motor_rpm = 294.0;
  CHECK_EQ(static_cast<double>(rig.ticksUntilAtSpeed(60)), -1.0);

  // Motor 296: wheel error 8, inside. Latches on the usual tick 16.
  rig.motor_rpm = 296.0;
  CHECK_EQ(static_cast<double>(rig.ticksUntilAtSpeed(60)), 16.0);
}

// ---------------------------------------------------------------------------
// 7. A bad ratio falls back to 1.0 and getConfig() says so.
// ---------------------------------------------------------------------------
void badRatioFallsBackToOne() {
  std::printf("-- ratio validation\n");
  const double bad[] = {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()};
  for (const double ratio : bad) {
    VelocityMechanismConfig config = plainConfig();
    config.ratio = ratio;
    Rig rig(config);
    CHECK_EQ(rig.mechanism.getConfig().ratio, 1.0);

    // And it really runs ungeared: motor 500 is output 500.
    rig.mechanism.setTargetRpm(500.0);
    rig.motor_rpm = 500.0;
    CHECK_EQ(rig.mechanism.getCurrentRpm(), 500.0);
    rig.tick();
    CHECK_EQ(rig.volts, 0.0);
  }

  VelocityMechanismConfig good = plainConfig();
  good.ratio = 0.5;
  Rig rig(good);
  CHECK_EQ(rig.mechanism.getConfig().ratio, 0.5);
  rig.motor_rpm = 400.0;
  CHECK_EQ(rig.mechanism.getCurrentRpm(), 200.0);
}

// ---------------------------------------------------------------------------
// 8. The integral only accumulates inside integral_range_rpm (output RPM,
//    converted to motor RPM for the loop) and is capped at integral_max_volts.
// ---------------------------------------------------------------------------
void integralGateAndCap() {
  std::printf("-- integral gate and cap\n");
  VelocityMechanismConfig config = plainConfig();
  config.kp = 0.0;
  config.ki = 0.01;
  config.integral_range_rpm = 100.0;
  config.integral_max_volts = 2.0;
  Rig rig(config);
  rig.mechanism.setTargetRpm(500.0);

  // Error 300, outside the 100 RPM gate: the integral stays at 0 however long
  // the spin-up takes.
  rig.motor_rpm = 200.0;
  for (int i = 0; i < 50; ++i) {
    rig.tick();
  }
  CHECK_EQ(rig.volts, 0.0);

  // Error 50, inside: 0.5 V per tick, pinned at 2.0 V after four ticks.
  rig.motor_rpm = 450.0;
  const double expected[] = {0.5, 1.0, 1.5, 2.0, 2.0, 2.0};
  for (const double e : expected) {
    rig.tick();
    CHECK_NEAR(rig.volts, e, 1e-12);
  }

  // A new target resets the loop: the accumulated 2.0 V is gone, atSpeed is
  // false, and the next tick starts from one increment again.
  rig.mechanism.setTargetRpm(400.0);
  CHECK(!rig.mechanism.atSpeed());
  rig.motor_rpm = 350.0;
  rig.tick();
  CHECK_NEAR(rig.volts, 0.5, 1e-12);

  // Geared: integral_range_rpm is output RPM, so at ratio 2 the gate is 50
  // motor RPM. Motor error 60 (120 wheel) is outside; motor error 40 (80
  // wheel) is inside.
  VelocityMechanismConfig geared = config;
  geared.ratio = 2.0;
  Rig g(geared);
  g.mechanism.setTargetRpm(600.0);  // 300 motor
  g.motor_rpm = 240.0;
  for (int i = 0; i < 10; ++i) {
    g.tick();
  }
  CHECK_EQ(g.volts, 0.0);
  g.motor_rpm = 260.0;
  g.tick();
  CHECK_NEAR(g.volts, 0.01 * 40.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 9. Command factories, driven by hand.
// ---------------------------------------------------------------------------
void commands() {
  std::printf("-- command factories\n");
  Rig rig(plainConfig());

  // makeSpinUpCommand finishes once atSpeed() latches and leaves it spinning.
  {
    std::unique_ptr<Command> spin_up = rig.mechanism.makeSpinUpCommand(500.0);
    const std::vector<Subsystem*> requirements = spin_up->getRequirements();
    CHECK_EQ(static_cast<double>(requirements.size()), 1.0);
    CHECK(!requirements.empty() && requirements.front() == &rig.mechanism);

    spin_up->initialize();
    CHECK_EQ(rig.mechanism.getTargetRpm(), 500.0);
    CHECK(!spin_up->isFinished());

    rig.motor_rpm = 500.0;
    int finished_on = -1;
    for (int i = 1; i <= 40; ++i) {
      spin_up->execute();
      rig.tick();
      if (spin_up->isFinished()) {
        finished_on = i;
        break;
      }
    }
    CHECK_EQ(static_cast<double>(finished_on), 16.0);
    spin_up->end(false);
    CHECK_EQ(rig.mechanism.getTargetRpm(), 500.0);
  }

  // With a timeout it finishes at the timeout even if never at speed.
  {
    std::unique_ptr<Command> spin_up = rig.mechanism.makeSpinUpCommand(500.0, 100.0);
    rig.motor_rpm = 0.0;
    spin_up->initialize();
    int finished_on = -1;
    for (int i = 1; i <= 40; ++i) {
      spin_up->execute();
      rig.tick();
      if (spin_up->isFinished()) {
        finished_on = i;
        break;
      }
    }
    CHECK_EQ(static_cast<double>(finished_on), 10.0);
    CHECK(!rig.mechanism.atSpeed());
  }

  // makeSpinCommand never finishes.
  {
    std::unique_ptr<Command> spin = rig.mechanism.makeSpinCommand(300.0);
    spin->initialize();
    for (int i = 0; i < 5; ++i) {
      spin->execute();
      rig.tick();
      CHECK(!spin->isFinished());
    }
    CHECK_EQ(rig.mechanism.getTargetRpm(), 300.0);
  }

  // makeStopCommand is a run(), not a runOnce(): it holds the requirement so
  // a spin default cannot take the subsystem straight back.
  {
    std::unique_ptr<Command> stop = rig.mechanism.makeStopCommand();
    stop->initialize();
    stop->execute();
    CHECK_EQ(rig.mechanism.getTargetRpm(), 0.0);
    CHECK(!stop->isFinished());
    rig.tick();
    CHECK_EQ(rig.volts, 0.0);
  }
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  defaultsAndPeriodic();
  zeroTargetCoasts();
  readmeWorkedExample();
  clampToTargetSign();
  atSpeedDwell();
  toleranceIsOutputSpace();
  badRatioFallsBackToOne();
  integralGateAndCap();
  commands();
  return mclib::test::summary("velocity_mechanism");
}
