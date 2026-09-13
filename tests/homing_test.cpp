// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Stop detection in HomingMechanism, driven by synthetic current and velocity
// traces on a mclib::time::ScopedClock so every dwell window is exact.
//
// The case that matters: a loaded mechanism that is still accelerating draws
// inrush current well past startup_grace_ms. A current detector with no dwell
// reads that as the hard stop and zeroes the position sensor somewhere in the
// middle of the travel.

#include "mclib/mechanism/homing_mechanism.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#include <cstdint>
#include <cstdio>

using mclib::mechanism::HomingMechanism;
using mclib::mechanism::HomingMechanismConfig;

namespace {

std::uint32_t g_fake_ms = 0;

constexpr double kTickMs = 10.0;

HomingMechanismConfig baseConfig() {
  HomingMechanismConfig config;
  config.homing_voltage = -3.0;
  config.current_threshold_amps = 2.0;
  config.current_dwell_ms = 100.0;
  config.velocity_threshold_rpm = 5.0;
  config.stall_dwell_ms = 150.0;
  config.startup_grace_ms = 150.0;
  config.timeout_ms = 5000.0;
  config.backoff_voltage = 0.0;
  config.backoff_ms = 0.0;
  return config;
}

/// One mechanism plus the signals it reads and the zeroing it fires.
struct Rig {
  double amps = 0.0;
  double rpm = 0.0;
  double volts = 0.0;
  int zeroed = 0;
  int zeroed_tick = -1;
  int tick_count = 0;
  HomingMechanism mechanism;

  explicit Rig(const HomingMechanismConfig& config)
      : mechanism(config, [this](double v) { volts = v; }) {
    mechanism.setCurrentSource([this]() { return amps; });
    mechanism.setVelocitySource([this]() { return rpm; });
    mechanism.setPositionReset([this]() {
      ++zeroed;
      if (zeroed_tick < 0) {
        zeroed_tick = tick_count;
      }
    });
  }

  void tick() {
    ++tick_count;
    g_fake_ms += static_cast<std::uint32_t>(kTickMs);
    mechanism.runPeriodic();
  }
};

// ---------------------------------------------------------------------------
// 1. A startup current spike on a mechanism that is accelerating must not latch.
// ---------------------------------------------------------------------------
void inrushDoesNotLatch() {
  std::printf("-- inrush current spike\n");
  Rig rig(baseConfig());
  rig.mechanism.startHoming();

  // 60 ticks = 600 ms. The arm is loaded: it draws 4.5 A while it accelerates
  // for the first 20 ticks (200 ms, past the 150 ms grace), reaching speed
  // at tick 8, then settles to 1.0 A once it is moving freely. Nothing in this
  // trace is a hard stop.
  for (int i = 1; i <= 60; ++i) {
    rig.amps = i <= 20 ? 4.5 : 1.0;
    rig.rpm = i <= 7 ? 1.0 : 60.0;
    rig.tick();
    if (rig.zeroed > 0) {
      break;
    }
  }
  std::printf("   spike ticks 1-20 at 4.5 A (threshold 2.0 A, dwell %.0f ms)\n",
              rig.mechanism.getConfig().current_dwell_ms);
  std::printf("   latched at tick %d (-1 = never), state=%s\n", rig.zeroed_tick,
              rig.mechanism.isHomed() ? "Homed" : "Seeking");
  CHECK_EQ(static_cast<double>(rig.zeroed), 0.0);
  CHECK(!rig.mechanism.isHomed());

  // Without the dwell the same trace latches on the first tick past the grace
  // window: tick 15, at 150 ms, with the arm running at 60 rpm.
  HomingMechanismConfig no_dwell = baseConfig();
  no_dwell.current_dwell_ms = 0.0;
  Rig old(no_dwell);
  old.mechanism.startHoming();
  for (int i = 1; i <= 60; ++i) {
    old.amps = i <= 20 ? 4.5 : 1.0;
    old.rpm = i <= 7 ? 1.0 : 60.0;
    old.tick();
    if (old.zeroed > 0) {
      break;
    }
  }
  std::printf("   same trace with current_dwell_ms = 0: latched at tick %d\n",
              old.zeroed_tick);
  CHECK_EQ(static_cast<double>(old.zeroed_tick), 15.0);
}

// ---------------------------------------------------------------------------
// 2. A real hard stop still latches, one dwell after the current comes up.
// ---------------------------------------------------------------------------
void hardStopStillLatches() {
  std::printf("-- real hard stop\n");
  Rig rig(baseConfig());
  rig.mechanism.startHoming();

  // Moving freely at 1.0 A for 40 ticks, then into the stop: current pins at
  // 6 A and stays there. Velocity is held above the stall threshold so the
  // current detector is the only one that can fire.
  for (int i = 1; i <= 120; ++i) {
    rig.amps = i <= 40 ? 1.0 : 6.0;
    rig.rpm = 60.0;
    rig.tick();
    if (rig.zeroed > 0) {
      break;
    }
  }
  std::printf("   stop at tick 41, latched at tick %d (dwell = %.0f ms = 10 ticks)\n",
              rig.zeroed_tick, rig.mechanism.getConfig().current_dwell_ms);
  CHECK_EQ(static_cast<double>(rig.zeroed), 1.0);
  // Over-threshold first seen on tick 41; 100 ms of dwell is 10 ticks later.
  CHECK_EQ(static_cast<double>(rig.zeroed_tick), 51.0);
  CHECK(rig.mechanism.isHomed());
  CHECK_EQ(rig.volts, 0.0);
}

// ---------------------------------------------------------------------------
// 3. Current that flickers over the threshold never accumulates a dwell.
// ---------------------------------------------------------------------------
void flickerResetsTheDwell() {
  std::printf("-- flickering current\n");
  Rig rig(baseConfig());
  rig.mechanism.startHoming();

  // Alternating 6 A / 1 A: the over-current is never sustained for two
  // consecutive ticks, let alone ten.
  for (int i = 1; i <= 200; ++i) {
    rig.amps = (i % 2 == 0) ? 6.0 : 1.0;
    rig.rpm = 60.0;
    rig.tick();
    if (rig.zeroed > 0) {
      break;
    }
  }
  std::printf("   200 ticks of alternating 6.0/1.0 A: latched at tick %d (-1 = never)\n",
              rig.zeroed_tick);
  CHECK_EQ(static_cast<double>(rig.zeroed), 0.0);
}

// ---------------------------------------------------------------------------
// 4. The stall detector is untouched: it still fires after stall_dwell_ms.
// ---------------------------------------------------------------------------
void stallDetectorUnchanged() {
  std::printf("-- velocity stall\n");
  HomingMechanismConfig config = baseConfig();
  config.current_threshold_amps = 0.0;  // Current detector off.
  Rig rig(config);
  rig.mechanism.startHoming();

  // Moves for 30 ticks, then stops dead.
  for (int i = 1; i <= 120; ++i) {
    rig.amps = 0.0;
    rig.rpm = i <= 30 ? 60.0 : 0.0;
    rig.tick();
    if (rig.zeroed > 0) {
      break;
    }
  }
  std::printf("   stall from tick 31, latched at tick %d (dwell = %.0f ms)\n",
              rig.zeroed_tick, config.stall_dwell_ms);
  CHECK_EQ(static_cast<double>(rig.zeroed), 1.0);
  // Stall first seen on tick 31; 150 ms of dwell is 15 ticks later.
  CHECK_EQ(static_cast<double>(rig.zeroed_tick), 46.0);
}

// ---------------------------------------------------------------------------
// 5. A limit switch still short-circuits everything, dwell included.
// ---------------------------------------------------------------------------
void limitSwitchIsImmediate() {
  std::printf("-- limit switch\n");
  Rig rig(baseConfig());
  bool pressed = false;
  rig.mechanism.setLimitSwitch([&pressed]() { return pressed; });
  rig.mechanism.startHoming();

  for (int i = 1; i <= 40; ++i) {
    rig.amps = 0.0;
    rig.rpm = 60.0;
    pressed = (i >= 12);
    rig.tick();
    if (rig.zeroed > 0) {
      break;
    }
  }
  std::printf("   pressed from tick 12, latched at tick %d\n", rig.zeroed_tick);
  CHECK_EQ(static_cast<double>(rig.zeroed_tick), 12.0);
  CHECK(rig.mechanism.isHomed());
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock([]() { return g_fake_ms; });
  inrushDoesNotLatch();
  hardStopStillLatches();
  flickerResetsTheDwell();
  stallDetectorUnchanged();
  limitSwitchIsImmediate();
  return mclib::test::summary("homing");
}
