// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file drive_curve_test.cpp
 * @brief Joystick shaping: deadzone, exponential curve, minimum output, slew.
 */
#include "mclib/control/drive_curve.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <limits>

using mclib::control::DriveCurve;
using mclib::control::DriveCurveConfig;
using mclib::control::shapeDriveInput;

namespace {

/// @brief Inside the deadzone is exactly 0; the output rises from 0 at the edge.
void testDeadzoneIsContinuousAtItsEdge() {
  DriveCurveConfig config;
  config.deadzone = 0.1;

  CHECK_EQ(shapeDriveInput(0.0, config), 0.0);
  CHECK_EQ(shapeDriveInput(0.05, config), 0.0);
  CHECK_EQ(shapeDriveInput(-0.099, config), 0.0);
  CHECK_EQ(shapeDriveInput(0.1, config), 0.0);

  // Just past the edge is just past zero: no jump.
  CHECK_NEAR(shapeDriveInput(0.1 + 1e-6, config), 0.0, 1e-5);
  CHECK_NEAR(shapeDriveInput(-0.1 - 1e-6, config), 0.0, 1e-5);

  // The rest of the range is stretched so full stick is still full power.
  CHECK_NEAR(shapeDriveInput(1.0, config), 1.0, 1e-12);
  CHECK_NEAR(shapeDriveInput(0.55, config), 0.5, 1e-12);
}

/// @brief With no gain, no deadzone and no floor the output is the input.
void testZeroGainIsLinear() {
  DriveCurveConfig config;
  config.deadzone = 0.0;
  config.gain = 0.0;
  for (double x = -1.0; x <= 1.0 + 1e-9; x += 0.05) {
    CHECK_NEAR(shapeDriveInput(x, config), x, 1e-12);
  }
}

/// @brief The published formula, at a point where every term matters.
void testGainMatchesTheFormula() {
  DriveCurveConfig config;
  config.deadzone = 0.0;
  config.gain = 10.0;
  const double x = 0.5;
  const double base = std::exp(-1.0);
  const double expected = (base + std::exp((x - 1.0) * 1.0) * (1.0 - base)) * x;
  CHECK_NEAR(shapeDriveInput(x, config), expected, 1e-12);
  // Near the centre the curve sits under the line: a quarter stick gives
  // (e^-1 + e^-0.75 (1 - e^-1)) / 4 = 0.167, not 0.25.
  CHECK_NEAR(shapeDriveInput(0.25, config), 0.1666, 1e-3);
  // And the end point is pinned at 1 for every gain.
  for (double gain : {0.0, 1.0, 5.0, 10.0, 20.0, 50.0}) {
    config.gain = gain;
    CHECK_NEAR(shapeDriveInput(1.0, config), 1.0, 1e-12);
    CHECK_NEAR(shapeDriveInput(-1.0, config), -1.0, 1e-12);
  }
}

/// @brief Odd in the input and never decreasing, whatever the gain.
void testCurveIsOddAndMonotonic() {
  for (double gain : {0.0, 3.0, 10.0, 25.0}) {
    for (double deadzone : {0.0, 0.05}) {
      DriveCurveConfig config;
      config.gain = gain;
      config.deadzone = deadzone;
      double previous = -1.0 - 1e-9;
      for (double x = -1.0; x <= 1.0 + 1e-9; x += 0.01) {
        const double out = shapeDriveInput(x, config);
        CHECK_NEAR(shapeDriveInput(-x, config), -out, 1e-12);
        CHECK(out >= previous - 1e-12);
        CHECK(std::fabs(out) <= std::fabs(x) + 1e-12);
        previous = out;
      }
    }
  }
}

/// @brief Anything outside the deadzone commands at least min_output.
void testMinOutputLiftsSmallCommands() {
  DriveCurveConfig config;
  config.deadzone = 0.05;
  config.min_output = 0.2;

  CHECK_EQ(shapeDriveInput(0.0, config), 0.0);
  CHECK_EQ(shapeDriveInput(0.04, config), 0.0);
  CHECK_NEAR(shapeDriveInput(0.06, config), 0.2, 0.01);
  CHECK_NEAR(shapeDriveInput(-0.06, config), -0.2, 0.01);
  CHECK_NEAR(shapeDriveInput(1.0, config), 1.0, 1e-12);

  // Still monotonic: the floor is a rescale, not a clamp.
  double previous = 0.0;
  for (double x = 0.06; x <= 1.0 + 1e-9; x += 0.01) {
    const double out = shapeDriveInput(x, config);
    CHECK(out >= 0.2 - 1e-12);
    CHECK(out >= previous - 1e-12);
    previous = out;
  }
}

/// @brief Inputs past full stick, and NaN, are contained.
void testOutputIsClamped() {
  DriveCurveConfig config;
  config.deadzone = 0.0;
  CHECK_EQ(shapeDriveInput(2.0, config), 1.0);
  CHECK_EQ(shapeDriveInput(-5.0, config), -1.0);
  CHECK_EQ(shapeDriveInput(std::numeric_limits<double>::quiet_NaN(), config), 0.0);
  CHECK_EQ(shapeDriveInput(std::numeric_limits<double>::infinity(), config), 1.0);

  config.gain = 10.0;
  config.min_output = 0.3;
  for (double x = -3.0; x <= 3.0; x += 0.1) {
    CHECK(std::fabs(shapeDriveInput(x, config)) <= 1.0);
  }
}

/// @brief Nonsense parameters mean "off", not undefined behaviour.
void testBadParametersAreOff() {
  DriveCurveConfig config;
  config.deadzone = 1.5;   // would swallow everything
  config.gain = -10.0;     // would bend the curve back on itself
  config.min_output = -1;  // would flip the sign
  for (double x = -1.0; x <= 1.0 + 1e-9; x += 0.25) {
    CHECK_NEAR(shapeDriveInput(x, config), x, 1e-12);
  }
}

/// @brief The output moves toward the shaped value by at most slew per call.
void testSlewLimitsTheChangePerTick() {
  DriveCurveConfig config;
  config.deadzone = 0.0;
  config.slew_per_tick = 0.25;
  DriveCurve curve(config);

  CHECK_NEAR(curve.apply(1.0), 0.25, 1e-12);
  CHECK_NEAR(curve.apply(1.0), 0.5, 1e-12);
  CHECK_NEAR(curve.apply(1.0), 0.75, 1e-12);
  CHECK_NEAR(curve.apply(1.0), 1.0, 1e-12);
  // At the target it stays there rather than overshooting.
  CHECK_NEAR(curve.apply(1.0), 1.0, 1e-12);
  // Reversal is limited the same way.
  CHECK_NEAR(curve.apply(-1.0), 0.75, 1e-12);
  // A small change inside the limit passes straight through.
  CHECK_NEAR(curve.apply(0.7), 0.7, 1e-12);

  curve.reset();
  CHECK_NEAR(curve.apply(-1.0), -0.25, 1e-12);
}

/// @brief Slew off: apply() is shapeDriveInput() with no memory.
void testNoSlewIsStateless() {
  DriveCurveConfig config;
  config.deadzone = 0.0;
  config.gain = 10.0;
  DriveCurve curve(config);
  CHECK_NEAR(curve.apply(1.0), 1.0, 1e-12);
  CHECK_NEAR(curve.apply(-1.0), -1.0, 1e-12);
  CHECK_NEAR(curve.apply(0.5), shapeDriveInput(0.5, config), 1e-12);
}

/// @brief setConfig() takes effect on the next call; config() reads it back.
void testConfigRoundTrips() {
  DriveCurve curve;
  CHECK_NEAR(curve.config().deadzone, 0.05, 1e-12);
  DriveCurveConfig config;
  config.deadzone = 0.0;
  config.slew_per_tick = 0.5;
  curve.setConfig(config);
  CHECK_NEAR(curve.config().slew_per_tick, 0.5, 1e-12);
  CHECK_NEAR(curve.apply(1.0), 0.5, 1e-12);
}

}  // namespace

int main() {
  testDeadzoneIsContinuousAtItsEdge();
  testZeroGainIsLinear();
  testGainMatchesTheFormula();
  testCurveIsOddAndMonotonic();
  testMinOutputLiftsSmallCommands();
  testOutputIsClamped();
  testBadParametersAreOff();
  testSlewLimitsTheChangePerTick();
  testNoSlewIsStateless();
  testConfigRoundTrips();
  return mclib::test::summary("drive_curve_test");
}
