// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file curvature_mix_test.cpp
 * @brief The curvature ("cheesy drive") mix in chassis_math.
 */
#include "mclib/chassis/chassis_math.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <limits>

using mclib::chassis_math::curvatureMix;
using mclib::chassis_math::DrivePair;

namespace {

/// @brief No turn: both sides get the forward command, whatever its sign.
void testPureForwardIsStraight() {
  for (double forward : {1.0, 0.5, -0.3, -1.0}) {
    const DrivePair pair = curvatureMix(forward, 0.0);
    CHECK_NEAR(pair.left, forward, 1e-12);
    CHECK_NEAR(pair.right, forward, 1e-12);
  }
}

/// @brief Positive turn leads with the left side: clockwise, like arcadeMix.
void testPositiveTurnIsClockwise() {
  const DrivePair pair = curvatureMix(0.5, 0.5);
  CHECK_NEAR(pair.left, 0.75, 1e-12);
  CHECK_NEAR(pair.right, 0.25, 1e-12);
  CHECK(pair.left > pair.right);
}

/// @brief The differential scales with |forward|: same stick, tighter at low speed.
void testTurnScalesWithSpeed() {
  // Both unsaturated: 0.6 * 1.5 = 0.9 on the leading side.
  const DrivePair slow = curvatureMix(0.2, 0.5);
  const DrivePair fast = curvatureMix(0.6, 0.5);
  CHECK_NEAR(slow.left - slow.right, 0.2, 1e-12);
  CHECK_NEAR(fast.left - fast.right, 0.6, 1e-12);
  // Curvature is the ratio of differential to speed, and that is constant.
  CHECK_NEAR((slow.left - slow.right) / 0.2, (fast.left - fast.right) / 0.6, 1e-12);
}

/// @brief In reverse a positive turn still rotates clockwise (|forward|, not forward).
void testReverseKeepsTurnDirection() {
  const DrivePair pair = curvatureMix(-0.5, 0.5);
  CHECK_NEAR(pair.left, -0.25, 1e-12);
  CHECK_NEAR(pair.right, -0.75, 1e-12);
  CHECK(pair.left > pair.right);
}

/// @brief Saturation scales both sides together, keeping the ratio.
void testSaturationPreservesRatio() {
  const DrivePair pair = curvatureMix(1.0, 1.0);
  CHECK_NEAR(pair.left, 1.0, 1e-12);
  CHECK_NEAR(pair.right, 0.0, 1e-12);

  const DrivePair half = curvatureMix(1.0, 0.5);  // raw 1.5 / 0.5
  CHECK_NEAR(half.left, 1.0, 1e-12);
  CHECK_NEAR(half.right, 1.0 / 3.0, 1e-12);

  const DrivePair reverse = curvatureMix(-1.0, -1.0);  // raw -2 / 0
  CHECK_NEAR(reverse.left, -1.0, 1e-12);
  CHECK_NEAR(reverse.right, 0.0, 1e-12);
}

/// @brief Every output fits the rail, and unsaturated pairs pass through as is.
void testOutputsFitTheRail() {
  for (double forward = -1.0; forward <= 1.0 + 1e-9; forward += 0.1) {
    for (double turn = -1.0; turn <= 1.0 + 1e-9; turn += 0.1) {
      const DrivePair pair = curvatureMix(forward, turn);
      CHECK(std::fabs(pair.left) <= 1.0 + 1e-12);
      CHECK(std::fabs(pair.right) <= 1.0 + 1e-12);
      if (forward != 0.0) {
        const double raw_left = forward + std::fabs(forward) * turn;
        const double raw_right = forward - std::fabs(forward) * turn;
        if (std::fabs(raw_left) <= 1.0 && std::fabs(raw_right) <= 1.0) {
          CHECK_NEAR(pair.left, raw_left, 1e-12);
          CHECK_NEAR(pair.right, raw_right, 1e-12);
        } else if (std::fabs(raw_right) > 1e-9) {
          CHECK_NEAR(pair.left / pair.right, raw_left / raw_right, 1e-9);
        }
      }
    }
  }
}

/// @brief Stopped: spin in place by default, or do nothing when told not to.
void testStoppedTurnsInPlaceOrNot() {
  const DrivePair spin = curvatureMix(0.0, 0.6);
  CHECK_NEAR(spin.left, 0.6, 1e-12);
  CHECK_NEAR(spin.right, -0.6, 1e-12);

  const DrivePair spin_ccw = curvatureMix(0.0, -1.0);
  CHECK_NEAR(spin_ccw.left, -1.0, 1e-12);
  CHECK_NEAR(spin_ccw.right, 1.0, 1e-12);

  const DrivePair none = curvatureMix(0.0, 0.6, false);
  CHECK_EQ(none.left, 0.0);
  CHECK_EQ(none.right, 0.0);

  // An oversized turn stick in place is clamped, not passed through.
  const DrivePair big = curvatureMix(0.0, 3.0);
  CHECK_NEAR(big.left, 1.0, 1e-12);
  CHECK_NEAR(big.right, -1.0, 1e-12);
}

/// @brief NaN in is zero out, not NaN written to a motor.
void testNaNIsZero() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const DrivePair a = curvatureMix(nan, 0.5);
  const DrivePair b = curvatureMix(0.5, nan);
  CHECK_EQ(a.left, 0.0);
  CHECK_EQ(a.right, 0.0);
  CHECK_EQ(b.left, 0.0);
  CHECK_EQ(b.right, 0.0);
}

}  // namespace

int main() {
  testPureForwardIsStraight();
  testPositiveTurnIsClockwise();
  testTurnScalesWithSpeed();
  testReverseKeepsTurnDirection();
  testSaturationPreservesRatio();
  testOutputsFitTheRail();
  testStoppedTurnsInPlaceOrNot();
  testNaNIsZero();
  return mclib::test::summary("curvature_mix_test");
}
