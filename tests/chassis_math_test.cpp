// mclib
/**
 * @file chassis_math_test.cpp
 * @brief The arithmetic lifted out of Chassis and ChassisController.
 *
 * Covers the three bugs the arithmetic carried: a heading target compared
 * against unbounded rotation, a saturating drive term that killed the heading
 * differential, and an arcade mix that turned the wrong way.
 */
#include "mclib/chassis/chassis_math.hpp"
#include "test_assert.hpp"

#include <cmath>

using mclib::chassis_math::arcadeMix;
using mclib::chassis_math::DrivePair;
using mclib::chassis_math::mixDriveCorrection;
using mclib::chassis_math::normalizeHeadingTarget;

namespace {

/// @brief A target within half a turn of the robot is left exactly alone.
void testNormalizeLeavesNearTargetsAlone() {
  CHECK_NEAR(normalizeHeadingTarget(90.0, 0.0), 90.0, 1e-12);
  CHECK_NEAR(normalizeHeadingTarget(-90.0, 0.0), -90.0, 1e-12);
  CHECK_NEAR(normalizeHeadingTarget(0.0, 0.0), 0.0, 1e-12);
  // Exactly half a turn stays put, on either side. Same as normalizeTarget().
  CHECK_NEAR(normalizeHeadingTarget(180.0, 0.0), 180.0, 1e-12);
  CHECK_NEAR(normalizeHeadingTarget(-180.0, 0.0), -180.0, 1e-12);
}

/**
 * @brief The reported bug: at 270 deg, turnToHeading(0) must not unwind 270.
 */
void testNormalizeTakesTheShortWay() {
  // Sitting at 270, "go to 0" is a quarter turn clockwise, to 360.
  CHECK_NEAR(normalizeHeadingTarget(0.0, 270.0), 360.0, 1e-12);
  // After two full turns, "go to 0" is still a quarter turn, not 720.
  CHECK_NEAR(normalizeHeadingTarget(0.0, 720.0 + 270.0), 1080.0, 1e-12);
  // And the same going the other way round.
  CHECK_NEAR(normalizeHeadingTarget(0.0, -270.0), -360.0, 1e-12);
  CHECK_NEAR(normalizeHeadingTarget(90.0, -725.0), -630.0, 1e-12);
}

/**
 * @brief The invariant, not just the examples: whatever the accumulated
 *        rotation, a target 90 deg away is reached by turning 90 deg.
 */
void testNormalizeNeverTurnsMoreThanHalfATurn() {
  for (int turns = -5; turns <= 5; ++turns) {
    for (int target = -360; target <= 360; target += 15) {
      const double current = turns * 360.0 + 17.5;
      const double normalized =
          normalizeHeadingTarget(static_cast<double>(target), current);
      const double error = normalized - current;
      CHECK(std::fabs(error) <= 180.0 + 1e-9);
      // Same heading, just a different branch: the shift is whole turns.
      const double shift = normalized - static_cast<double>(target);
      CHECK_NEAR(shift / 360.0, std::round(shift / 360.0), 1e-9);
    }
  }
}

/// @brief A pair that already fits the rail passes through untouched.
void testMixLeavesUnsaturatedPairsAlone() {
  const DrivePair pair = mixDriveCorrection(6.0, 1.5, 12.0);
  CHECK_NEAR(pair.left, 7.5, 1e-12);
  CHECK_NEAR(pair.right, 4.5, 1e-12);
}

/**
 * @brief The reported bug, with the numbers from the review: a 40 in move at
 *        kp = 0.4 with a 5 deg heading error at heading kp = 0.3.
 *
 * The old code clamped each side on its own and got 12 V / 12 V - a
 * differential of zero, with the heading loop asking for 3 V of it.
 */
void testMixKeepsDifferentialWhenDriveSaturates() {
  const double drive = 12.0;       // 0.4 * 40 in = 16 V, clamped to the rail.
  const double correction = 1.5;   // 0.3 * 5 deg.
  const DrivePair pair = mixDriveCorrection(drive, correction, 12.0);

  CHECK_NEAR(pair.left, 12.0, 1e-12);
  CHECK_NEAR(pair.right, 9.0, 1e-12);
  CHECK_NEAR(pair.left - pair.right, 2.0 * correction, 1e-12);
}

/**
 * @brief The invariant behind that example: a saturating drive term still
 *        yields a non-zero left - right whenever the correction is non-zero.
 */
void testMixAlwaysPreservesNonZeroDifferential() {
  const double max_volts = 12.0;
  for (double drive = -24.0; drive <= 24.0; drive += 0.5) {
    // Up to the half-rail ceiling, which is where the correction is passed
    // through untouched.
    for (double correction = -6.0; correction <= 6.0; correction += 0.5) {
      const DrivePair pair = mixDriveCorrection(drive, correction, max_volts);

      CHECK(std::fabs(pair.left) <= max_volts + 1e-9);
      CHECK(std::fabs(pair.right) <= max_volts + 1e-9);
      // The differential the heading loop asked for survives in full.
      CHECK_NEAR(pair.left - pair.right, 2.0 * correction, 1e-9);
      if (std::fabs(correction) > 1e-9) {
        CHECK(std::fabs(pair.left - pair.right) > 1e-9);
      }
    }
  }
}

/**
 * @brief A huge correction is capped at half the rail, so the move still
 *        makes forward progress instead of spinning on the spot.
 */
void testMixCapsCorrectionAtHalfTheRail() {
  const DrivePair pair = mixDriveCorrection(12.0, 20.0, 12.0);
  CHECK_NEAR(pair.left, 12.0, 1e-12);
  CHECK_NEAR(pair.right, 0.0, 1e-12);
  // Hard over: the differential is the whole rail.
  CHECK_NEAR(pair.left - pair.right, 12.0, 1e-12);
  // And the mean is still driving, not stalled.
  CHECK(pair.left + pair.right > 0.0);

  const DrivePair reverse = mixDriveCorrection(-12.0, -20.0, 12.0);
  CHECK_NEAR(reverse.left, -12.0, 1e-12);
  CHECK_NEAR(reverse.right, 0.0, 1e-12);
  CHECK_NEAR(reverse.left - reverse.right, -12.0, 1e-12);
  CHECK(reverse.left + reverse.right < 0.0);
}

/**
 * @brief The slow-approach case from review: 3 V cap, 10 deg of error.
 *
 * Uncapped, the correction alone fills the rail and the pair becomes
 * {+3, -3}: a turn in place that travels zero inches for the whole timeout.
 */
void testMixStillCreepsForwardOnASlowApproach() {
  const DrivePair pair = mixDriveCorrection(3.0, 0.3 * 10.0, 3.0);
  CHECK_NEAR(pair.left, 3.0, 1e-12);
  CHECK_NEAR(pair.right, 0.0, 1e-12);
  CHECK(pair.left + pair.right > 0.0);
}

/// @brief A cap of zero or less means no command at all, not a NaN.
void testMixRejectsNonPositiveCap() {
  const DrivePair zero = mixDriveCorrection(8.0, 2.0, 0.0);
  CHECK_EQ(zero.left, 0.0);
  CHECK_EQ(zero.right, 0.0);
}

/**
 * @brief The reported bug: a positive turn must rotate the same way a positive
 *        heading delta does.
 *
 * `runTurnToHeading()` drives `tankVoltage(output, -output)` for a positive
 * error, so left leads on a clockwise turn. Arcade has to agree.
 */
void testArcadeTurnsTheSameWayAsATurnGoal() {
  const DrivePair turn_only = arcadeMix(0.0, 0.5);
  CHECK(turn_only.left > 0.0);
  CHECK(turn_only.right < 0.0);
  CHECK_NEAR(turn_only.left, 0.5, 1e-12);
  CHECK_NEAR(turn_only.right, -0.5, 1e-12);

  // Same sign relationship as the turn loop's own command pair.
  const DrivePair drive_goal = mixDriveCorrection(0.0, 3.0, 12.0);
  CHECK((turn_only.left - turn_only.right > 0.0) ==
        (drive_goal.left - drive_goal.right > 0.0));
}

/// @brief Forward is forward, and the mix is a plain sum.
void testArcadeForwardAndCombined() {
  const DrivePair forward = arcadeMix(0.7, 0.0);
  CHECK_NEAR(forward.left, 0.7, 1e-12);
  CHECK_NEAR(forward.right, 0.7, 1e-12);

  const DrivePair combined = arcadeMix(0.6, 0.25);
  CHECK_NEAR(combined.left, 0.85, 1e-12);
  CHECK_NEAR(combined.right, 0.35, 1e-12);
  CHECK_NEAR((combined.left + combined.right) * 0.5, 0.6, 1e-12);
}

}  // namespace

int main() {
  testNormalizeLeavesNearTargetsAlone();
  testNormalizeTakesTheShortWay();
  testNormalizeNeverTurnsMoreThanHalfATurn();
  testMixLeavesUnsaturatedPairsAlone();
  testMixKeepsDifferentialWhenDriveSaturates();
  testMixAlwaysPreservesNonZeroDifferential();
  testMixCapsCorrectionAtHalfTheRail();
  testMixStillCreepsForwardOnASlowApproach();
  testMixRejectsNonPositiveCap();
  testArcadeTurnsTheSameWayAsATurnGoal();
  testArcadeForwardAndCombined();
  return mclib::test::summary("chassis_math");
}
