// mclib
#include "mclib/control/scaling.hpp"

#include <cmath>

#include "test_assert.hpp"

namespace {

constexpr double kEps = 1e-12;

/** @brief Check that left/right keeps the ratio it had before scaling. */
void checkRatioPreserved(double left_before, double right_before,
                         double left_after, double right_after) {
  // left_after * right_before == right_after * left_before, cross-multiplied
  // so a zero on either side does not divide.
  CHECK_NEAR(left_after * right_before, right_after * left_before, 1e-9);
}

void testScaleToMinBoostsBothSides() {
  double left = 0.5;
  double right = 1.0;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, 2.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);
  checkRatioPreserved(0.5, 1.0, left, right);

  // The larger-magnitude side being the small one works too.
  left = 1.0;
  right = 0.5;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, 2.0, kEps);
  checkRatioPreserved(1.0, 0.5, left, right);
}

void testScaleToMinNegative() {
  double left = -0.5;
  double right = -1.0;
  scaleToMin(left, right, 2.0);
  // min_output is a magnitude; negative commands are pushed to -min.
  CHECK_NEAR(left, -2.0, kEps);
  CHECK_NEAR(right, -4.0, kEps);
  checkRatioPreserved(-0.5, -1.0, left, right);

  left = -1.0;
  right = -0.5;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, -4.0, kEps);
  CHECK_NEAR(right, -2.0, kEps);
}

void testScaleToMinEqualMagnitudes() {
  double left = 0.5;
  double right = 0.5;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, 2.0, kEps);
  CHECK_NEAR(right, 2.0, kEps);

  left = -0.5;
  right = -0.5;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, -2.0, kEps);
  CHECK_NEAR(right, -2.0, kEps);
}

void testScaleToMinMixedSigns() {
  // A point turn: sides oppose. The ratio (and therefore the turn) survives.
  double left = -0.5;
  double right = 1.0;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, -2.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);
  checkRatioPreserved(-0.5, 1.0, left, right);
}

void testScaleToMinLeavesLargeInputsAlone() {
  double left = 3.0;
  double right = 6.0;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, 3.0, kEps);
  CHECK_NEAR(right, 6.0, kEps);

  left = -3.0;
  right = -6.0;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, -3.0, kEps);
  CHECK_NEAR(right, -6.0, kEps);

  // Exactly at the bound is left alone.
  left = 2.0;
  right = 2.0;
  scaleToMin(left, right, 2.0);
  CHECK_NEAR(left, 2.0, kEps);
  CHECK_NEAR(right, 2.0, kEps);
}

void testScaleToMinZeroSideIsNotDividedBy() {
  // Both zero: nothing to scale, and no division by zero.
  double left = 0.0;
  double right = 0.0;
  scaleToMin(left, right, 2.0);
  CHECK_EQ(left, 0.0);
  CHECK_EQ(right, 0.0);
  CHECK(std::isfinite(left) && std::isfinite(right));

  // One side zero: scaling would need an infinite ratio, so nothing happens.
  left = 0.0;
  right = 1.0;
  scaleToMin(left, right, 2.0);
  CHECK_EQ(left, 0.0);
  CHECK_EQ(right, 1.0);

  // LIMITATION (report only): the non-zero side stays below min_output here.
  // A one-side-only crawl is never boosted over static friction.
  left = 0.5;
  right = 0.0;
  scaleToMin(left, right, 2.0);
  CHECK_EQ(left, 0.5);
  CHECK_EQ(right, 0.0);
}

void testScaleToMaxCapsBothSides() {
  double left = 6.0;
  double right = 3.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, 2.0, kEps);
  checkRatioPreserved(6.0, 3.0, left, right);

  left = 3.0;
  right = 6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 2.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);
  checkRatioPreserved(3.0, 6.0, left, right);
}

void testScaleToMaxNegative() {
  double left = -6.0;
  double right = -3.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, -4.0, kEps);
  CHECK_NEAR(right, -2.0, kEps);
  checkRatioPreserved(-6.0, -3.0, left, right);

  left = -3.0;
  right = -6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, -2.0, kEps);
  CHECK_NEAR(right, -4.0, kEps);
}

void testScaleToMaxMixedSigns() {
  double left = 6.0;
  double right = -3.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, -2.0, kEps);
  checkRatioPreserved(6.0, -3.0, left, right);
}

void testScaleToMaxLeavesSmallInputsAlone() {
  double left = 3.0;
  double right = 2.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 3.0, kEps);
  CHECK_NEAR(right, 2.0, kEps);

  left = -3.0;
  right = -2.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, -3.0, kEps);
  CHECK_NEAR(right, -2.0, kEps);

  // Exactly at the bound is left alone.
  left = 4.0;
  right = 4.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);
}

void testScaleToMaxEqualMagnitudes() {
  // Equal positive magnitudes are capped: branch 1 uses >=.
  double left = 6.0;
  double right = 6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);

  // BUG (report only, control/ is owned by Phase 2): equal *negative*
  // magnitudes are NOT capped. The positive-overflow branch uses
  // `fabs(left) >= fabs(right)` but the negative-overflow branch uses a
  // strict `fabs(left) > fabs(right)`, so a tie falls through every branch.
  // Driving straight backwards at full command is exactly this case.
  // Reported, not asserted: fixing the branch must not turn this test red.
  left = -6.0;
  right = -6.0;
  scaleToMax(left, right, 4.0);
  mclib::test::knownBug(
      left < -4.0 - kEps || right < -4.0 - kEps,
      "scaleToMax leaves equal negative magnitudes above max_output "
      "(scaling.cpp negative branch uses > where the positive branch uses >=)");
}

void testScaleToMaxZero() {
  double left = 0.0;
  double right = 0.0;
  scaleToMax(left, right, 4.0);
  CHECK_EQ(left, 0.0);
  CHECK_EQ(right, 0.0);

  // One side zero, the other over the cap: the non-zero side is the larger
  // magnitude, so it is capped and the zero side stays zero.
  left = 6.0;
  right = 0.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_EQ(right, 0.0);

  left = 0.0;
  right = -6.0;
  scaleToMax(left, right, 4.0);
  CHECK_EQ(left, 0.0);
  CHECK_NEAR(right, -4.0, kEps);
}

}  // namespace

int main() {
  testScaleToMinBoostsBothSides();
  testScaleToMinNegative();
  testScaleToMinEqualMagnitudes();
  testScaleToMinMixedSigns();
  testScaleToMinLeavesLargeInputsAlone();
  testScaleToMinZeroSideIsNotDividedBy();
  testScaleToMaxCapsBothSides();
  testScaleToMaxNegative();
  testScaleToMaxMixedSigns();
  testScaleToMaxLeavesSmallInputsAlone();
  testScaleToMaxEqualMagnitudes();
  testScaleToMaxZero();
  return mclib::test::summary("scaling");
}
