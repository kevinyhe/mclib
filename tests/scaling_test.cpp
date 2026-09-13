// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
  // Equal positive magnitudes are capped.
  double left = 6.0;
  double right = 6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);
  checkRatioPreserved(6.0, 6.0, left, right);

  // Equal negative magnitudes are capped too: driving straight backwards at
  // full command. This used to fall through all four branches unchanged,
  // because the negative-overflow branch tested `>` where the positive one
  // tested `>=`.
  left = -6.0;
  right = -6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, -4.0, kEps);
  CHECK_NEAR(right, -4.0, kEps);
  checkRatioPreserved(-6.0, -6.0, left, right);

  // Mixed-sign ties - a point turn at full command - are the same hole.
  left = 6.0;
  right = -6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, 4.0, kEps);
  CHECK_NEAR(right, -4.0, kEps);
  checkRatioPreserved(6.0, -6.0, left, right);

  left = -6.0;
  right = 6.0;
  scaleToMax(left, right, 4.0);
  CHECK_NEAR(left, -4.0, kEps);
  CHECK_NEAR(right, 4.0, kEps);
  checkRatioPreserved(-6.0, 6.0, left, right);
}

/**
 * @brief Every sign combination of an over-cap pair, ratio checked each time.
 *
 * The tie bug was a boundary nobody had enumerated, so enumerate the lot.
 */
void testScaleToMaxSignMatrix() {
  struct Case {
    double left_in, right_in, left_out, right_out;
  };
  const Case cases[] = {
      // both positive, either side larger
      {6.0, 3.0, 4.0, 2.0},
      {3.0, 6.0, 2.0, 4.0},
      // both negative, either side larger
      {-6.0, -3.0, -4.0, -2.0},
      {-3.0, -6.0, -2.0, -4.0},
      // mixed signs, either side larger
      {6.0, -3.0, 4.0, -2.0},
      {-6.0, 3.0, -4.0, 2.0},
      {3.0, -6.0, 2.0, -4.0},
      {-3.0, 6.0, -2.0, 4.0},
      // ties, all four sign pairs
      {6.0, 6.0, 4.0, 4.0},
      {-6.0, -6.0, -4.0, -4.0},
      {6.0, -6.0, 4.0, -4.0},
      {-6.0, 6.0, -4.0, 4.0},
      // one side zero, the other over the cap in either direction
      {6.0, 0.0, 4.0, 0.0},
      {0.0, 6.0, 0.0, 4.0},
      {-6.0, 0.0, -4.0, 0.0},
      {0.0, -6.0, 0.0, -4.0},
      // both zero, and pairs already inside the cap
      {0.0, 0.0, 0.0, 0.0},
      {4.0, 4.0, 4.0, 4.0},
      {-4.0, -4.0, -4.0, -4.0},
      {3.0, 2.0, 3.0, 2.0},
      {-3.0, -2.0, -3.0, -2.0},
      {2.0, -3.0, 2.0, -3.0},
  };
  for (const Case& c : cases) {
    double left = c.left_in;
    double right = c.right_in;
    scaleToMax(left, right, 4.0);
    CHECK_NEAR(left, c.left_out, kEps);
    CHECK_NEAR(right, c.right_out, kEps);
    CHECK(std::isfinite(left) && std::isfinite(right));
    // Nothing comes back over the cap.
    CHECK(fabs(left) <= 4.0 + kEps);
    CHECK(fabs(right) <= 4.0 + kEps);
    checkRatioPreserved(c.left_in, c.right_in, left, right);
  }
}

/**
 * @brief The sibling function has no tie hole - both of its "left side" branches
 *        already use `<=`, so a tie goes to the left in every sign pair.
 */
void testScaleToMinSignMatrix() {
  struct Case {
    double left_in, right_in, left_out, right_out;
  };
  const Case cases[] = {
      // both positive, either side smaller
      {0.5, 1.0, 2.0, 4.0},
      {1.0, 0.5, 4.0, 2.0},
      // both negative, either side smaller
      {-0.5, -1.0, -2.0, -4.0},
      {-1.0, -0.5, -4.0, -2.0},
      // mixed signs, either side smaller
      {-0.5, 1.0, -2.0, 4.0},
      {1.0, -0.5, 4.0, -2.0},
      {0.5, -1.0, 2.0, -4.0},
      {-1.0, 0.5, -4.0, 2.0},
      // ties, all four sign pairs
      {0.5, 0.5, 2.0, 2.0},
      {-0.5, -0.5, -2.0, -2.0},
      {0.5, -0.5, 2.0, -2.0},
      {-0.5, 0.5, -2.0, 2.0},
      // A zero side is never divided by. Only the both-zero case is asserted
      // here: a zero paired with a below-floor value is the known limitation
      // that testScaleToMinZeroSideIsNotDividedBy documents, and pinning it
      // twice would just mean two tests to delete when it is fixed.
      {0.0, 0.0, 0.0, 0.0},
      // already at or above the floor
      {2.0, 2.0, 2.0, 2.0},
      {-2.0, -2.0, -2.0, -2.0},
      {3.0, 6.0, 3.0, 6.0},
      {-3.0, -6.0, -3.0, -6.0},
  };
  for (const Case& c : cases) {
    double left = c.left_in;
    double right = c.right_in;
    scaleToMin(left, right, 2.0);
    CHECK_NEAR(left, c.left_out, kEps);
    CHECK_NEAR(right, c.right_out, kEps);
    CHECK(std::isfinite(left) && std::isfinite(right));
    checkRatioPreserved(c.left_in, c.right_in, left, right);
  }
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
  testScaleToMaxSignMatrix();
  testScaleToMinSignMatrix();
  return mclib::test::summary("scaling");
}
