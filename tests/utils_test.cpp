// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/utils.hpp"

#include <cmath>

#include "test_assert.hpp"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEps = 1e-12;

void testDegToRad() {
  CHECK_NEAR(degToRad(0.0), 0.0, kEps);
  CHECK_NEAR(degToRad(90.0), 0.5 * kPi, kEps);
  CHECK_NEAR(degToRad(180.0), kPi, kEps);
  CHECK_NEAR(degToRad(360.0), 2.0 * kPi, kEps);
  CHECK_NEAR(degToRad(-90.0), -0.5 * kPi, kEps);
  CHECK_NEAR(degToRad(45.0), 0.25 * kPi, kEps);
  // No wrapping: degToRad is a plain scale.
  CHECK_NEAR(degToRad(720.0), 4.0 * kPi, kEps);
}

void testRadToDeg() {
  CHECK_NEAR(radToDeg(0.0), 0.0, kEps);
  CHECK_NEAR(radToDeg(0.5 * kPi), 90.0, 1e-10);
  CHECK_NEAR(radToDeg(kPi), 180.0, 1e-10);
  CHECK_NEAR(radToDeg(-kPi), -180.0, 1e-10);
  CHECK_NEAR(radToDeg(2.0 * kPi), 360.0, 1e-10);
}

void testRoundTrip() {
  for (int deg = -720; deg <= 720; deg += 15) {
    const double d = static_cast<double>(deg);
    CHECK_NEAR(radToDeg(degToRad(d)), d, 1e-10);
  }
}

void testGetRadius() {
  // denominator = 2 * dy * sin(90 - angle) = 2 * dy * cos(angle).
  // radius = (dx^2 + dy^2) / denominator.

  // dx = 3, dy = 4, angle = 0 -> denom = 8, chord^2 = 25 -> 3.125.
  CHECK_NEAR(getRadius(0.0, 0.0, 3.0, 4.0, 0.0), 3.125, 1e-12);

  // dx = 0, dy = 2, angle = 60 -> denom = 2 * 2 * 0.5 = 2, chord^2 = 4 -> 2.
  CHECK_NEAR(getRadius(0.0, 0.0, 0.0, 2.0, 60.0), 2.0, 1e-12);

  // Start point is subtracted, not assumed to be the origin.
  CHECK_NEAR(getRadius(10.0, 20.0, 13.0, 24.0, 0.0), 3.125, 1e-12);

  // Negative dy flips the sign of the radius.
  CHECK_NEAR(getRadius(0.0, 0.0, 3.0, -4.0, 0.0), -3.125, 1e-12);

  // Degenerate: dy == 0 makes the denominator exactly zero, so the function
  // returns +infinity rather than dividing. (It returned a magic 999 before
  // the frame-reconciliation change; infinity is an honest "no finite radius"
  // and does not silently become a finite speed limit downstream.)
  CHECK(std::isinf(getRadius(0.0, 0.0, 5.0, 0.0, 0.0)));
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 0.0, 0.0)));
  CHECK(std::isinf(getRadius(7.0, 7.0, -3.0, 7.0, 30.0)));

  // All three headings where sin(90 - angle) vanishes are the same degenerate
  // case and must agree. angle == 90 gives sin(0) == 0 exactly; angle == -90
  // and angle == 270 give +/-1.2246e-16, which an exact `denominator == 0`
  // test missed -- they used to return +/-4.0828e15, a finite number that
  // flowed downstream as if it were a real radius.
  CHECK(std::isinf(getRadius(0.0, 0.0, 1.0, 1.0, 90.0)));
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 1.0, -90.0)));
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 1.0, 270.0)));
  CHECK(std::isinf(getRadius(0.0, 0.0, 3.0, 4.0, -90.0)));
  CHECK(std::isinf(getRadius(0.0, 0.0, 3.0, 4.0, 450.0)));
  // The sentinel is +infinity, not -infinity, on the branch where the finite
  // result would have been negative (angle == 270 flips the denominator sign).
  CHECK(getRadius(0.0, 0.0, 0.0, 1.0, 270.0) > 0.0);

  // Heading boundary. The check is |sin(degToRad(90 - angle))| <= 1e-9, and
  // sin(degToRad(d)) ~= 1.7453e-8 * d for small d in degrees, so the boundary
  // sits 5.7296e-8 deg from 90. Straddle it by a decade on each side.
  //
  // Just inside (degenerate): 1e-8 deg off 90 -> |sin| = 1.7453e-10 <= 1e-9.
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 1.0, 90.0 - 1e-8)));
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 1.0, 90.0 + 1e-8)));
  // Just outside (finite): 1e-7 deg off 90 -> |sin| = 1.7453e-9 > 1e-9, and
  // radius = 1 / (2 * 1.745329148e-9) = 2.8647e8. Relative tolerance, because
  // the value is large and the sin argument is itself near cancellation.
  const double just_outside = getRadius(0.0, 0.0, 0.0, 1.0, 90.0 - 1e-7);
  CHECK(std::isfinite(just_outside));
  CHECK_NEAR(just_outside, 2.8647891457e8, 1e3);

  // A heading that is near-degenerate by robot standards but nowhere near the
  // tolerance still returns its real finite radius: 0.01 deg off 90 is the
  // finest a V5 IMU reports, and sin(degToRad(0.01)) = 1.74533e-4.
  const double imu_resolution = getRadius(0.0, 0.0, 0.0, 1.0, 90.0 - 0.01);
  CHECK(std::isfinite(imu_resolution));
  CHECK_NEAR(imu_resolution, 1.0 / (2.0 * std::sin(degToRad(0.01))), 1e-6);

  // Magnitude boundary. Independent of the heading, the result is called
  // degenerate once |denominator| <= 1e-12 * chord^2, i.e. once |radius| would
  // reach 1e12. With dx = 0 and angle = 0 the radius is just dy / 2, so the
  // boundary sits at dy = 2e12.
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 2e12, 0.0)));      // radius 1e12
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, -2e12, 0.0)));     // radius -1e12
  const double under_cap = getRadius(0.0, 0.0, 0.0, 1e12, 0.0);  // radius 5e11
  CHECK(std::isfinite(under_cap));
  CHECK_NEAR(under_cap, 5e11, 1.0);

  // The same guard catches a tiny dy that the heading floor cannot see: at
  // angle == -90 the sin term is 1.2246e-16, so dy = 1e-4 gives a denominator
  // of 2.4e-20 against a chord^2 of 1e-8 -- a ratio of 2.4e-12, which the
  // magnitude test rejects even though the old code returned 4.08e11.
  CHECK(std::isinf(getRadius(0.0, 0.0, 0.0, 1e-4, -90.0)));

  // A genuinely small radius is not degenerate: the test is relative to the
  // chord, so shrinking the whole geometry does not trip it.
  CHECK_NEAR(getRadius(0.0, 0.0, 0.0, 1e-10, 0.0), 5e-11, 1e-22);
  CHECK_NEAR(getRadius(0.0, 0.0, 3e-6, 4e-6, 0.0), 3.125e-6, 1e-18);
}

}  // namespace

int main() {
  testDegToRad();
  testRadToDeg();
  testRoundTrip();
  testGetRadius();
  return mclib::test::summary("utils");
}
