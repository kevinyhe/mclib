// mclib
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

  // angle == 90 gives sin(0), which is exactly zero, so this is degenerate too.
  CHECK(std::isinf(getRadius(0.0, 0.0, 1.0, 1.0, 90.0)));

  // BUG (report only, owned by another worker): the zero check is an exact
  // `denominator == 0` test. angle == -90 is geometrically the same
  // degenerate case as angle == 90, but sin(degToRad(180)) is 1.22e-16
  // rather than 0, so no sentinel is returned and the radius blows up to
  // ~1e15 instead of +infinity. Same for angle == 270.
  // Reported, not asserted: switching to an epsilon check is the fix, and
  // that must not turn this test red.
  const double near_degenerate = getRadius(0.0, 0.0, 0.0, 1.0, -90.0);
  mclib::test::knownBug(
      !std::isinf(near_degenerate),
      "getRadius compares the denominator to exactly 0, so angle == -90 "
      "returns ~1e15 instead of +infinity");
}

}  // namespace

int main() {
  testDegToRad();
  testRadToDeg();
  testRoundTrip();
  testGetRadius();
  return mclib::test::summary("utils");
}
