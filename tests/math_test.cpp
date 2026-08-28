// mclib
#include "mclib/math.hpp"

#include "test_assert.hpp"

#include <cmath>
#include <limits>

using mclib::arcRadius;
using mclib::clamp;
using mclib::kPi;
using mclib::kTwoPi;
using mclib::Mat2;
using mclib::Pose2D;
using mclib::rotate;
using mclib::rotationMatrix;
using mclib::Vec2;
using mclib::wrapAngle;
using mclib::Vec3;

namespace {

constexpr double kEps = 1e-12;

void testClamp() {
  CHECK_EQ(clamp(5.0, 0.0, 10.0), 5.0);
  CHECK_EQ(clamp(-1.0, 0.0, 10.0), 0.0);
  CHECK_EQ(clamp(11.0, 0.0, 10.0), 10.0);
  CHECK_EQ(clamp(0.0, 0.0, 10.0), 0.0);
  CHECK_EQ(clamp(10.0, 0.0, 10.0), 10.0);

  // clamp swaps min and max when they are given inverted.
  CHECK_EQ(clamp(5.0, 10.0, 0.0), 5.0);
  CHECK_EQ(clamp(-1.0, 10.0, 0.0), 0.0);
  CHECK_EQ(clamp(11.0, 10.0, 0.0), 10.0);

  // Degenerate range: min == max pins every input to that value.
  CHECK_EQ(clamp(5.0, 3.0, 3.0), 3.0);
  CHECK_EQ(clamp(-5.0, 3.0, 3.0), 3.0);

  // Negative ranges.
  CHECK_EQ(clamp(-7.0, -5.0, -1.0), -5.0);
  CHECK_EQ(clamp(-3.0, -5.0, -1.0), -3.0);
  CHECK_EQ(clamp(0.0, -5.0, -1.0), -1.0);
}

void testWrapAngle() {
  CHECK_NEAR(wrapAngle(0.0), 0.0, kEps);
  CHECK_NEAR(wrapAngle(1.0), 1.0, kEps);
  CHECK_NEAR(wrapAngle(-1.0), -1.0, kEps);

  // +pi stays +pi, so the range is closed at the top.
  CHECK_NEAR(wrapAngle(kPi), kPi, kEps);
  // -pi also stays -pi. The range is really [-pi, pi], not (-pi, pi].
  CHECK_NEAR(wrapAngle(-kPi), -kPi, kEps);

  CHECK_NEAR(wrapAngle(kTwoPi), 0.0, kEps);
  CHECK_NEAR(wrapAngle(-kTwoPi), 0.0, kEps);
  CHECK_NEAR(wrapAngle(3.0 * kPi), kPi, kEps);
  CHECK_NEAR(wrapAngle(1.5 * kPi), -0.5 * kPi, kEps);
  CHECK_NEAR(wrapAngle(-1.5 * kPi), 0.5 * kPi, kEps);
  CHECK_NEAR(wrapAngle(kTwoPi + 0.25), 0.25, 1e-9);
  CHECK_NEAR(wrapAngle(-kTwoPi - 0.25), -0.25, 1e-9);

  // A single fmod plus one correction is enough no matter how many turns.
  CHECK_NEAR(wrapAngle(100.0), 100.0 - 16.0 * kTwoPi, 1e-9);
  CHECK_NEAR(wrapAngle(-100.0), -100.0 + 16.0 * kTwoPi, 1e-9);

  // The result is always inside [-pi, pi].
  for (int i = -40; i <= 40; ++i) {
    const double wrapped = wrapAngle(static_cast<double>(i) * 0.7);
    CHECK(wrapped >= -kPi - kEps && wrapped <= kPi + kEps);
  }
}

void testPoseAdd() {
  const Pose2D a{1.0, 2.0, 0.5 * kPi};
  const Pose2D b{3.0, 4.0, kPi};
  const Pose2D sum = a + b;
  CHECK_NEAR(sum.x, 4.0, kEps);
  CHECK_NEAR(sum.y, 6.0, kEps);
  // theta is wrapped: 1.5*pi -> -0.5*pi.
  CHECK_NEAR(sum.theta, -0.5 * kPi, kEps);

  // operator+ is component-wise, NOT an SE(2) compose. The right-hand
  // translation is added in the world frame, not rotated into a's frame.
  const Pose2D at_origin{0.0, 0.0, 0.5 * kPi};
  const Pose2D forward{1.0, 0.0, 0.0};
  const Pose2D moved = at_origin + forward;
  CHECK_NEAR(moved.x, 1.0, kEps);
  CHECK_NEAR(moved.y, 0.0, kEps);
  // An SE(2) compose would have given (0, 1) instead. If operator+ is ever
  // changed to compose properly, the two checks above are what to update.

  // Adding the identity pose leaves x and y alone.
  const Pose2D identity{0.0, 0.0, 0.0};
  const Pose2D same = a + identity;
  CHECK_NEAR(same.x, a.x, kEps);
  CHECK_NEAR(same.y, a.y, kEps);
  CHECK_NEAR(same.theta, a.theta, kEps);
}

void testDistanceTo() {
  const Pose2D origin{0.0, 0.0, 0.0};
  const Pose2D p{3.0, 4.0, 0.0};
  CHECK_NEAR(origin.distanceTo(p), 5.0, kEps);
  CHECK_NEAR(p.distanceTo(origin), 5.0, kEps);
  CHECK_NEAR(p.distanceTo(p), 0.0, kEps);

  // theta is ignored.
  const Pose2D spun{3.0, 4.0, 10.0};
  CHECK_NEAR(origin.distanceTo(spun), 5.0, kEps);

  const Pose2D neg{-3.0, -4.0, 0.0};
  CHECK_NEAR(origin.distanceTo(neg), 5.0, kEps);
  CHECK_NEAR(p.distanceTo(neg), 10.0, kEps);
}

void testTranslationAndVector() {
  const Pose2D p{1.5, -2.5, 0.25};
  const Vec2 t = p.translation();
  CHECK_NEAR(t.x(), 1.5, kEps);
  CHECK_NEAR(t.y(), -2.5, kEps);

  const Vec3 v = p.vector();
  CHECK_NEAR(v(0), 1.5, kEps);
  CHECK_NEAR(v(1), -2.5, kEps);
  CHECK_NEAR(v(2), 0.25, kEps);
}

void testRotationMatrix() {
  const Mat2 identity = rotationMatrix(0.0);
  CHECK_NEAR(identity(0, 0), 1.0, kEps);
  CHECK_NEAR(identity(0, 1), 0.0, kEps);
  CHECK_NEAR(identity(1, 0), 0.0, kEps);
  CHECK_NEAR(identity(1, 1), 1.0, kEps);

  // Counter-clockwise by 90 degrees: [[0, -1], [1, 0]].
  const Mat2 quarter = rotationMatrix(0.5 * kPi);
  CHECK_NEAR(quarter(0, 0), 0.0, 1e-15);
  CHECK_NEAR(quarter(0, 1), -1.0, kEps);
  CHECK_NEAR(quarter(1, 0), 1.0, kEps);
  CHECK_NEAR(quarter(1, 1), 0.0, 1e-15);

  // Rotations have determinant 1 and compose additively.
  const Mat2 r = rotationMatrix(0.7);
  const double det = r(0, 0) * r(1, 1) - r(0, 1) * r(1, 0);
  CHECK_NEAR(det, 1.0, 1e-12);
  const Mat2 composed = rotationMatrix(0.3) * rotationMatrix(0.4);
  const Mat2 direct = rotationMatrix(0.7);
  CHECK_NEAR((composed - direct).cwiseAbs().maxCoeff(), 0.0, 1e-12);

  // Transpose is the inverse.
  const Mat2 round_trip = rotationMatrix(1.1) * rotationMatrix(1.1).transpose();
  CHECK_NEAR(round_trip(0, 0), 1.0, 1e-12);
  CHECK_NEAR(round_trip(0, 1), 0.0, 1e-12);
  CHECK_NEAR(round_trip(1, 0), 0.0, 1e-12);
  CHECK_NEAR(round_trip(1, 1), 1.0, 1e-12);
}

void testRotate() {
  const Vec2 unit_x{1.0, 0.0};
  const Vec2 turned = rotate(unit_x, 0.5 * kPi);
  CHECK_NEAR(turned.x(), 0.0, 1e-15);
  CHECK_NEAR(turned.y(), 1.0, kEps);

  const Vec2 back = rotate(turned, -0.5 * kPi);
  CHECK_NEAR(back.x(), 1.0, kEps);
  CHECK_NEAR(back.y(), 0.0, 1e-15);

  // Rotation preserves length.
  const Vec2 v{3.0, -4.0};
  CHECK_NEAR(rotate(v, 1.234).norm(), 5.0, 1e-12);

  // Rotating by pi negates.
  const Vec2 flipped = rotate(v, kPi);
  CHECK_NEAR(flipped.x(), -3.0, 1e-12);
  CHECK_NEAR(flipped.y(), 4.0, 1e-12);

  // Zero vector is fixed.
  const Vec2 zero{0.0, 0.0};
  CHECK_NEAR(rotate(zero, 2.0).norm(), 0.0, kEps);
}

/**
 * @brief arcRadius() answers "straight line" on a tolerance, not `== 0.0`.
 *
 * The exact test that used to be here split two geometrically identical cases:
 * dead ahead of a robot at heading 0 has a lateral offset of exactly 0, but
 * dead behind (heading pi) picks up `sin(pi) == 1.22e-16` and produced a
 * finite -4.08e16.
 */
void testArcRadiusStraightLine() {
  const double kInf = std::numeric_limits<double>::infinity();

  // Dead ahead, heading 0: exactly zero lateral offset even before the floor.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{0.0, 10.0}), kInf);
  // Dead behind the same robot. Also exactly zero lateral.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{0.0, -10.0}), kInf);

  // The cases the exact test got wrong: the heading, not the target, carries
  // the rounding. A robot at heading pi with the target 10 in behind it in
  // field terms is 10 in dead ahead of itself. sin(pi) is 1.2246e-16, so the
  // lateral offset is 1.2246e-15 and the old code returned 100 / 2.4493e-15 =
  // 4.0828e16.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, kPi}, Vec2{0.0, -10.0}), kInf);
  // ...and 10 in dead behind that robot, where the old answer was -4.0828e16
  // and its sqrt() is NaN.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, kPi}, Vec2{0.0, 10.0}), kInf);

  // Heading +/-90 deg, the other place the rounding shows up.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, kPi / 2.0}, Vec2{10.0, 0.0}), kInf);
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, -kPi / 2.0}, Vec2{-10.0, 0.0}), kInf);

  // Target on top of the robot: 0 / 0, answered as a straight line rather than
  // NaN.
  CHECK_EQ(arcRadius(Pose2D{3.0, -4.0, 1.0}, Vec2{3.0, -4.0}), kInf);
}

/**
 * @brief Pin the straight-line tolerance, so the epsilon is deliberate.
 *
 * The test is `|lateral| <= 1e-12 * chord_sq`. With the target 10 in ahead the
 * chord squared is 100, so the threshold is a lateral offset of 1e-10 in.
 */
void testArcRadiusBoundary() {
  const double kInf = std::numeric_limits<double>::infinity();
  const Pose2D at_origin{0.0, 0.0, 0.0};

  // Just inside the floor: 9e-11 <= 1e-10, so this is a straight line. (The
  // chord squared is 100 to well within the tolerance either way.)
  CHECK_EQ(arcRadius(at_origin, Vec2{9e-11, 10.0}), kInf);
  // Exactly on it. The comparison is `<=`, so the boundary is a straight line.
  CHECK_EQ(arcRadius(at_origin, Vec2{1e-10, 10.0}), kInf);
  // Just outside: a finite radius of about 100 / 2.2e-10 = 4.5e11 in.
  const double just_outside = arcRadius(at_origin, Vec2{1.1e-10, 10.0});
  CHECK(std::isfinite(just_outside));
  CHECK_NEAR(just_outside, 100.0 / (2.0 * 1.1e-10), 1.0);

  // The bound is relative to the chord, so it scales with distance. At 1 in
  // out the chord squared is 1 and the threshold is 1e-12, which means a
  // lateral offset of 1e-10 - a straight line at 10 in - is a finite arc here.
  CHECK(std::isfinite(arcRadius(at_origin, Vec2{1e-10, 1.0})));
}

/// @brief A real arc of known radius still comes back as that radius.
void testArcRadiusKnownArcs() {
  // Circle of radius 5 tangent to the robot's forward axis at the origin, with
  // its centre 5 in to the right. Sample it at a few angles: every point on it
  // must report radius 5.
  const double r = 5.0;
  for (double t = 0.1; t < 3.0; t += 0.1) {
    const Vec2 on_circle{r - r * std::cos(t), r * std::sin(t)};
    CHECK_NEAR(arcRadius(Pose2D{0.0, 0.0, 0.0}, on_circle), r, 1e-11);
  }

  // Signed: the mirror image curves left and reports -5.
  for (double t = 0.1; t < 3.0; t += 0.1) {
    const Vec2 on_circle{-(r - r * std::cos(t)), r * std::sin(t)};
    CHECK_NEAR(arcRadius(Pose2D{0.0, 0.0, 0.0}, on_circle), -r, 1e-11);
  }

  // Translated and rotated: the same arc seen from a robot at (30, -12)
  // heading 90 deg. Its forward axis is +X, its right is -Y.
  const Pose2D pose{30.0, -12.0, kPi / 2.0};
  const Vec2 quarter_turn{30.0 + r, -12.0 - r};  // r forward, r right
  CHECK_NEAR(arcRadius(pose, quarter_turn), r, 1e-11);

  // The simplest case: a target 1 in directly to the robot's right is a
  // half-circle of radius 0.5.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{1.0, 0.0}), 0.5);
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{-1.0, 0.0}), -0.5);
}

}  // namespace

int main() {
  testClamp();
  testWrapAngle();
  testPoseAdd();
  testDistanceTo();
  testTranslationAndVector();
  testRotationMatrix();
  testRotate();
  testArcRadiusStraightLine();
  testArcRadiusBoundary();
  testArcRadiusKnownArcs();
  return mclib::test::summary("math");
}
