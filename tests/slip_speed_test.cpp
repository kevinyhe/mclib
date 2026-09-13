// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file slip_speed_test.cpp
 * @brief Host tests for boomerang()'s slip-speed limiter.
 *
 * `boomerang()` caps its drive output at
 * `sqrt(chase_power * radius * 9.8)` so it does not slide off a tight arc.
 * The radius used to come from `getRadius()` in utils.hpp, which is
 * frame-transposed: it puts the target's field-frame Y offset where its
 * robot-frame lateral offset belongs. `mclib::arcRadius()` is the correct
 * compass-frame version, and this file pins what changes.
 *
 * Everything here is the real code: `mclib::arcRadius()` from math.cpp and
 * `mclib::control::slipSpeedLimit()` from motion_math.hpp, wired together the
 * way motion.cpp wires them.
 */

#include "mclib/control/motion_math.hpp"
#include "mclib/math.hpp"
#include "mclib/utils.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <limits>

using mclib::Pose2D;
using mclib::Vec2;
using mclib::arcRadius;
using mclib::control::slipSpeedLimit;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// @brief What motion.cpp computes: the cap in volts for a robot at
///        @p heading_deg chasing a carrot at (@p cx, @p cy).
double slipSpeedAt(double chase_power, double x, double y, double heading_deg, double cx,
                   double cy) {
  return slipSpeedLimit(chase_power,
                        arcRadius(Pose2D{x, y, degToRad(heading_deg)}, Vec2{cx, cy}));
}

/// @brief The clamp motion.cpp applies. Returns the output after clamping.
double applyClamp(double output, double slip_speed) {
  if (output > slip_speed) {
    return slip_speed;
  }
  if (output < -slip_speed) {
    return -slip_speed;
  }
  return output;
}

}  // namespace

int main() {
  // -------------------------------------------------------------------------
  // 1. Dead ahead: infinite radius, no clamp.
  // -------------------------------------------------------------------------
  // Robot at the origin heading 0 (compass: +Y is forward), carrot 10 in
  // straight ahead at (0, 10). That is a straight line, not an arc.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{0.0, 10.0}), kInf);
  CHECK_EQ(slipSpeedAt(10.0, 0.0, 0.0, 0.0, 0.0, 10.0), kInf);
  // 12 V is not greater than infinity, so the clamp does not fire.
  CHECK_EQ(applyClamp(12.0, slipSpeedAt(10.0, 0.0, 0.0, 0.0, 0.0, 10.0)), 12.0);
  CHECK_EQ(applyClamp(-12.0, slipSpeedAt(10.0, 0.0, 0.0, 0.0, 0.0, 10.0)), -12.0);

  // The old helper said 5.0 in here - a hard 22.14 V cap on a straight line.
  // Harmless only because 22.14 is above the 12 V rail. Kept as a witness.
  CHECK_EQ(getRadius(0.0, 0.0, 0.0, 10.0, 0.0), 5.0);
  CHECK_NEAR(std::sqrt(10.0 * 5.0 * 9.8), 22.13594362, 1e-8);

  // Dead ahead is dead ahead at any heading. Robot heading 90 deg (compass:
  // facing +X), carrot 10 in along +X. cos(pi/2) is 6.1e-17 rather than 0, so
  // the lateral offset is 6.1e-16 in instead of exactly zero and the radius
  // comes out at 8.2e16 in rather than literally infinite. Same thing in
  // practice: the cap is 2.8e9 V, and nothing clamps against that.
  CHECK(arcRadius(Pose2D{0.0, 0.0, degToRad(90.0)}, Vec2{10.0, 0.0}) > 1e15);
  CHECK(slipSpeedAt(10.0, 0.0, 0.0, 90.0, 10.0, 0.0) > 1e9);
  CHECK_EQ(applyClamp(12.0, slipSpeedAt(10.0, 0.0, 0.0, 90.0, 10.0, 0.0)), 12.0);
  // ...and the old helper's denominator collapses at exactly +/-90 deg, so it
  // agreed here by accident, not by being right.
  CHECK_EQ(getRadius(0.0, 0.0, 10.0, 0.0, 90.0), kInf);

  // Dead behind is also a straight line.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{0.0, -10.0}), kInf);

  // -------------------------------------------------------------------------
  // 2. A genuine arc: finite radius, clamp fires at a predicted value.
  // -------------------------------------------------------------------------
  // Robot at the origin heading 0, carrot 1 in directly to its right at
  // (1, 0). Robot-frame offset is (right 1, forward 0), so
  //   radius = |(1, 0)|^2 / (2 * 1) = 1 / 2 = 0.5 in.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{1.0, 0.0}), 0.5);
  // sqrt(10 * 0.5 * 9.8) = sqrt(49) = 7 V, exactly.
  CHECK_EQ(slipSpeedAt(10.0, 0.0, 0.0, 0.0, 1.0, 0.0), 7.0);
  CHECK_EQ(applyClamp(12.0, 7.0), 7.0);
  CHECK_EQ(applyClamp(-12.0, 7.0), -7.0);
  CHECK_EQ(applyClamp(3.0, 7.0), 3.0);  // below the cap, untouched

  // The old helper's denominator is 2 * delta_y * sin(90 - heading) = 0 here,
  // so it called this tight arc a straight line and never limited it at all.
  CHECK_EQ(getRadius(0.0, 0.0, 1.0, 0.0, 0.0), kInf);

  // The mirror image, 1 in to the LEFT at (-1, 0). arcRadius is signed, so
  // this is -0.5 in...
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{-1.0, 0.0}), -0.5);
  // ...and slipSpeedLimit takes the magnitude, so it caps at the same 7 V. The
  // old expression did sqrt(-4.9) = NaN, and every comparison against NaN is
  // false, so the limiter silently dropped out on every left-hand arc.
  CHECK_EQ(slipSpeedAt(10.0, 0.0, 0.0, 0.0, -1.0, 0.0), 7.0);
  CHECK(std::isnan(std::sqrt(10.0 * -0.5 * 9.8)));

  // A wider arc allows more speed: carrot at (5, 5), robot-frame (right 5,
  // forward 5), radius = 50 / 10 = 5 in, cap = sqrt(490) = 22.136 V, which is
  // above the 12 V rail so nothing is limited in practice.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, 0.0}, Vec2{5.0, 5.0}), 5.0);
  CHECK_NEAR(slipSpeedAt(10.0, 0.0, 0.0, 0.0, 5.0, 5.0), 22.13594362, 1e-8);
  CHECK_EQ(applyClamp(12.0, slipSpeedAt(10.0, 0.0, 0.0, 0.0, 5.0, 5.0)), 12.0);

  // The frame matters, not just the offsets. Same carrot, robot rotated 90 deg
  // clockwise: (5, 5) is now (right -5, forward 5) in its frame, so the arc
  // curves left and the radius is -5 in - same magnitude, so the same 22.14 V
  // cap. The old helper, which reads field-frame Y where the robot-frame
  // lateral offset belongs, jumps from 5 to infinity for that same rotation.
  CHECK_EQ(arcRadius(Pose2D{0.0, 0.0, degToRad(90.0)}, Vec2{5.0, 5.0}), -5.0);
  CHECK_NEAR(slipSpeedAt(10.0, 0.0, 0.0, 90.0, 5.0, 5.0), 22.13594362, 1e-8);
  CHECK_NEAR(getRadius(0.0, 0.0, 5.0, 5.0, 0.0), 5.0, 1e-12);
  CHECK_EQ(getRadius(0.0, 0.0, 5.0, 5.0, 90.0), kInf);

  // Translation invariance: the same arc, driven from (30, -12).
  CHECK_EQ(arcRadius(Pose2D{30.0, -12.0, 0.0}, Vec2{31.0, -12.0}), 0.5);
  CHECK_EQ(slipSpeedAt(10.0, 30.0, -12.0, 0.0, 31.0, -12.0), 7.0);

  // -------------------------------------------------------------------------
  // 3. chase_power == 0: clamps to zero, never NaN.
  // -------------------------------------------------------------------------
  // With a finite radius the old expression already gave 0 - sqrt(0 * r * 9.8)
  // - and clamped the output to a standstill. On a straight line it gave
  // 0 * infinity = NaN, every comparison against NaN is false, and the limiter
  // silently vanished. Same gain, opposite behaviour, depending on geometry.
  CHECK(std::isnan(0.0 * kInf));
  // slipSpeedLimit is 0 at every radius, including infinite.
  CHECK_EQ(slipSpeedLimit(0.0, 0.5), 0.0);
  CHECK_EQ(slipSpeedLimit(0.0, kInf), 0.0);
  CHECK_EQ(slipSpeedLimit(0.0, -kInf), 0.0);
  CHECK_EQ(slipSpeedAt(0.0, 0.0, 0.0, 0.0, 0.0, 10.0), 0.0);  // dead ahead
  CHECK_EQ(slipSpeedAt(0.0, 0.0, 0.0, 0.0, 1.0, 0.0), 0.0);   // tight arc
  // And the clamp actually fires, in both directions.
  CHECK_EQ(applyClamp(12.0, slipSpeedAt(0.0, 0.0, 0.0, 0.0, 0.0, 10.0)), 0.0);
  CHECK_EQ(applyClamp(-12.0, slipSpeedAt(0.0, 0.0, 0.0, 0.0, 0.0, 10.0)), 0.0);

  // A negative chase_power is nonsense and used to make sqrt() return NaN at
  // every radius. Treated as zero: the robot stops, loudly, instead of the
  // limiter disappearing.
  CHECK_EQ(slipSpeedLimit(-1.0, 0.5), 0.0);
  CHECK_EQ(slipSpeedLimit(-1.0, kInf), 0.0);

  // The shipped default, 10, is unchanged and still positive.
  CHECK(slipSpeedLimit(10.0, 0.5) > 0.0);

  return mclib::test::summary("slip_speed");
}
