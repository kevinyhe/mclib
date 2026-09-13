// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file holonomic_math_test.cpp
 * @brief The X-drive / mecanum wheel mix and the field-to-robot rotation.
 *
 * The wheel expectations are written from first principles, not from the
 * formula: pure forward is four equal wheels, a right strafe on mecanum pushes
 * the front-left/back-right diagonal forward, a clockwise turn drives the
 * left side forward.
 */
#include "mclib/chassis/holonomic_math.hpp"
#include "mclib/math.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <limits>

using mclib::holonomic::fieldToRobot;
using mclib::holonomic::Kind;
using mclib::holonomic::mix;
using mclib::holonomic::RobotFrameInput;
using mclib::holonomic::WheelSpeeds;

namespace {

constexpr double kDeg = mclib::kPi / 180.0;

double peak(const WheelSpeeds& w) {
  return std::max({std::fabs(w.front_left), std::fabs(w.front_right),
                   std::fabs(w.back_left), std::fabs(w.back_right)});
}

/// @brief Forward alone: every wheel gets the same command, on both layouts.
void testPureForwardIsAllEqual() {
  for (Kind kind : {Kind::XDrive, Kind::Mecanum}) {
    for (double f : {1.0, 0.4, -0.7}) {
      const WheelSpeeds w = mix(f, 0.0, 0.0, kind);
      CHECK_NEAR(w.front_left, f, 1e-12);
      CHECK_NEAR(w.front_right, f, 1e-12);
      CHECK_NEAR(w.back_left, f, 1e-12);
      CHECK_NEAR(w.back_right, f, 1e-12);
    }
  }
}

/// @brief Right strafe on mecanum: front-left and back-right forward, the other diagonal back.
void testRightStrafeOnMecanum() {
  const WheelSpeeds w = mix(0.0, 0.5, 0.0, Kind::Mecanum);
  CHECK_NEAR(w.front_left, 0.5, 1e-12);
  CHECK_NEAR(w.back_right, 0.5, 1e-12);
  CHECK_NEAR(w.front_right, -0.5, 1e-12);
  CHECK_NEAR(w.back_left, -0.5, 1e-12);
  CHECK(w.front_left > 0.0 && w.back_right > 0.0);
  CHECK(w.front_right < 0.0 && w.back_left < 0.0);

  // Left strafe is the mirror image.
  const WheelSpeeds left = mix(0.0, -0.5, 0.0, Kind::Mecanum);
  CHECK_NEAR(left.front_left, -0.5, 1e-12);
  CHECK_NEAR(left.back_right, -0.5, 1e-12);
  CHECK_NEAR(left.front_right, 0.5, 1e-12);
  CHECK_NEAR(left.back_left, 0.5, 1e-12);
}

/// @brief The X-drive has the same 45 deg geometry on the same diagonals.
void testRightStrafeOnXDrive() {
  const WheelSpeeds w = mix(0.0, 1.0, 0.0, Kind::XDrive);
  CHECK(w.front_left > 0.0 && w.back_right > 0.0);
  CHECK(w.front_right < 0.0 && w.back_left < 0.0);
  CHECK_NEAR(w.front_left, -w.front_right, 1e-12);
  CHECK_NEAR(w.back_right, -w.back_left, 1e-12);
  // Forward and strafe carry the same weight: full strafe is full wheel.
  CHECK_NEAR(w.front_left, 1.0, 1e-12);
}

/// @brief Clockwise turn: left side forward, right side backward.
void testClockwiseTurn() {
  for (Kind kind : {Kind::XDrive, Kind::Mecanum}) {
    const WheelSpeeds w = mix(0.0, 0.0, 0.3, kind);
    CHECK_NEAR(w.front_left, 0.3, 1e-12);
    CHECK_NEAR(w.back_left, 0.3, 1e-12);
    CHECK_NEAR(w.front_right, -0.3, 1e-12);
    CHECK_NEAR(w.back_right, -0.3, 1e-12);
  }
}

/// @brief Saturation divides all four by the largest, keeping ratios.
void testNormalisationPreservesRatios() {
  const WheelSpeeds w = mix(1.0, 1.0, 1.0, Kind::Mecanum);  // raw 3, -1, 1, 1
  CHECK_NEAR(w.front_left, 1.0, 1e-12);
  CHECK_NEAR(w.front_right, -1.0 / 3.0, 1e-12);
  CHECK_NEAR(w.back_left, 1.0 / 3.0, 1e-12);
  CHECK_NEAR(w.back_right, 1.0 / 3.0, 1e-12);

  // Exhaustively: never past the rail, and unsaturated inputs untouched.
  for (double f = -1.0; f <= 1.0 + 1e-9; f += 0.25) {
    for (double s = -1.0; s <= 1.0 + 1e-9; s += 0.25) {
      for (double t = -1.0; t <= 1.0 + 1e-9; t += 0.25) {
        const WheelSpeeds w2 = mix(f, s, t, Kind::XDrive);
        CHECK(peak(w2) <= 1.0 + 1e-12);
        const double raw_fl = f + s + t;
        const double raw_fr = f - s - t;
        const double raw_bl = f - s + t;
        const double raw_br = f + s - t;
        const double raw_peak = std::max({std::fabs(raw_fl), std::fabs(raw_fr),
                                          std::fabs(raw_bl), std::fabs(raw_br)});
        const double scale = raw_peak > 1.0 ? raw_peak : 1.0;
        CHECK_NEAR(w2.front_left, raw_fl / scale, 1e-12);
        CHECK_NEAR(w2.front_right, raw_fr / scale, 1e-12);
        CHECK_NEAR(w2.back_left, raw_bl / scale, 1e-12);
        CHECK_NEAR(w2.back_right, raw_br / scale, 1e-12);
      }
    }
  }
}

/// @brief A NaN stick reading commands nothing rather than NaN volts.
void testNonFiniteInputsAreZero() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const WheelSpeeds w = mix(nan, 0.5, nan, Kind::Mecanum);
  CHECK_NEAR(w.front_left, 0.5, 1e-12);
  CHECK_NEAR(w.front_right, -0.5, 1e-12);
  const RobotFrameInput r = fieldToRobot(nan, 1.0, 0.0);
  CHECK_NEAR(r.forward, 1.0, 1e-12);
  CHECK_NEAR(r.strafe, 0.0, 1e-12);
}

/// @brief At heading 0 the frames coincide: field +Y is forward, field +X is right.
void testFieldToRobotIdentityAtZero() {
  const RobotFrameInput r = fieldToRobot(0.3, 0.8, 0.0);
  CHECK_NEAR(r.forward, 0.8, 1e-12);
  CHECK_NEAR(r.strafe, 0.3, 1e-12);
}

/// @brief Facing +X (heading 90): field +X is forward, field +Y is to the left.
void testFieldToRobotAtNinety() {
  const RobotFrameInput along_x = fieldToRobot(1.0, 0.0, 90.0 * kDeg);
  CHECK_NEAR(along_x.forward, 1.0, 1e-12);
  CHECK_NEAR(along_x.strafe, 0.0, 1e-12);

  const RobotFrameInput along_y = fieldToRobot(0.0, 1.0, 90.0 * kDeg);
  CHECK_NEAR(along_y.forward, 0.0, 1e-12);
  CHECK_NEAR(along_y.strafe, -1.0, 1e-12);

  // Facing -Y: everything reverses.
  const RobotFrameInput backwards = fieldToRobot(0.0, 1.0, 180.0 * kDeg);
  CHECK_NEAR(backwards.forward, -1.0, 1e-12);
  CHECK_NEAR(backwards.strafe, 0.0, 1e-12);
}

/// @brief Bit-for-bit the same rotation as mclib::fieldToRobot() in math.hpp.
void testMatchesMathFieldToRobot() {
  for (double heading = -720.0; heading <= 720.0; heading += 37.0) {
    for (double x : {-1.0, -0.2, 0.0, 0.6, 1.0}) {
      for (double y : {-1.0, 0.1, 1.0}) {
        const RobotFrameInput r = fieldToRobot(x, y, heading * kDeg);
        const mclib::Vec2 expected = mclib::fieldToRobot(mclib::Vec2{x, y}, heading * kDeg);
        CHECK_NEAR(r.strafe, expected.x(), 1e-12);
        CHECK_NEAR(r.forward, expected.y(), 1e-12);
      }
    }
  }
}

/// @brief Rotation preserves the length of the command.
void testFieldToRobotPreservesMagnitude() {
  for (double heading = 0.0; heading < 360.0; heading += 15.0) {
    const RobotFrameInput r = fieldToRobot(0.6, -0.8, heading * kDeg);
    CHECK_NEAR(std::hypot(r.forward, r.strafe), 1.0, 1e-12);
  }
}

}  // namespace

int main() {
  testPureForwardIsAllEqual();
  testRightStrafeOnMecanum();
  testRightStrafeOnXDrive();
  testClockwiseTurn();
  testNormalisationPreservesRatios();
  testNonFiniteInputsAreZero();
  testFieldToRobotIdentityAtZero();
  testFieldToRobotAtNinety();
  testMatchesMathFieldToRobot();
  testFieldToRobotPreservesMagnitude();
  return mclib::test::summary("holonomic_math_test");
}
