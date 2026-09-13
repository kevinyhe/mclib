// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/chassis/holonomic_math.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace holonomic {

namespace {

double finiteOrZero(double value) {
  return std::isfinite(value) ? value : 0.0;
}

}  // namespace

WheelSpeeds mix(double forward, double strafe, double turn, Kind kind) {
  // See the header: the sums are the same for both layouts. The parameter is
  // still read so a future difference between them has a home.
  (void)kind;

  const double f = finiteOrZero(forward);
  const double s = finiteOrZero(strafe);
  const double t = finiteOrZero(turn);

  WheelSpeeds w;
  w.front_left = f + s + t;
  w.front_right = f - s - t;
  w.back_left = f - s + t;
  w.back_right = f + s - t;

  const double peak = std::max({std::fabs(w.front_left), std::fabs(w.front_right),
                                std::fabs(w.back_left), std::fabs(w.back_right)});
  if (peak > 1.0) {
    w.front_left /= peak;
    w.front_right /= peak;
    w.back_left /= peak;
    w.back_right /= peak;
  }
  return w;
}

RobotFrameInput fieldToRobot(double field_x, double field_y, double heading_rad) {
  const double x = finiteOrZero(field_x);
  const double y = finiteOrZero(field_y);
  if (!std::isfinite(heading_rad)) {
    return RobotFrameInput{y, x};
  }
  const double c = std::cos(heading_rad);
  const double s = std::sin(heading_rad);
  // Robot forward axis in the field is (sin, cos); robot right axis is
  // (cos, -sin). Same matrix as mclib::fieldToRobot().
  return RobotFrameInput{x * s + y * c, x * c - y * s};
}

}  // namespace holonomic
}  // namespace mclib
