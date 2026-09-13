// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/chassis/chassis_math.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace chassis_math {

double encoderHeadingDeg(double left_distance, double right_distance, double track_width) {
  if (!std::isfinite(left_distance) || !std::isfinite(right_distance) ||
      !std::isfinite(track_width) || track_width <= 0.0) return NAN;
  return (left_distance - right_distance) / track_width * 180.0 / std::acos(-1.0);
}

double normalizeHeadingTarget(double target_deg, double current_deg) {
  const double delta = target_deg - current_deg;
  if (!std::isfinite(delta)) {
    return target_deg;
  }

  // The same repeated +/-360 as control::normalizeTarget(), done in one step.
  // Two full turns of accumulated rotation is 720 deg of loop there; a robot
  // that has been spinning for a while should not cost the caller a loop.
  if (delta > 180.0) {
    return target_deg - 360.0 * std::ceil((delta - 180.0) / 360.0);
  }
  if (delta < -180.0) {
    return target_deg + 360.0 * std::ceil((-delta - 180.0) / 360.0);
  }
  return target_deg;
}

DrivePair mixDriveCorrection(double drive_volts,
                            double correction_volts,
                            double max_volts) {
  if (!(max_volts > 0.0)) {
    return DrivePair{0.0, 0.0};
  }

  // Half the rail of correction is a differential of the whole rail, and it
  // still leaves half the rail of common-mode drive. Letting the correction
  // have more than that buys a harder turn at the price of a move that makes
  // no forward progress at all -- see the header.
  const double correction_ceiling = max_volts * 0.5;
  const double correction =
      std::clamp(correction_volts, -correction_ceiling, correction_ceiling);

  double left = drive_volts + correction;
  double right = drive_volts - correction;

  // Slide the pair as a unit until it fits. The span is 2*|correction|, which
  // is at most max_volts, so one of these shifts always lands both sides
  // inside the rail and the other branch can never also be needed.
  const double high = std::max(left, right);
  const double low = std::min(left, right);
  double shift = 0.0;
  if (high > max_volts) {
    shift = max_volts - high;
  } else if (low < -max_volts) {
    shift = -max_volts - low;
  }

  return DrivePair{left + shift, right + shift};
}

DrivePair arcadeMix(double forward, double turn) {
  return DrivePair{forward + turn, forward - turn};
}

DrivePair curvatureMix(double forward,
                       double turn,
                       bool turn_in_place_when_stopped) {
  if (!std::isfinite(forward) || !std::isfinite(turn)) {
    return DrivePair{0.0, 0.0};
  }
  if (forward == 0.0) {
    if (!turn_in_place_when_stopped) {
      return DrivePair{0.0, 0.0};
    }
    const double spin = std::clamp(turn, -1.0, 1.0);
    return DrivePair{spin, -spin};
  }

  const double differential = std::fabs(forward) * turn;
  double left = forward + differential;
  double right = forward - differential;

  // Fit the rail by scaling both sides by the same factor, which keeps the
  // ratio - and so the curvature the driver asked for - intact.
  const double peak = std::max(std::fabs(left), std::fabs(right));
  if (peak > 1.0) {
    left /= peak;
    right /= peak;
  }
  return DrivePair{left, right};
}

}  // namespace chassis_math
}  // namespace mclib
