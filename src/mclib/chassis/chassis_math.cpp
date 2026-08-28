// mclib
#include "mclib/chassis/chassis_math.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace chassis_math {

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

}  // namespace chassis_math
}  // namespace mclib
