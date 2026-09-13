// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/drive_curve.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace control {

namespace {

/// @brief A config parameter that must lie in [0, 1); anything else is "off".
double unitParameter(double value) {
  return (std::isfinite(value) && value >= 0.0 && value < 1.0) ? value : 0.0;
}

}  // namespace

double shapeDriveInput(double input, const DriveCurveConfig& config) {
  if (std::isnan(input)) {
    return 0.0;
  }
  const double x = std::clamp(input, -1.0, 1.0);
  const double sign = x < 0.0 ? -1.0 : 1.0;
  double magnitude = std::fabs(x);

  // 1. Deadzone, then stretch what is left back onto [0, 1] so the output
  //    starts from 0 at the edge of the deadzone rather than jumping to it.
  const double deadzone = unitParameter(config.deadzone);
  if (magnitude <= deadzone) {
    return 0.0;
  }
  magnitude = (magnitude - deadzone) / (1.0 - deadzone);

  // 2. The exponential curve. Written on the magnitude and re-signed below,
  //    which is the same as the |x| form in the header and keeps it odd.
  const double gain = (std::isfinite(config.gain) && config.gain > 0.0) ? config.gain : 0.0;
  if (gain > 0.0) {
    const double base = std::exp(-gain / 10.0);
    magnitude = (base + std::exp((magnitude - 1.0) * gain / 10.0) * (1.0 - base)) * magnitude;
  }

  // 3. Lift the whole range so the smallest non-zero command still moves the
  //    robot. A rescale rather than a floor: a floor would flatten every
  //    input below it onto the same output, a rescale keeps the curve
  //    monotonic.
  const double min_output = unitParameter(config.min_output);
  if (min_output > 0.0) {
    magnitude = min_output + (1.0 - min_output) * magnitude;
  }

  // 4. Clamp. Nothing above can exceed 1 in exact arithmetic, but the curve
  //    is two exp() calls and the caller is promised [-1, 1].
  return sign * std::clamp(magnitude, 0.0, 1.0);
}

DriveCurve::DriveCurve(DriveCurveConfig config) : m_config(config) {}

double DriveCurve::apply(double input) {
  const double target = shapeDriveInput(input, m_config);
  const double slew = m_config.slew_per_tick;
  if (!(std::isfinite(slew) && slew > 0.0)) {
    m_last_output = target;
    return target;
  }
  const double step = std::clamp(target - m_last_output, -slew, slew);
  m_last_output += step;
  return m_last_output;
}

void DriveCurve::reset() {
  m_last_output = 0.0;
}

const DriveCurveConfig& DriveCurve::config() const {
  return m_config;
}

void DriveCurve::setConfig(const DriveCurveConfig& config) {
  m_config = config;
}

}  // namespace control
}  // namespace mclib
