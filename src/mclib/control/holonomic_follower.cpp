// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/holonomic_follower.hpp"

#include <algorithm>
#include <cmath>

namespace mclib {
namespace control {

namespace {

double sgn(double x) { return (x > 0.0) - (x < 0.0); }

double linear(const FeedforwardGains& gains, double velocity_base) {
  return gains.kS.raw() * sgn(velocity_base) + gains.kV.raw() * velocity_base;
}

}  // namespace

HolonomicTarget holonomicTarget(const path::TrajectoryState& state,
                                units::QAngle start, units::QAngle end,
                                units::QTime duration, units::QTime t) {
  HolonomicTarget out;
  out.state = state;
  const double turn = wrapAngle(end.rad() - start.rad());
  const double total = duration.raw();
  if (!(total > 0.0) || !(t.raw() < total)) {
    out.heading = units::QAngle::fromBase(wrapAngle(start.rad() + turn));
    return out;
  }
  const double frac = std::max(0.0, t.raw() / total);
  out.heading = units::QAngle::fromBase(wrapAngle(start.rad() + turn * frac));
  out.heading_rate = units::QAngularVelocity::fromBase(t.raw() < 0.0 ? 0.0 : turn / total);
  return out;
}

HolonomicVolts holonomicFollowStep(const Pose2D& pose,
                                   const HolonomicTarget& target,
                                   const HolonomicFollowerConfig& config) {
  const path::TrajectoryState& s = target.state;
  // Field velocity, in/s: along the path tangent, plus position correction.
  // The tangent is the path heading; the trajectory's velocity is signed, so
  // a reversed plan still points the right way.
  const double v = s.velocity.inps();
  const double tangent = s.heading.rad();
  const double field_x =
      v * std::sin(tangent) + config.translation_kp * (s.x.in() - pose.x);
  const double field_y =
      v * std::cos(tangent) + config.translation_kp * (s.y.in() - pose.y);

  // Compass frame: forward is (sin, cos), right is (cos, -sin).
  const double sn = std::sin(pose.theta);
  const double cs = std::cos(pose.theta);
  const double forward_ips = field_x * sn + field_y * cs;
  const double strafe_ips = field_x * cs - field_y * sn;

  const double heading_error = wrapAngle(target.heading.rad() - pose.theta);
  const double turn_radps = target.heading_rate.raw() + config.heading_kp * heading_error;

  const HolonomicFeedforward& ff = config.feedforward;
  HolonomicVolts out;
  out.forward = linear(ff.forward, (forward_ips * units::inps).raw());
  out.strafe = linear(ff.strafe, (strafe_ips * units::inps).raw());
  out.turn = ff.turn.kS.raw() * sgn(turn_radps) + ff.turn.kV.raw() * turn_radps;
  return out;
}

}  // namespace control
}  // namespace mclib
