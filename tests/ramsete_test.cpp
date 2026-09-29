// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file ramsete_test.cpp
 * @brief Host tests for the RAMSETE control law, and a closed-loop check
 *        against a tank drive that slips.
 *
 * The slip model is the failure `driveTo()` and `curveCircle()` have: one
 * side covers less ground than it was told to. Following the trajectory open
 * loop drifts off; RAMSETE sees the drift in the pose and corrects it.
 */

#include "mclib/control/ramsete.hpp"
#include "mclib/math.hpp"
#include "mclib/path/spline.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>

using namespace mclib;
using namespace mclib::control;
using namespace mclib::path;
using namespace mclib::units;
using namespace mclib::units::literals;

namespace {

const QLength kTrack = 12_in;

Trajectory sBend() {
  TrajectoryConstraints c;
  c.max_velocity = 40 * inps;
  c.max_acceleration = 80 * inps2;
  c.max_lateral_acceleration = 60 * inps2;
  c.track_width = kTrack;
  return Trajectory::generate(
      generateSpline({Waypoint{0_in, 0_in}, Waypoint{12_in, 24_in},
                      Waypoint{0_in, 48_in}, Waypoint{-12_in, 72_in}}),
      c);
}

/// Run a tank drive through @p traj. @p left_slip / @p right_slip scale how
/// much of each commanded wheel speed reaches the ground. Returns the final
/// position error in inches.
double simulate(const Trajectory& traj, bool use_ramsete, double left_slip,
                double right_slip, Pose2D pose, RamseteGains gains = {}) {
  const Ramsete ramsete(gains);
  const double dt = 0.01;
  for (double t = 0; t <= traj.duration().s() + dt; t += dt) {
    const TrajectoryState target = traj.sample(QTime::fromBase(t));
    RamseteOutput cmd{target.velocity, target.angularVelocity()};
    if (use_ramsete) cmd = ramsete.calculate(pose, target);
    const WheelSpeeds wheels = tankWheelSpeeds(cmd, kTrack);
    const double l = wheels.left.inps() * left_slip;
    const double r = wheels.right.inps() * right_slip;
    const double v = (l + r) / 2;
    const double w = (l - r) / kTrack.in();
    // Midpoint heading, so the model's own error stays small next to the
    // errors being measured.
    const double mid = pose.theta + w * dt / 2;
    pose.x += v * std::sin(mid) * dt;
    pose.y += v * std::cos(mid) * dt;
    pose.theta = wrapAngle(pose.theta + w * dt);
  }
  const TrajectoryState end = traj.states().back();
  return std::hypot(pose.x - end.x.in(), pose.y - end.y.in());
}

}  // namespace

int main() {
  const Ramsete ramsete;

  std::printf("-- on target: output is the trajectory's own speed and turn rate\n");
  {
    const Trajectory traj = sBend();
    const TrajectoryState target = traj.sample(0.8_s);
    const RamseteOutput out = ramsete.calculate(target.pose(), target);
    CHECK_NEAR(out.velocity.raw(), target.velocity.raw(), 1e-12);
    CHECK_NEAR(out.angular_velocity.raw(), target.angularVelocity().raw(), 1e-12);
  }

  std::printf("-- error signs in the compass frame\n");
  {
    TrajectoryState target;
    target.x = 0_in;
    target.y = 10_in;
    target.velocity = 20 * inps;
    // Behind the target: speed up.
    CHECK(ramsete.calculate(Pose2D{0, 8, 0}, target).velocity > target.velocity);
    // Ahead of the target: slow down.
    CHECK(ramsete.calculate(Pose2D{0, 12, 0}, target).velocity < target.velocity);
    // Target is to the robot's right (+X at heading 0): turn clockwise.
    CHECK(ramsete.calculate(Pose2D{-2, 10, 0}, target).angular_velocity.raw() > 0);
    // Target is to the left: turn counter-clockwise.
    CHECK(ramsete.calculate(Pose2D{2, 10, 0}, target).angular_velocity.raw() < 0);
    // Pointing left of the target heading: turn clockwise.
    CHECK(ramsete.calculate(Pose2D{0, 10, -0.2}, target).angular_velocity.raw() > 0);
    // Same checks facing +X (heading 90 deg): the right side is now -Y.
    target.x = 10_in;
    target.y = 0_in;
    target.heading = 90_deg;
    CHECK(ramsete.calculate(Pose2D{10, 2, M_PI / 2}, target).angular_velocity.raw() > 0);
    CHECK(ramsete.calculate(Pose2D{8, 0, M_PI / 2}, target).velocity > target.velocity);
    // Reversed: behind means further along the negative direction.
    target.velocity = -20 * inps;
    target.heading = 0_deg;
    target.x = 0_in;
    target.y = 10_in;
    CHECK(ramsete.calculate(Pose2D{0, 12, 0}, target).velocity < target.velocity);
  }

  std::printf("-- a stopped target gets no correction\n");
  {
    TrajectoryState target;
    target.y = 10_in;
    const RamseteOutput out = ramsete.calculate(Pose2D{0, 0, 0}, target);
    CHECK_EQ(out.velocity.raw(), 0.0);
    CHECK_EQ(out.angular_velocity.raw(), 0.0);
  }

  std::printf("-- closed loop on an S-bend\n");
  {
    const Trajectory traj = sBend();
    CHECK(!traj.empty());
    const Pose2D start = traj.states().front().pose();
    const double open_clean = simulate(traj, false, 1.0, 1.0, start);
    const double open_slip = simulate(traj, false, 0.92, 1.0, start);
    const double ramsete_slip = simulate(traj, true, 0.92, 1.0, start);
    const double stiff_slip =
        simulate(traj, true, 0.92, 1.0, start, RamseteGains{10.0, 0.7});
    const double open_both = simulate(traj, false, 0.85, 0.95, start);
    const double ramsete_both = simulate(traj, true, 0.85, 0.95, start);
    const double ramsete_offset =
        simulate(traj, true, 1.0, 1.0,
                 Pose2D{start.x + 3, start.y - 2, start.theta + 0.17});
    std::printf("   open loop, no slip:              %.3f in\n", open_clean);
    std::printf("   open loop, left 8%% slip:         %.3f in\n", open_slip);
    std::printf("   RAMSETE b=2, left 8%% slip:       %.3f in\n", ramsete_slip);
    std::printf("   RAMSETE b=10, left 8%% slip:      %.3f in\n", stiff_slip);
    std::printf("   open loop, 15%% / 5%% slip:        %.3f in\n", open_both);
    std::printf("   RAMSETE b=2, 15%% / 5%% slip:      %.3f in\n", ramsete_both);
    std::printf("   RAMSETE b=2, start 3.6 in and 10 deg off: %.3f in\n",
                ramsete_offset);
    // Open loop on a perfect drive only misses by the gap between the
    // spline's sampled curvature and its real shape.
    CHECK(open_clean < 2.0);
    CHECK(open_slip > 10.0);
    // A constant slip on one side is a steady push RAMSETE's proportional
    // correction can only partly cancel. Larger b cancels more of it.
    CHECK(ramsete_slip < open_slip / 3);
    CHECK(stiff_slip < 1.5);
    CHECK(ramsete_both < open_both / 3);
    CHECK(ramsete_offset < 1.0);

    // Reversed trajectories are followed backward just as well.
    TrajectoryConstraints c;
    c.max_velocity = 40 * inps;
    c.max_acceleration = 80 * inps2;
    c.max_lateral_acceleration = 60 * inps2;
    c.reversed = true;
    const Trajectory back = Trajectory::generate(
        generateSpline({Waypoint{0_in, 0_in}, Waypoint{12_in, -24_in},
                        Waypoint{0_in, -48_in}}),
        c);
    const Pose2D back_start = back.states().front().pose();
    const double reversed_open = simulate(back, false, 0.92, 1.0, back_start);
    const double reversed_slip = simulate(back, true, 0.92, 1.0, back_start);
    std::printf("   reversed, left 8%% slip: open loop %.3f in, RAMSETE %.3f in\n",
                reversed_open, reversed_slip);
    CHECK(reversed_slip < reversed_open / 2);
    const double reversed_offset =
        simulate(back, true, 1.0, 1.0,
                 Pose2D{back_start.x - 3, back_start.y + 2, back_start.theta - 0.17});
    std::printf("   reversed, start 3.6 in and 10 deg off: %.3f in\n", reversed_offset);
    CHECK(reversed_offset < 1.0);
  }

  std::printf("-- voltages\n");
  {
    const SimpleMotorFeedforward ff(FeedforwardGains{
        1_V, 0.2_V / inps, QVoltagePerAcceleration{}});
    RamseteOutput cmd{30 * inps, QAngularVelocity{}};
    DriveVoltages volts = ramseteVoltages(cmd, QAcceleration{}, kTrack, ff, 12_V);
    CHECK_NEAR(volts.left.volts(), 7.0, 1e-9);
    CHECK_NEAR(volts.right.volts(), 7.0, 1e-9);
    // Turning clockwise at 2 rad/s on a 12 in track: +/-12 in/s per side.
    cmd.angular_velocity = QAngularVelocity::fromBase(2.0);
    volts = ramseteVoltages(cmd, QAcceleration{}, kTrack, ff, 12_V);
    CHECK_NEAR(volts.left.volts(), 1 + 0.2 * 42, 1e-9);
    CHECK_NEAR(volts.right.volts(), 1 + 0.2 * 18, 1e-9);
    // Over the cap: both sides scale by the same factor.
    cmd.velocity = 80 * inps;
    volts = ramseteVoltages(cmd, QAcceleration{}, kTrack, ff, 12_V);
    CHECK_NEAR(volts.left.volts(), 12.0, 1e-9);
    CHECK_NEAR(volts.right.volts() / volts.left.volts(),
               (1 + 0.2 * 68) / (1 + 0.2 * 92), 1e-9);
  }

  return mclib::test::summary("ramsete");
}
