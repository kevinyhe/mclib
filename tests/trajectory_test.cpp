// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file trajectory_test.cpp
 * @brief Host tests for Trajectory::generate() and sample().
 *
 * The straight-line case has a closed form (the same trapezoid as
 * profile_test.cpp), so it checks the timing. The curved cases check that
 * every limit holds at every planned sample.
 */

#include "mclib/math.hpp"
#include "mclib/path/path.hpp"
#include "mclib/path/spline.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace mclib::path;
using namespace mclib::units;
using namespace mclib::units::literals;

namespace {

const QAcceleration inps2 = inch / (second * second);

Path straight48() {
  return Path::fromWaypoints({Waypoint{0_in, 0_in}, Waypoint{0_in, 48_in}});
}

// An S-bend: right, then left.
Path sBend() {
  return generateSpline({Waypoint{0_in, 0_in}, Waypoint{12_in, 24_in},
                         Waypoint{0_in, 48_in}, Waypoint{-12_in, 72_in}});
}

TrajectoryConstraints limits() {
  TrajectoryConstraints c;
  c.max_velocity = 48 * inps;
  c.max_acceleration = 96 * inps2;
  return c;
}

void checkLimits(const Trajectory& traj, const TrajectoryConstraints& c) {
  const double vmax = c.max_velocity.raw();
  const double accel = c.max_acceleration.raw();
  const double decel =
      c.max_deceleration.raw() > 0 ? c.max_deceleration.raw() : accel;
  const double eps = 1e-9;
  bool speed_ok = true, accel_ok = true, lateral_ok = true, wheel_ok = true;
  bool time_ok = true;
  const auto& s = traj.states();
  for (std::size_t i = 0; i < s.size(); ++i) {
    const double v = std::fabs(s[i].velocity.raw());
    const double k = std::fabs(s[i].curvature.raw());
    speed_ok &= v <= vmax + eps;
    if (c.max_lateral_acceleration.raw() > 0)
      lateral_ok &= v * v * k <= c.max_lateral_acceleration.raw() + 1e-6;
    if (c.track_width.raw() > 0)
      wheel_ok &= v * (1 + k * c.track_width.raw() / 2) <= vmax + 1e-6;
    if (i + 1 < s.size()) {
      const double ds = (s[i + 1].distance - s[i].distance).raw();
      const double v1 = std::fabs(s[i + 1].velocity.raw());
      const double a = (v1 * v1 - v * v) / (2 * ds);
      accel_ok &= a <= accel + 1e-6 && a >= -decel - 1e-6;
      time_ok &= s[i + 1].time > s[i].time;
    }
  }
  CHECK(speed_ok);
  CHECK(accel_ok);
  CHECK(lateral_ok);
  CHECK(wheel_ok);
  CHECK(time_ok);
}

}  // namespace

int main() {
  std::printf("-- invalid input gives an empty trajectory\n");
  {
    CHECK(Trajectory::generate(Path{}, limits()).empty());
    TrajectoryConstraints c = limits();
    c.max_velocity = QVelocity{};
    CHECK(Trajectory::generate(straight48(), c).empty());
    c = limits();
    c.max_acceleration = -1 * inps2;
    CHECK(Trajectory::generate(straight48(), c).empty());
    c = limits();
    c.end_velocity = QVelocity::fromBase(NAN);
    CHECK(Trajectory::generate(straight48(), c).empty());
    const Trajectory empty;
    CHECK_EQ(empty.duration().raw(), 0.0);
    CHECK_EQ(empty.sample(1_s).velocity.raw(), 0.0);
  }

  std::printf("-- straight 48 in: 0.5 s up, 0.5 s cruise, 0.5 s down\n");
  {
    const Trajectory traj = Trajectory::generate(straight48(), limits());
    CHECK(!traj.empty());
    std::printf("   duration = %.4f s, length = %.4f in, samples = %zu\n",
                traj.duration().s(), traj.length().in(), traj.states().size());
    CHECK_NEAR(traj.duration().s(), 1.5, 0.01);
    CHECK_NEAR(traj.length().in(), 48.0, 1e-9);
    CHECK_EQ(traj.states().front().velocity.raw(), 0.0);
    CHECK_EQ(traj.states().back().velocity.raw(), 0.0);
    const TrajectoryState mid = traj.sample(0.75_s);
    std::printf("   at 0.75 s: %.4f in, %.4f in/s\n", mid.distance.in(),
                mid.velocity.inps());
    CHECK_NEAR(mid.distance.in(), 24.0, 0.2);
    CHECK_NEAR(mid.velocity.inps(), 48.0, 0.2);
    CHECK_NEAR(mid.x.in(), 0.0, 1e-9);
    CHECK_NEAR(mid.y.in(), mid.distance.in(), 1e-9);
    CHECK_NEAR(mid.heading.rad(), 0.0, 1e-12);
    const TrajectoryState up = traj.sample(0.25_s);
    CHECK_NEAR(up.acceleration.raw(), (96 * inps2).raw(), 1e-6);
    CHECK_NEAR(up.velocity.inps(), 24.0, 0.2);
    checkLimits(traj, limits());

    // Clamped outside [0, duration].
    CHECK_EQ(traj.sample(-1_s).distance.raw(), 0.0);
    CHECK_NEAR(traj.sample(10_s).distance.in(), 48.0, 1e-9);

    // Integrating the sampled speed over time covers the path.
    double travelled = 0;
    for (double t = 0; t < traj.duration().s(); t += 0.001)
      travelled += traj.sample(QTime::fromBase(t)).velocity.inps() * 0.001;
    std::printf("   integrated distance = %.4f in\n", travelled);
    CHECK_NEAR(travelled, 48.0, 0.1);

    // sample() is continuous and never goes backward.
    bool monotonic = true;
    double last = -1;
    for (double t = 0; t <= traj.duration().s(); t += 0.0037) {
      const double d = traj.sample(QTime::fromBase(t)).distance.in();
      monotonic &= d >= last;
      last = d;
    }
    CHECK(monotonic);
  }

  std::printf("-- separate deceleration limit\n");
  {
    TrajectoryConstraints c = limits();
    c.max_deceleration = 48 * inps2;
    const Trajectory traj = Trajectory::generate(straight48(), c);
    // 0.5 s up (12 in), 1.0 s down (24 in), 12 in cruise at 48 in/s (0.25 s).
    std::printf("   duration = %.4f s (expect 1.75)\n", traj.duration().s());
    CHECK_NEAR(traj.duration().s(), 1.75, 0.01);
    checkLimits(traj, c);
  }

  std::printf("-- chaining: start and end speeds\n");
  {
    TrajectoryConstraints c = limits();
    c.start_velocity = 48 * inps;
    c.end_velocity = 24 * inps;
    const Trajectory traj = Trajectory::generate(straight48(), c);
    CHECK_NEAR(traj.states().front().velocity.inps(), 48.0, 1e-9);
    CHECK_NEAR(traj.states().back().velocity.inps(), 24.0, 1e-9);
    checkLimits(traj, c);
    // A start speed above the limit is capped, not honoured.
    c.start_velocity = 100 * inps;
    CHECK_NEAR(Trajectory::generate(straight48(), c).states().front().velocity.inps(),
               48.0, 1e-9);
  }

  std::printf("-- S-bend with cornering and wheel limits\n");
  {
    TrajectoryConstraints c = limits();
    c.max_lateral_acceleration = 60 * inps2;
    c.track_width = 12_in;
    const Path path = sBend();
    const Trajectory traj = Trajectory::generate(path, c);
    CHECK(!traj.empty());
    checkLimits(traj, c);
    double slowest_cruise = 1e9;
    for (const auto& s : traj.states())
      if (s.distance.in() > 12 && s.distance.in() < path.length().in() - 12)
        slowest_cruise = std::fmin(slowest_cruise, s.velocity.inps());
    std::printf("   duration = %.4f s, slowest mid-path speed = %.4f in/s\n",
                traj.duration().s(), slowest_cruise);
    CHECK(slowest_cruise < 48.0);  // the corners did slow it down
    const Trajectory loose = Trajectory::generate(path, limits());
    CHECK(traj.duration() > loose.duration());

    // Positions and headings match the path.
    bool on_path = true;
    for (const auto& s : traj.states()) {
      const PathPoint p = path.atDistance(s.distance);
      on_path &= std::fabs((s.x - p.x).in()) < 1e-9 &&
                 std::fabs((s.y - p.y).in()) < 1e-9 &&
                 std::fabs(mclib::wrapAngle((s.heading - p.heading).rad())) < 1e-9;
    }
    CHECK(on_path);
  }

  std::printf("-- reversed\n");
  {
    TrajectoryConstraints c = limits();
    c.max_lateral_acceleration = 60 * inps2;
    const Path path = sBend();
    const Trajectory fwd = Trajectory::generate(path, c);
    c.reversed = true;
    const Trajectory rev = Trajectory::generate(path, c);
    CHECK_EQ(fwd.states().size(), rev.states().size());
    CHECK_NEAR(fwd.duration().s(), rev.duration().s(), 1e-12);
    bool mirrored = true;
    for (std::size_t i = 0; i < fwd.states().size(); ++i) {
      const auto& f = fwd.states()[i];
      const auto& r = rev.states()[i];
      mirrored &= r.velocity.raw() <= 0 &&
                  std::fabs(r.velocity.raw() + f.velocity.raw()) < 1e-12 &&
                  std::fabs(mclib::wrapAngle((r.heading - f.heading).rad() - M_PI)) < 1e-9 &&
                  // Same path, same direction along it: same turn rate.
                  std::fabs((r.angularVelocity() - f.angularVelocity()).raw()) < 1e-9;
    }
    CHECK(mirrored);
    const TrajectoryState mid = rev.sample(rev.duration() * 0.5);
    CHECK(mid.velocity.raw() < 0);
    CHECK_NEAR(mid.distance.in(), fwd.sample(fwd.duration() * 0.5).distance.in(), 1e-9);
  }

  return mclib::test::summary("trajectory");
}
