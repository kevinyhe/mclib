// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Closed-loop tests for HolonomicController on the real HolonomicChassis and
// device::MotorGroup, linked over the libpros stand-ins in
// tests/support/host_devices.cpp.
//
// Each tick the controller writes four wheel voltages. A kinematic model
// turns them back into a body velocity (forward, strafe, clockwise turn),
// integrates a field pose and publishes it as the odometry. A sign error
// anywhere in the chain -- field-to-robot rotation, the mix, the heading
// error -- drives the model away from the target and the move times out.
#include "mclib/chassis/holonomic_chassis.hpp"
#include "mclib/chassis/holonomic_controller.hpp"
#include "mclib/command/commandScheduler.h"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/device/controller.hpp"
#include "mclib/math.hpp"
#include "mclib/time.hpp"
#include "pros/motors.h"
#include "support/host_devices.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace {

using mclib::HolonomicChassis;
using mclib::HolonomicController;
using mclib::Pose2D;
using mclib::holonomic::Kind;
namespace hd = mclib::test::host_devices;
namespace units = mclib::units;

constexpr double kDeg = mclib::kPi / 180.0;
constexpr int kTickMs = 10;
constexpr int kHold = static_cast<int>(pros::E_MOTOR_BRAKE_HOLD);

// A settled move ends inside the default exit bands: the "big" band is
// 1.5 in and 3 deg held for 250 ms, and it is what ends most moves here,
// since the loop's output is small that close to the target.
const double kMaxMissIn = mclib::HolonomicControllerConfig{}.translation_exit.big_error;
const double kMaxMissDeg = mclib::HolonomicControllerConfig{}.heading_exit.big_error;

// Ports: one motor per corner, all forward so the stand-in's volts are the
// wheel commands as the controller wrote them.
constexpr int kFL = 1, kFR = 2, kBL = 3, kBR = 4;

std::uint32_t now_ms = 0;
std::uint32_t fakeClock() { return now_ms; }

double wheel(int port) { return hd::motors[port].volts; }

bool allWheelsZero() {
  return wheel(kFL) == 0.0 && wheel(kFR) == 0.0 && wheel(kBL) == 0.0 &&
         wheel(kBR) == 0.0;
}

bool anyWheelDriven() {
  return std::fabs(wheel(kFL)) > 0.1 || std::fabs(wheel(kFR)) > 0.1 ||
         std::fabs(wheel(kBL)) > 0.1 || std::fabs(wheel(kBR)) > 0.1;
}

int brakeCalls() {
  return hd::motors[kFL].brake_calls + hd::motors[kFR].brake_calls +
         hd::motors[kBL].brake_calls + hd::motors[kBR].brake_calls;
}

bool allHold() {
  return hd::motors[kFL].brake_mode == kHold && hd::motors[kFR].brake_mode == kHold &&
         hd::motors[kBL].brake_mode == kHold && hd::motors[kBR].brake_mode == kHold;
}

/// Heading difference a - b wrapped to (-180, 180], degrees.
double headingDiffDeg(double a_rad, double b_rad) {
  return mclib::wrapAngle(a_rad - b_rad) / kDeg;
}

/**
 * A holonomic base with a first-order velocity lag.
 *
 * Wheel volts go through the inverse of holonomic::mix() to get forward,
 * strafe and turn volts. Forward moves 3 in/s per volt (a 200 rpm cartridge
 * on a 4 in wheel at 45 deg, roughly). Strafe is scaled by `strafe_gain`:
 * 1 for an X-drive, less for mecanum, whose rollers slip. Turn is the
 * wheel's surface speed over a 7 in radius.
 */
struct Robot {
  Pose2D pose;
  double strafe_gain = 1.0;
  double forward_ips = 0.0;
  double strafe_ips = 0.0;
  double turn_rps = 0.0;  // clockwise positive

  static constexpr double kInPerSecPerVolt = 3.0;
  static constexpr double kTurnRadiusIn = 7.0;
  static constexpr double kLagSec = 0.06;

  void place(Pose2D start) {
    pose = start;
    forward_ips = strafe_ips = turn_rps = 0.0;
    mclib::control::resetOdometry(pose);
  }

  void step(double dt) {
    const double fl = wheel(kFL), fr = wheel(kFR), bl = wheel(kBL), br = wheel(kBR);
    const double forward_v = (fl + fr + bl + br) / 4.0;
    const double strafe_v = (fl - fr - bl + br) / 4.0;
    const double turn_v = (fl - fr + bl - br) / 4.0;

    const double alpha = dt / kLagSec;
    forward_ips += (forward_v * kInPerSecPerVolt - forward_ips) * alpha;
    strafe_ips += (strafe_v * kInPerSecPerVolt * strafe_gain - strafe_ips) * alpha;
    turn_rps += (turn_v * kInPerSecPerVolt / kTurnRadiusIn - turn_rps) * alpha;

    // Compass frame: robot forward points along (sin t, cos t) on the field,
    // robot right along (cos t, -sin t).
    const double mid = pose.theta + turn_rps * dt / 2.0;
    pose.x += (forward_ips * std::sin(mid) + strafe_ips * std::cos(mid)) * dt;
    pose.y += (forward_ips * std::cos(mid) - strafe_ips * std::sin(mid)) * dt;
    pose.theta += turn_rps * dt;
    mclib::control::resetOdometry(pose);
  }
};

struct Rig {
  HolonomicChassis chassis;
  HolonomicController controller;
  Robot robot;

  explicit Rig(Kind kind)
      : chassis({kFL}, {kFR}, {kBL}, {kBR}, mclib::device::Gearset::Green, kind),
        controller(chassis) {
    robot.strafe_gain = kind == Kind::Mecanum ? 0.75 : 1.0;
  }

  /// One scheduler tick: the controller writes the wheels, then the robot moves.
  void tick(bool move_robot = true) {
    now_ms += kTickMs;
    controller.periodic();
    if (move_robot) {
      robot.step(kTickMs / 1000.0);
    }
  }

  /// Tick until the goal ends or @p limit_ms passes. Returns the time it took.
  std::uint32_t runUntilDone(std::uint32_t limit_ms, bool move_robot = true) {
    const std::uint32_t start = now_ms;
    while (controller.isActive() && now_ms - start < limit_ms) {
      tick(move_robot);
    }
    return now_ms - start;
  }
};

void resetWorld() {
  now_ms = 1000;
  hd::reset();
  mclib::test::setCompetitionStatus(0);
  mclib::control::resetOdometry({0.0, 0.0, 0.0});
}

const char* kindName(Kind kind) {
  return kind == Kind::Mecanum ? "mecanum" : "x-drive";
}

// ---------------------------------------------------------------------------

/// moveToPose from @p start to @p target converges well inside the timeout
/// and ends held at the target.
void checkPoseConverges(Kind kind, Pose2D start, Pose2D target) {
  resetWorld();
  Rig rig(kind);
  rig.robot.place(start);
  const double start_distance = std::hypot(target.x - start.x, target.y - start.y);

  rig.controller.moveToPose(target, 5.0 * units::second);
  CHECK(rig.controller.isActive());
  CHECK(!rig.controller.isSettled());

  // After 300 ms the robot must be heading the right way: closer to the target.
  for (int i = 0; i < 30; ++i) rig.tick();
  const double early = std::hypot(target.x - rig.robot.pose.x, target.y - rig.robot.pose.y);
  if (!CHECK(start_distance < 1e-9 || early < start_distance)) {
    std::printf("    %s (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg): "
                "%.2f in away after 300 ms, started %.2f in away\n",
                kindName(kind), start.x, start.y, start.theta / kDeg, target.x,
                target.y, target.theta / kDeg, early, start_distance);
  }

  const std::uint32_t took = 300 + rig.runUntilDone(5000);
  const double miss = std::hypot(target.x - rig.robot.pose.x, target.y - rig.robot.pose.y);
  const double heading_miss = headingDiffDeg(target.theta, rig.robot.pose.theta);

  const bool ok = CHECK(!rig.controller.isActive()) & CHECK(rig.controller.isSettled()) &
                  CHECK(took < 5000) & CHECK(miss <= kMaxMissIn) &
                  CHECK(std::fabs(heading_miss) <= kMaxMissDeg);
  if (!ok) {
    std::printf("    %s (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg): "
                "took %u ms, ended (%.2f, %.2f, %.1f deg), miss %.2f in, %.2f deg\n",
                kindName(kind), start.x, start.y, start.theta / kDeg, target.x,
                target.y, target.theta / kDeg, took, rig.robot.pose.x,
                rig.robot.pose.y, rig.robot.pose.theta / kDeg, miss, heading_miss);
  }

  // stop_at_end defaults to true: the drive is braked and held.
  CHECK(allWheelsZero());
  CHECK(allHold());
  CHECK(brakeCalls() >= 4);
}

void testMoveToPoseAllQuadrants(Kind kind) {
  const Pose2D origin{0.0, 0.0, 0.0};
  // One target per quadrant, each with a heading change, clockwise and
  // counter-clockwise.
  checkPoseConverges(kind, origin, {24.0, 24.0, 90.0 * kDeg});
  checkPoseConverges(kind, origin, {-24.0, 24.0, -90.0 * kDeg});
  checkPoseConverges(kind, origin, {-24.0, -24.0, 180.0 * kDeg});
  checkPoseConverges(kind, origin, {24.0, -24.0, 45.0 * kDeg});
  // Pure axis moves.
  checkPoseConverges(kind, origin, {0.0, 30.0, 0.0});
  checkPoseConverges(kind, origin, {30.0, 0.0, 0.0});
  checkPoseConverges(kind, origin, {0.0, -30.0, 0.0});
  checkPoseConverges(kind, origin, {-30.0, 0.0, 0.0});
  // Start already turned, so field-to-robot rotation matters from tick one.
  checkPoseConverges(kind, {10.0, -5.0, 135.0 * kDeg}, {-20.0, 20.0, -45.0 * kDeg});
  checkPoseConverges(kind, {0.0, 0.0, 90.0 * kDeg}, {0.0, 24.0, 90.0 * kDeg});
  checkPoseConverges(kind, {0.0, 0.0, -60.0 * kDeg}, {18.0, -12.0, 30.0 * kDeg});
  // Turn in place.
  checkPoseConverges(kind, origin, {0.0, 0.0, 120.0 * kDeg});
}

/// moveToPoint drives to (x, y) and keeps the heading the robot started with.
void testMoveToPointAllQuadrants(Kind kind) {
  const double quadrants[4][2] = {{24, 24}, {-24, 24}, {-24, -24}, {24, -24}};
  for (const auto& q : quadrants) {
    resetWorld();
    Rig rig(kind);
    const Pose2D start{3.0, -2.0, 30.0 * kDeg};
    rig.robot.place(start);
    rig.controller.moveToPoint(q[0] * units::inch, q[1] * units::inch, 5.0 * units::second);
    const std::uint32_t took = rig.runUntilDone(5000);
    const double miss = std::hypot(q[0] - rig.robot.pose.x, q[1] - rig.robot.pose.y);
    const double heading_miss = headingDiffDeg(start.theta, rig.robot.pose.theta);
    const bool ok = CHECK(rig.controller.isSettled()) & CHECK(took < 5000) &
                    CHECK(miss <= kMaxMissIn) & CHECK(std::fabs(heading_miss) <= kMaxMissDeg);
    if (!ok) {
      std::printf("    %s moveToPoint(%.0f, %.0f): took %u ms, miss %.2f in, %.2f deg\n",
                  kindName(kind), q[0], q[1], took, miss, heading_miss);
    }
    CHECK(allWheelsZero());
    CHECK(allHold());
  }
}

/// 170 deg to -170 deg is a 20 deg clockwise turn, not 340 deg the other way.
void testHeadingTakesShortWay(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.robot.place({0.0, 0.0, 170.0 * kDeg});
  rig.controller.moveToPose({0.0, 0.0, -170.0 * kDeg}, 3.0 * units::second);
  double lowest_deg = 170.0;
  while (rig.controller.isActive() && now_ms < 1000 + 3000) {
    rig.tick();
    lowest_deg = std::min(lowest_deg, rig.robot.pose.theta / kDeg);
  }
  CHECK(rig.controller.isSettled());
  // The model's theta is unwrapped: the short way ends near +190.
  CHECK_NEAR(rig.robot.pose.theta / kDeg, 190.0, kMaxMissDeg);
  CHECK(lowest_deg > 165.0);
}

/// stop_at_end = false ends on zero volts, not a brake.
void testNoStopAtEnd(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.moveToPose({12.0, 12.0, 0.0}, 5.0 * units::second, false);
  rig.runUntilDone(5000);
  CHECK(rig.controller.isSettled());
  CHECK(allWheelsZero());
  CHECK_EQ(brakeCalls(), 0);
}

/// A robot that never moves runs out the timeout, then is braked and held.
void testTimeout(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.moveToPose({24.0, 24.0, 0.0}, 500.0 * units::millisecond, false);
  rig.tick(false);
  CHECK(anyWheelDriven());
  const std::uint32_t took = rig.runUntilDone(5000, false) + kTickMs;
  CHECK(!rig.controller.isActive());
  CHECK(rig.controller.isSettled());
  CHECK_EQ(took, 500);
  // A timeout brakes even with stop_at_end = false.
  CHECK(allWheelsZero());
  CHECK(allHold());
  CHECK(brakeCalls() >= 4);

  // Nothing is written after the goal ends.
  const int calls = brakeCalls();
  for (int i = 0; i < 10; ++i) rig.tick(false);
  CHECK(allWheelsZero());
  CHECK_EQ(brakeCalls(), calls);
}

/// A zero timeout means none: the stuck robot keeps trying.
void testZeroTimeoutKeepsTrying(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.moveToPose({24.0, 24.0, 0.0});
  rig.runUntilDone(10000, false);
  CHECK(rig.controller.isActive());
  CHECK(anyWheelDriven());
  rig.controller.cancel();
}

void testCancelStopsDrive(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.moveToPose({-30.0, 20.0, 45.0 * kDeg}, 5.0 * units::second, false);
  for (int i = 0; i < 20; ++i) rig.tick();
  CHECK(rig.controller.isActive());
  CHECK(anyWheelDriven());

  rig.controller.cancel();
  CHECK(!rig.controller.isActive());
  CHECK(rig.controller.isSettled());
  CHECK(allWheelsZero());
  CHECK(allHold());

  for (int i = 0; i < 20; ++i) rig.tick();
  CHECK(allWheelsZero());

  // Cancel while idle writes nothing.
  const int calls = brakeCalls();
  rig.controller.cancel();
  CHECK_EQ(brakeCalls(), calls);
}

void testOnDisabledStopsDrive(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.moveToPose({30.0, -10.0, 0.0}, 5.0 * units::second);
  for (int i = 0; i < 20; ++i) rig.tick();
  CHECK(anyWheelDriven());

  rig.controller.onDisabled();
  CHECK(!rig.controller.isActive());
  CHECK(allWheelsZero());
  for (int i = 0; i < 20; ++i) rig.tick();
  CHECK(allWheelsZero());

  // Idle: onDisabled() still stops the drive, whatever wrote it last.
  rig.chassis.drive(0.5, 0.0, 0.0);
  CHECK(anyWheelDriven());
  rig.controller.onDisabled();
  CHECK(allWheelsZero());
}

/// The same through the scheduler: the move command runs from periodic(),
/// finishes when settled, and a disable mid-move stops the drive.
void testThroughScheduler(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.registerSelf();

  auto command = rig.controller.makeMoveToPoseCommand({20.0, 20.0, 90.0 * kDeg},
                                                      5.0 * units::second);
  command->schedule();
  int ticks = 0;
  while (command->scheduled() && ticks < 600) {
    now_ms += kTickMs;
    CommandScheduler::run();
    rig.robot.step(kTickMs / 1000.0);
    ++ticks;
  }
  CHECK(!command->scheduled());
  CHECK(ticks < 500);  // settled, not timed out
  CHECK(std::hypot(20.0 - rig.robot.pose.x, 20.0 - rig.robot.pose.y) <= kMaxMissIn);
  CHECK(allWheelsZero());

  auto second = rig.controller.makeMoveToPointCommand(0.0 * units::inch, 0.0 * units::inch,
                                                      5.0 * units::second);
  second->schedule();
  for (int i = 0; i < 20; ++i) {
    now_ms += kTickMs;
    CommandScheduler::run();
    rig.robot.step(kTickMs / 1000.0);
  }
  CHECK(second->scheduled());
  CHECK(anyWheelDriven());

  mclib::test::setCompetitionStatus(1);  // disabled
  now_ms += kTickMs;
  CommandScheduler::run();
  CHECK(!second->scheduled());
  CHECK(!rig.controller.isActive());
  CHECK(allWheelsZero());

  mclib::test::setCompetitionStatus(0);
  CommandScheduler::unregisterSubsystem(&rig.controller);
  CommandScheduler::forgetCommand(command.get());
  CommandScheduler::forgetCommand(second.get());
}

/// A non-finite target ends the goal on its first tick with the drive held.
void testNonFiniteTarget(Kind kind) {
  resetWorld();
  Rig rig(kind);
  rig.controller.moveToPose({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                            5.0 * units::second, false);
  rig.tick();
  CHECK(!rig.controller.isActive());
  CHECK(allWheelsZero());
  CHECK(allHold());
}

/// Field-centric teleop: stick up is field +Y whichever way the robot faces.
void testFieldCentricTeleop(Kind kind) {
  resetWorld();
  Rig rig(kind);
  mclib::device::Controller pad;
  // Facing field +X, so field +Y is to the robot's left.
  rig.robot.place({0.0, 0.0, 90.0 * kDeg});
  auto drive = rig.controller.makeDriveCommand(pad);
  hd::analog[static_cast<int>(pros::E_CONTROLLER_ANALOG_LEFT_Y)] = 127;
  drive->initialize();
  for (int i = 0; i < 50; ++i) {
    now_ms += kTickMs;
    drive->execute();
    rig.robot.step(kTickMs / 1000.0);
  }
  CHECK(rig.robot.pose.y > 5.0);
  CHECK(std::fabs(rig.robot.pose.x) < 0.5);
  CHECK(std::fabs(headingDiffDeg(rig.robot.pose.theta, 90.0 * kDeg)) < 1.0);
  drive->end(true);
  CHECK(allWheelsZero());
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock(fakeClock);
  for (const Kind kind : {Kind::XDrive, Kind::Mecanum}) {
    testMoveToPoseAllQuadrants(kind);
    testMoveToPointAllQuadrants(kind);
    testHeadingTakesShortWay(kind);
    testNoStopAtEnd(kind);
    testTimeout(kind);
    testZeroTimeoutKeepsTrying(kind);
    testCancelStopsDrive(kind);
    testOnDisabledStopsDrive(kind);
    testThroughScheduler(kind);
    testNonFiniteTarget(kind);
    testFieldCentricTeleop(kind);
  }
  return mclib::test::summary("holonomic_controller_test");
}
