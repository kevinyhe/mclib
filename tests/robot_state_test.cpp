// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/robot_state.hpp"

#include "test_assert.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <thread>

// RobotState replaced six bare globals that three tasks wrote at once. The
// interesting property is not that the fields exist, it is that a reader
// cannot pair an x from one odometry tick with a y from the next.

using mclib::Pose2D;
using mclib::control::robotState;

namespace {

constexpr double kPi = mclib::kPi;

void testDefaultsAndRoundTrip() {
  std::printf("-- fields round-trip, setPosition keeps the heading\n");
  robotState().setPose(Pose2D{});
  robotState().clearMotionOutputs();
  robotState().setCorrectAngleDeg(0.0);

  CHECK_EQ(robotState().pose().x, 0.0);
  CHECK_EQ(robotState().pose().y, 0.0);
  CHECK_EQ(robotState().pose().theta, 0.0);
  CHECK(robotState().isTurning() == false);
  CHECK_EQ(robotState().prevLeftOutput(), 0.0);
  CHECK_EQ(robotState().prevRightOutput(), 0.0);
  CHECK_EQ(robotState().correctAngleDeg(), 0.0);

  robotState().setPose(Pose2D{12.5, -3.25, kPi / 4.0});
  CHECK_EQ(robotState().pose().x, 12.5);
  CHECK_EQ(robotState().pose().y, -3.25);
  CHECK_NEAR(robotState().pose().theta, kPi / 4.0, 1e-15);

  // The heading is wrapped on the way in, so a caller cannot smuggle in an
  // unbounded angle that then breaks every downstream atan2 comparison.
  robotState().setPose(Pose2D{0.0, 0.0, 3.0 * kPi});
  CHECK_NEAR(std::fabs(robotState().pose().theta), kPi, 1e-12);

  // wallReset() fixes the position off a wall but leaves the IMU owning the
  // heading.
  robotState().setPose(Pose2D{1.0, 2.0, 0.75});
  robotState().setPosition(-10.0, 40.0);
  const Pose2D pose = robotState().pose();
  CHECK_EQ(pose.x, -10.0);
  CHECK_EQ(pose.y, 40.0);
  CHECK_EQ(pose.theta, 0.75);

  robotState().setPrevOutputs(6.5, -6.5);
  CHECK_EQ(robotState().prevLeftOutput(), 6.5);
  CHECK_EQ(robotState().prevRightOutput(), -6.5);
  robotState().setTurning(true);
  CHECK(robotState().isTurning() == true);
  robotState().setCorrectAngleDeg(-137.5);
  CHECK_EQ(robotState().correctAngleDeg(), -137.5);

  // A cancelled motion puts the drive scalars back to rest and leaves the
  // pose alone -- the robot is still where it is.
  robotState().clearMotionOutputs();
  CHECK(robotState().isTurning() == false);
  CHECK_EQ(robotState().prevLeftOutput(), 0.0);
  CHECK_EQ(robotState().prevRightOutput(), 0.0);
  CHECK_EQ(robotState().pose().x, -10.0);
  CHECK_EQ(robotState().pose().y, 40.0);
}

void testUnitAccessors() {
  std::printf("-- unit-typed accessors agree with the raw ones\n");
  robotState().setPose(Pose2D{24.0, -12.0, kPi / 2.0});
  // Position comes from pose() only - there are no single-axis accessors, so
  // the tearing read shape cannot be written.
  CHECK_NEAR(robotState().pose().x, 24.0, 1e-9);
  CHECK_NEAR(robotState().pose().y, -12.0, 1e-9);
  CHECK_NEAR(robotState().headingRad(), kPi / 2.0, 1e-9);
  CHECK_NEAR(robotState().heading().deg(), 90.0, 1e-9);
}

/**
 * @brief The tearing test.
 *
 * A writer moves the pose along the line y = 2x + 1 as fast as it can. Every
 * pose it publishes satisfies that invariant; no pose between two of them
 * does. A reader that loads x and y separately - which is what
 * `hypot(x - xpos, y - ypos)` in motion.cpp did - lands between them and sees
 * a point off the line. `pose()` returns both from one locked read, so it
 * cannot.
 */
void testPoseReadsCannotTear() {
  std::printf("-- pose() cannot tear x against y\n");
  constexpr int kIterations = 200000;

  std::atomic_bool stop{false};
  std::atomic<long> consistent_reads{0};
  std::atomic<long> atomic_tears{0};

  robotState().setPose(Pose2D{0.0, 1.0, 0.0});

  std::thread writer([&]() {
    for (int i = 0; i < kIterations && !stop.load(); ++i) {
      const double x = static_cast<double>(i % 1000);
      robotState().setPose(Pose2D{x, 2.0 * x + 1.0, 0.0});
    }
    stop.store(true);
  });

  std::thread reader([&]() {
    while (!stop.load()) {
      // The safe read: one call, one lock, both fields.
      const Pose2D pose = robotState().pose();
      if (pose.y == 2.0 * pose.x + 1.0) {
        consistent_reads.fetch_add(1);
      } else {
        atomic_tears.fetch_add(1);
      }
    }
  });

  writer.join();
  reader.join();

  std::printf("   pose():      %ld consistent, %ld torn\n",
              consistent_reads.load(), atomic_tears.load());

  // The property under test: not one of those reads saw a half-updated pose.
  CHECK_EQ(static_cast<double>(atomic_tears.load()), 0.0);
  CHECK(consistent_reads.load() > 0);

  // The separate-loads shape that used to tear here is now unrepresentable:
  // x(), y(), xLength() and yLength() were removed, so pose() is the only way
  // to read a position and it takes one lock for both fields. Nothing to
  // report - the bug cannot be written.
}

void testCancelFlag() {
  std::printf("-- cooperative cancel flag\n");
  using mclib::control::CancelToken;
  mclib::control::clearCancel();
  CHECK(mclib::control::cancelRequested() == false);
  mclib::control::requestCancel();
  CHECK(mclib::control::cancelRequested() == true);

  // Visible from another thread - it is what stops the motion task.
  std::atomic_bool seen{false};
  std::thread observer([&]() { seen.store(mclib::control::cancelRequested()); });
  observer.join();
  CHECK(seen.load() == true);

  mclib::control::clearCancel();
  CHECK(mclib::control::cancelRequested() == false);

  // The two tokens are independent. correctHeading() is built to run alongside
  // a motion, so cancelling a motion must not take the heading hold down with
  // it -- one shared flag would have killed it silently on the first
  // interrupted move and nothing would have restarted it.
  std::printf("-- Motion and HeadingCorrection cancel independently\n");
  mclib::control::clearCancel(CancelToken::Motion);
  mclib::control::clearCancel(CancelToken::HeadingCorrection);

  mclib::control::requestCancel(CancelToken::Motion);
  CHECK(mclib::control::cancelRequested(CancelToken::Motion) == true);
  CHECK(mclib::control::cancelRequested(CancelToken::HeadingCorrection) == false);

  mclib::control::requestCancel(CancelToken::HeadingCorrection);
  mclib::control::clearCancel(CancelToken::Motion);
  CHECK(mclib::control::cancelRequested(CancelToken::Motion) == false);
  CHECK(mclib::control::cancelRequested(CancelToken::HeadingCorrection) == true);

  mclib::control::clearCancel(CancelToken::HeadingCorrection);
  CHECK(mclib::control::cancelRequested(CancelToken::HeadingCorrection) == false);

  // Motion is the default, so the old single-argument spelling still means
  // the motion routines.
  mclib::control::requestCancel();
  CHECK(mclib::control::cancelRequested(CancelToken::Motion) == true);
  mclib::control::clearCancel();
}

}  // namespace

int main() {
  testDefaultsAndRoundTrip();
  testUnitAccessors();
  testPoseReadsCannotTear();
  testCancelFlag();
  return mclib::test::summary("robot_state");
}
