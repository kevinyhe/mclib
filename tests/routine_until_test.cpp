// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Routine::MotionStep::until() ends a motion step early once its condition
// comes true, and owns the motion it races.
//
// The routine's own source is pulled in, and the chassis stubbed, the same way
// as routine_follow_trajectory_test.cpp; see that file for why.
#undef _GNU_SOURCE
#define _GNU_SOURCE

#include "mclib/command/commandScheduler.h"
#include "mclib/command/instantCommand.h"
#include "test_assert.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "../src/mclib/auton/autonomous_routine.cpp"  // NOLINT
#pragma GCC diagnostic pop

#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

using mclib::auton::Point;
using mclib::auton::Routine;
using namespace mclib::units::literals;

namespace {

struct MotionLog {
  int initialized = 0;
  int executes = 0;
  int ends = 0;
  bool interrupted = false;
  double max_volts = 0;
} motion;

int executes_to_finish = 0;
int after_steps = 0;

class ScriptedMotion : public Command {
public:
  void initialize() override {
    ++motion.initialized;
    m_executes = 0;
  }
  void execute() override {
    ++motion.executes;
    ++m_executes;
  }
  bool isFinished() override { return m_executes >= executes_to_finish; }
  void end(bool interrupted) override {
    ++motion.ends;
    motion.interrupted = interrupted;
  }

private:
  int m_executes = 0;
};

template <typename T, typename = void>
struct HasUntil : std::false_type {};

template <typename T>
struct HasUntil<T, std::void_t<decltype(std::declval<T&>().until(std::function<bool()>{}))>>
    : std::true_type {};

static_assert(HasUntil<Routine::MotionStep>::value);
static_assert(!HasUntil<Routine>::value,
              "until() on a Routine would race the whole routine and discard the result");

void runToEnd(Command& routine) {
  routine.schedule();
  for (int i = 0; i < 40 && routine.scheduled(); ++i) CommandScheduler::run();
  CHECK(!routine.scheduled());
}

void reset(int finish_after) {
  motion = {};
  executes_to_finish = finish_after;
  after_steps = 0;
}

}  // namespace

int main() {
  alignas(mclib::Chassis) static unsigned char storage[sizeof(mclib::Chassis)];
  mclib::ChassisController chassis(*reinterpret_cast<mclib::Chassis*>(storage));

  std::printf("-- condition comes true first: motion interrupted, next step runs\n");
  {
    reset(20);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .until([] { return motion.executes >= 3; })
        .runOnce([] { ++after_steps; });
    runToEnd(routine);
    CHECK_EQ(motion.initialized, 1);
    CHECK_EQ(motion.executes, 3);
    CHECK_EQ(motion.ends, 1);
    CHECK(motion.interrupted);
    CHECK_EQ(after_steps, 1);
  }

  std::printf("-- motion finishes first: ends normally\n");
  {
    reset(4);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .until([] { return false; })
        .runOnce([] { ++after_steps; });
    runToEnd(routine);
    CHECK_EQ(motion.executes, 4);
    CHECK_EQ(motion.ends, 1);
    CHECK(!motion.interrupted);
    CHECK_EQ(after_steps, 1);
  }

  std::printf("-- motion and condition finish on the same tick: not interrupted\n");
  {
    reset(3);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s).until([] {
      return motion.executes >= 3;
    });
    runToEnd(routine);
    CHECK_EQ(motion.ends, 1);
    CHECK(!motion.interrupted);
  }

  std::printf("-- a later withXxx() keeps the condition\n");
  {
    reset(20);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .until([] { return motion.executes >= 2; })
        .withMaxVoltage(6_V);
    runToEnd(routine);
    CHECK_NEAR(motion.max_volts, 6.0, 1e-12);
    CHECK_EQ(motion.executes, 2);
    CHECK(motion.interrupted);
  }

  std::printf("-- condition already true at the start: the motion never runs\n");
  {
    reset(20);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .until([] { return true; })
        .runOnce([] { ++after_steps; });
    runToEnd(routine);
    CHECK_EQ(motion.initialized, 0);
    CHECK_EQ(motion.executes, 0);
    CHECK_EQ(motion.ends, 0);
    CHECK_EQ(after_steps, 1);
  }

  std::printf("-- each run starts from a fresh copy of the condition\n");
  {
    reset(20);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .until([checks = 0]() mutable { return ++checks >= 3; });
    runToEnd(routine);
    const int first_run_executes = motion.executes;
    motion.executes = 0;
    runToEnd(routine);
    CHECK_EQ(first_run_executes, 2);
    CHECK_EQ(motion.executes, first_run_executes);
    CHECK_EQ(motion.initialized, 2);
  }

  std::printf("-- an empty condition is a configuration error\n");
  {
    reset(4);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s).until(std::function<bool()>{});
    CHECK(routine.hasConfigurationError());
    runToEnd(routine);
    CHECK_EQ(motion.executes, 4);
    CHECK(!motion.interrupted);
  }

  std::printf("-- without until() the motion runs to the end\n");
  {
    reset(5);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s);
    runToEnd(routine);
    CHECK_EQ(motion.executes, 5);
    CHECK(!motion.interrupted);
  }

  return mclib::test::summary("routine_until");
}

extern "C" void delay(std::uint32_t /*milliseconds*/) {}

namespace mclib {

ChassisController::ChassisController(Chassis& chassis, ChassisControllerConfig config)
    : m_chassis(chassis),
      m_config(config),
      m_distance_pid(0, 0, 0),
      m_turn_pid(0, 0, 0),
      m_heading_pid(0, 0, 0) {}

void ChassisController::periodic() {}
void Chassis::stop(device::BrakeMode) {}
void ChassisController::driveDistance(QLength, QTime, bool, QVoltage) {}
void ChassisController::cancel() {}

std::unique_ptr<Command> ChassisController::makeMoveToPointCommand(
    QLength, QLength, int, QTime, bool, QVoltage max_output, bool, QVoltage) {
  motion.max_volts = max_output.volts();
  return std::make_unique<ScriptedMotion>();
}

std::unique_ptr<Command> ChassisController::makeFollowTrajectoryCommand(const path::Trajectory&, const control::RamseteConfig&, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeDriveToCommand(QLength, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeDriveDistanceCommand(QLength, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToHeadingCommand(QAngle, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToAngleCommand(QAngle, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeCurveCircleCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage, bool) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeCurveCircleReverseCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeSwingCommand(QAngle, double, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeWallResetCommand(QLength, QLength, QAngle, QVoltage, QTime, QCurrent, QAngularVelocity) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToPointCommand(QLength, QLength, int, QTime, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeBoomerangCommand(QLength, QLength, int, QAngle, double, QTime, bool, QVoltage, bool, QVoltage) { return nullptr; }

}  // namespace mclib
