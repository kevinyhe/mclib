// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Routine::MotionStep::atTime(), atDistance(), atDistanceFromEnd() and
// withEarlyExit().
//
// The routine's own source is pulled in, and the chassis stubbed, the same way
// as routine_follow_trajectory_test.cpp; see that file for why. The scripted
// motion moves the pose 2 in forward and the clock 10 ms on every tick, and
// publishes the distance left to y = 24 in, as a real motion would.
#undef _GNU_SOURCE
#define _GNU_SOURCE

#include "mclib/command/commandScheduler.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/runCommand.h"
#include "mclib/control/robot_state.hpp"
#include "mclib/time.hpp"
#include "test_assert.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "../src/mclib/auton/autonomous_routine.cpp"  // NOLINT
#pragma GCC diagnostic pop

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

using mclib::auton::Point;
using mclib::auton::Routine;
using mclib::control::robotState;
using namespace mclib::units::literals;

namespace {

std::uint32_t now_ms = 0;
std::uint32_t fakeClock() { return now_ms; }

struct MotionArgs {
  bool exit = true;
  double early_exit_in = -1;
  double early_exit_deg = -1;
} args;

int ticks_to_finish = 0;
int motion_tick = 0;
std::vector<int> fired_at_tick;

class ScriptedMotion : public Command {
public:
  void initialize() override { motion_tick = 0; }
  void execute() override {
    ++motion_tick;
    now_ms += 10;
    const mclib::Pose2D pose = robotState().pose();
    robotState().setPose({pose.x, pose.y + 2, pose.theta});
    mclib::control::MotionTelemetry telemetry{};
    telemetry.remaining_in = 24 - (pose.y + 2);
    robotState().setMotionTelemetry(telemetry);
  }
  bool isFinished() override { return motion_tick >= ticks_to_finish; }
  void end(bool) override { robotState().setMotionTelemetry({}); }
};

std::unique_ptr<Command> recordTick() {
  return std::make_unique<InstantCommand>([] { fired_at_tick.push_back(motion_tick); },
                                          std::initializer_list<Subsystem*>{});
}

void runToEnd(Command& routine) {
  routine.schedule();
  for (int i = 0; i < 60 && routine.scheduled(); ++i) CommandScheduler::run();
  CHECK(!routine.scheduled());
}

void reset(int finish_after) {
  now_ms = 1000;
  args = {};
  ticks_to_finish = finish_after;
  motion_tick = 0;
  fired_at_tick.clear();
  robotState().setPose({0, 0, 0});
  robotState().setMotionTelemetry({});
}

}  // namespace

int main() {
  mclib::time::setClock(fakeClock);
  alignas(mclib::Chassis) static unsigned char storage[sizeof(mclib::Chassis)];
  mclib::ChassisController chassis(*reinterpret_cast<mclib::Chassis*>(storage));

  std::printf("-- each event starts its command on the tick it comes due\n");
  {
    reset(12);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .atTime(45_ms, recordTick())
        .atDistance(9_in, recordTick())
        .atDistanceFromEnd(7_in, recordTick());
    runToEnd(routine);
    // 10 ms and 2 in per tick: 45 ms passes at tick 5, 9 in at tick 5, and
    // 7 in left (y = 17 in) at tick 9. Each threshold falls between ticks:
    // 6_in reads back as 5.9999999 in.
    CHECK_EQ(fired_at_tick.size(), 3u);
    if (fired_at_tick.size() == 3) {
      CHECK_EQ(fired_at_tick[0], 5);
      CHECK_EQ(fired_at_tick[1], 5);
      CHECK_EQ(fired_at_tick[2], 9);
    }
    CHECK_EQ(routine.missedEvents(), 0u);
  }

  std::printf("-- the distance travelled counts along a curve\n");
  {
    reset(12);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s).atDistance(9_in, recordTick());
    // Each tick also moves 2 in sideways on top of the scripted 2 in forward:
    // 2.83 in of path a tick, so 9 in comes at tick 4, not tick 5.
    routine.schedule();
    for (int i = 0; i < 60 && routine.scheduled(); ++i) {
      const mclib::Pose2D pose = robotState().pose();
      robotState().setPose({pose.x + 2, pose.y, pose.theta});
      CommandScheduler::run();
    }
    CHECK_EQ(fired_at_tick.size(), 1u);
    if (!fired_at_tick.empty()) CHECK_EQ(fired_at_tick[0], 4);
  }

  std::printf("-- a step that ends first misses its event\n");
  {
    reset(5);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .atDistance(30_in, recordTick())
        .atTime(15_ms, recordTick());
    runToEnd(routine);
    CHECK_EQ(fired_at_tick.size(), 1u);
    CHECK_EQ(routine.missedEvents(), 1u);
  }

  std::printf("-- a step cancelled or cut by the time budget misses its events\n");
  {
    reset(20);
    Routine cancelled(chassis);
    cancelled.moveToPoint(Point{0_in, 24_in}, 1, 2_s).atTime(150_ms, recordTick());
    cancelled.schedule();
    for (int i = 0; i < 3; ++i) CommandScheduler::run();
    cancelled.cancel();
    CHECK(!cancelled.scheduled());
    CHECK(fired_at_tick.empty());
    CHECK_EQ(cancelled.missedEvents(), 1u);

    reset(20);
    Routine budgeted(chassis);
    budgeted.withTimeBudget(50_ms)
        .moveToPoint(Point{0_in, 24_in}, 1, 2_s)
        .atTime(150_ms, recordTick());
    runToEnd(budgeted);
    CHECK(budgeted.budgetExpired());
    CHECK(fired_at_tick.empty());
    CHECK_EQ(budgeted.missedEvents(), 1u);
  }

  std::printf("-- running the routine again fires the events again\n");
  {
    reset(6);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s).atTime(25_ms, recordTick());
    runToEnd(routine);
    robotState().setPose({0, 0, 0});
    runToEnd(routine);
    CHECK_EQ(fired_at_tick.size(), 2u);
    CHECK_EQ(routine.missedEvents(), 0u);
  }

  std::printf("-- an event's command stops when the routine ends\n");
  {
    reset(4);
    int runs = 0;
    auto forever = std::make_unique<RunCommand>([&runs] { ++runs; },
                                                std::initializer_list<Subsystem*>{});
    Command* forever_raw = forever.get();
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s).atTime(0_ms, std::move(forever));
    runToEnd(routine);
    CHECK(runs > 0);
    CommandScheduler::run();
    CHECK(!forever_raw->scheduled());
  }

  std::printf("-- withEarlyExit() chains the step and passes the range\n");
  {
    reset(2);
    Routine routine(chassis);
    routine.moveToPoint(Point{0_in, 24_in}, 1, 2_s).withEarlyExit(6_in);
    runToEnd(routine);
    CHECK(!args.exit);
    CHECK_NEAR(args.early_exit_in, 6, 1e-9);

    reset(2);
    Routine turn(chassis);
    turn.turnToAngle(90_deg, 1_s).withEarlyExit(10_deg);
    runToEnd(turn);
    CHECK(!args.exit);
    CHECK_NEAR(args.early_exit_deg, 10, 1e-9);

    reset(2);
    Routine stopped(chassis);
    stopped.moveToPoint(Point{0_in, 24_in}, 1, 2_s).withEarlyExit(6_in).withExit();
    runToEnd(stopped);
    CHECK(args.exit);
    CHECK_NEAR(args.early_exit_in, 0, 1e-9);
    CHECK(!stopped.hasConfigurationError());
  }

  std::printf("-- configuration errors: nothing runs\n");
  {
    auto expectError = [](Routine& routine) {
      CHECK(routine.hasConfigurationError());
      reset(3);
      runToEnd(routine);
      CHECK_EQ(motion_tick, 0);
    };
    Routine turn_with_length(chassis);
    turn_with_length.turnToAngle(90_deg, 1_s).withEarlyExit(6_in);
    expectError(turn_with_length);

    Routine point_with_angle(chassis);
    point_with_angle.moveToPoint(Point{0_in, 24_in}, 1, 2_s).withEarlyExit(10_deg);
    expectError(point_with_angle);

    Routine negative_range(chassis);
    negative_range.moveToPoint(Point{0_in, 24_in}, 1, 2_s).withEarlyExit(-1_in);
    expectError(negative_range);

    Routine from_end_of_turn(chassis);
    from_end_of_turn.turnToPoint(Point{24_in, 24_in}, 1_s).atDistanceFromEnd(6_in, recordTick());
    expectError(from_end_of_turn);

    Routine no_command(chassis);
    no_command.moveToPoint(Point{0_in, 24_in}, 1, 2_s).atTime(10_ms, nullptr);
    expectError(no_command);

    Routine negative_time(chassis);
    negative_time.moveToPoint(Point{0_in, 24_in}, 1, 2_s).atTime(-10_ms, recordTick());
    expectError(negative_time);
  }

  mclib::time::setClock(nullptr);
  return mclib::test::summary("routine_events");
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
    QLength, QLength, int, QTime, bool exit, QVoltage, bool, QVoltage, QLength early_exit_range) {
  args.exit = exit;
  args.early_exit_in = early_exit_range.in();
  return std::make_unique<ScriptedMotion>();
}

std::unique_ptr<Command> ChassisController::makeTurnToAngleCommand(
    QAngle, QTime, bool exit, QVoltage, QVoltage, QAngle early_exit_range) {
  args.exit = exit;
  args.early_exit_deg = early_exit_range.deg();
  return std::make_unique<ScriptedMotion>();
}

std::unique_ptr<Command> ChassisController::makeTurnToPointCommand(QLength, QLength, int, QTime, QVoltage) {
  return std::make_unique<ScriptedMotion>();
}

std::unique_ptr<Command> ChassisController::makeFollowTrajectoryCommand(const path::Trajectory&, const control::RamseteConfig&, QTime, bool, QVoltage, QLength) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeDriveToCommand(QLength, QTime, bool, QVoltage, QVoltage, QLength) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeDriveDistanceCommand(QLength, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToHeadingCommand(QAngle, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeCurveCircleCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage, bool, QLength) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeCurveCircleReverseCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage, QLength) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeSwingCommand(QAngle, double, QTime, bool, QVoltage, QVoltage, QAngle) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeWallResetCommand(QLength, QLength, QAngle, QVoltage, QTime, QCurrent, QAngularVelocity, std::optional<control::WallDistanceCheck>) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeBoomerangCommand(QLength, QLength, int, QAngle, double, QTime, bool, QVoltage, bool, QVoltage, QLength) { return nullptr; }

}  // namespace mclib
