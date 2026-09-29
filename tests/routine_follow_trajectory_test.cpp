// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Routine::followTrajectory() hands the trajectory, the RAMSETE config and the
// step's options to ChassisController::makeFollowTrajectoryCommand(), and
// keeps its own copy of the trajectory.
//
// As in routine_trigger_test.cpp, the routine's source is pulled in here and
// the ChassisController methods it calls are stubbed at the bottom, because
// the real ones need PROS. The stub records what it was given.

// See routine_trigger_test.cpp for why these two lines are here.
#undef _GNU_SOURCE
#define _GNU_SOURCE

#include "mclib/command/commandScheduler.h"
#include "mclib/command/instantCommand.h"
#include "mclib/control/ramsete.hpp"
#include "mclib/path/spline.hpp"
#include "mclib/path/trajectory.hpp"
#include "test_assert.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "../src/mclib/auton/autonomous_routine.cpp"  // NOLINT
#pragma GCC diagnostic pop

#include <cstdint>
#include <cstdio>
#include <memory>
#include <new>

using mclib::auton::Routine;
using namespace mclib::units::literals;

namespace {

struct Recorded {
  int calls = 0;
  std::size_t samples = 0;
  double first_y_in = 0;
  double last_y_in = 0;
  double kv = 0;
  double b = 0;
  double timeout_ms = 0;
  bool exit = true;
  double max_volts = 0;
} recorded;

mclib::path::Trajectory straight(double length_in) {
  mclib::path::TrajectoryConstraints limits;
  limits.max_velocity = 40 * mclib::units::inps;
  limits.max_acceleration = 80 * mclib::control::inps2;
  return mclib::path::Trajectory::generate(
      mclib::path::Path::fromWaypoints(
          {{0_in, 0_in}, {0_in, length_in * mclib::units::inch}}),
      limits);
}

void runToEnd(Command& routine) {
  routine.schedule();
  for (int i = 0; i < 20 && routine.scheduled(); ++i) CommandScheduler::run();
  CHECK(!routine.scheduled());
}

}  // namespace

int main() {
  // A ChassisController needs a Chassis reference. The stubbed constructor
  // below only stores it and nothing here calls through it, so it can refer
  // to storage that never holds a Chassis.
  alignas(mclib::Chassis) static unsigned char storage[sizeof(mclib::Chassis)];
  mclib::ChassisController chassis(*reinterpret_cast<mclib::Chassis*>(storage));

  mclib::control::RamseteConfig config;
  config.feedforward.kV = 0.2_V / mclib::units::inps;
  config.gains.b = 50;

  std::printf("-- options reach the chassis command\n");
  {
    Routine routine(chassis);
    {
      // The trajectory goes out of scope before the routine runs.
      const mclib::path::Trajectory trajectory = straight(36);
      routine.followTrajectory(trajectory, config, 3_s)
          .withMaxVoltage(8_V)
          .withoutExit();
    }
    CHECK(!routine.hasConfigurationError());
    runToEnd(routine);
    // Built once when the step is added and again on each of the two
    // .withXxx() calls; `recorded` holds the last, which is the one that ran.
    CHECK_EQ(recorded.calls, 3);
    CHECK(recorded.samples > 2);
    CHECK_NEAR(recorded.first_y_in, 0.0, 1e-9);
    CHECK_NEAR(recorded.last_y_in, 36.0, 1e-9);
    CHECK_NEAR(recorded.kv, config.feedforward.kV.raw(), 1e-12);
    CHECK_EQ(recorded.b, 50.0);
    CHECK_NEAR(recorded.timeout_ms, 3000.0, 1e-9);
    CHECK(!recorded.exit);
    CHECK_NEAR(recorded.max_volts, 8.0, 1e-12);
  }

  std::printf("-- defaults, and the MotionStep chain form\n");
  {
    recorded = {};
    Routine routine(chassis);
    routine.driveTo(1_in, 1_s).followTrajectory(straight(12), config, 2_s);
    // driveTo's stub returns an instant command, so both steps run.
    runToEnd(routine);
    CHECK_EQ(recorded.calls, 1);
    CHECK_NEAR(recorded.last_y_in, 12.0, 1e-9);
    CHECK(recorded.exit);
    CHECK_NEAR(recorded.max_volts, 12.0, 1e-12);
    CHECK_NEAR(recorded.timeout_ms, 2000.0, 1e-9);
  }

  std::printf("-- no chassis is a configuration error\n");
  {
    Routine routine;
    routine.followTrajectory(straight(12), config, 2_s);
    CHECK(routine.hasConfigurationError());
  }

  return mclib::test::summary("routine_follow_trajectory");
}

// ---------------------------------------------------------------------------
// Link-time stand-ins for the parts of ChassisController and Chassis the
// routine's source references. Only the ones this test calls do anything.
// ---------------------------------------------------------------------------

extern "C" void delay(std::uint32_t /*milliseconds*/) {}

namespace mclib {

namespace {
std::unique_ptr<Command> finished() {
  return std::make_unique<InstantCommand>([] {}, std::initializer_list<Subsystem*>{});
}
}  // namespace

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

std::unique_ptr<Command> ChassisController::makeFollowTrajectoryCommand(
    const path::Trajectory& trajectory, const control::RamseteConfig& config,
    QTime time_limit, bool exit, QVoltage max_output) {
  ++recorded.calls;
  recorded.samples = trajectory.states().size();
  recorded.first_y_in = trajectory.states().front().y.in();
  recorded.last_y_in = trajectory.states().back().y.in();
  recorded.kv = config.feedforward.kV.raw();
  recorded.b = config.gains.b;
  recorded.timeout_ms = time_limit.ms();
  recorded.exit = exit;
  recorded.max_volts = max_output.volts();
  return finished();
}

std::unique_ptr<Command> ChassisController::makeDriveToCommand(QLength, QTime, bool, QVoltage, QVoltage) {
  return finished();
}

std::unique_ptr<Command> ChassisController::makeDriveDistanceCommand(QLength, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToHeadingCommand(QAngle, QTime, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToAngleCommand(QAngle, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeCurveCircleCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage, bool) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeCurveCircleReverseCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeSwingCommand(QAngle, double, QTime, bool, QVoltage, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeWallResetCommand(QLength, QLength, QAngle, QVoltage, QTime, QCurrent, QAngularVelocity) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeTurnToPointCommand(QLength, QLength, int, QTime, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeMoveToPointCommand(QLength, QLength, int, QTime, bool, QVoltage, bool, QVoltage) { return nullptr; }
std::unique_ptr<Command> ChassisController::makeBoomerangCommand(QLength, QLength, int, QAngle, double, QTime, bool, QVoltage, bool, QVoltage) { return nullptr; }

}  // namespace mclib
