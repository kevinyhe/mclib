// mclib
//
// A trigger() step must not cancel the routine that runs it.
//
// A Routine reserves every subsystem its steps need, and every motion step
// requires the ChassisController. trigger() schedules its inner command through
// the CommandScheduler, so a triggered command that also wants the chassis -
// `.trigger(chassis.makeCorrectHeadingCommand())`, the pattern the header
// advertises - used to be scheduled straight over the top of the routine: the
// scheduler found the routine holding that requirement and, with the default
// CancelRunning, ended it and erased it. Every remaining step was skipped and
// runBlocking() returned as if autonomous had finished.
//
// The routine's own source is pulled in here rather than added to
// HOST_TEST_SRC, because it references ChassisController, whose definitions
// need PROS. The handful of chassis symbols the linker then wants are stubbed
// at the bottom of this file; no test builds a ChassisController.

// autonomous_routine.hpp reaches api.h, which is the one PROS header the host
// build otherwise never sees. Two of its warnings are unavoidable from here and
// have nothing to do with this test: pros/screen.h redefines the _GNU_SOURCE
// that g++ already predefines, and pros/llemu.hpp declares functions inside a
// deprecated namespace. Undefining the macro first and muting the deprecation
// keeps `make test` free of noise nobody in this repo can act on. g++
// predefines _GNU_SOURCE as 1 and screen.h defines it empty; redefining it
// empty here first makes screen.h's definition identical, which is silent.
// glibc only tests whether the macro is defined, so the value does not matter.
#undef _GNU_SOURCE
#define _GNU_SOURCE

#include "mclib/command/commandScheduler.h"
#include "mclib/command/subsystem.h"
#include "test_assert.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "../src/mclib/auton/autonomous_routine.cpp"  // NOLINT
#pragma GCC diagnostic pop

#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

using mclib::auton::Routine;

namespace {

/// Stands in for the ChassisController: something several steps require.
class SharedSubsystem : public Subsystem {};

/// Runs for a fixed number of execute() calls, then finishes.
class CountedCommand : public Command {
public:
  CountedCommand(Subsystem* requirement, int executes_to_finish, int* ran_flag)
      : m_requirement(requirement),
        m_executes_to_finish(executes_to_finish),
        m_ran_flag(ran_flag) {}

  void initialize() override {
    m_executes = 0;
    if (m_ran_flag != nullptr) {
      ++*m_ran_flag;
    }
  }

  void execute() override { ++m_executes; }

  bool isFinished() override { return m_executes >= m_executes_to_finish; }

  std::vector<Subsystem*> getRequirements() override {
    if (m_requirement == nullptr) {
      return {};
    }
    return {m_requirement};
  }

private:
  Subsystem* m_requirement = nullptr;
  int m_executes_to_finish = 0;
  int m_executes = 0;
  int* m_ran_flag = nullptr;
};

/// Never finishes on its own. This is the shape of makeCorrectHeadingCommand():
/// it requires the chassis and runs until something cancels it.
class ForeverCommand : public Command {
public:
  ForeverCommand(Subsystem* requirement, int* ran_flag, int* end_flag)
      : m_requirement(requirement), m_ran_flag(ran_flag), m_end_flag(end_flag) {}

  int executes = 0;
  /// Anything counted here means the command kept running after it was ended.
  int executes_after_end = 0;

  /**
   * @brief Count how many of these are live at once on the same subsystem.
   *
   * Shared counters, so a test can assert that starting one command really
   * stopped another rather than just that both were stopped by the end.
   */
  void countConcurrencyIn(int* live, int* peak_live) {
    m_live = live;
    m_peak_live = peak_live;
  }

  void initialize() override {
    if (m_ran_flag != nullptr) {
      ++*m_ran_flag;
    }
    if (m_live != nullptr) {
      ++*m_live;
      if (m_peak_live != nullptr && *m_live > *m_peak_live) {
        *m_peak_live = *m_live;
      }
    }
  }

  void execute() override {
    ++executes;
    if (m_ended) {
      ++executes_after_end;
    }
  }

  bool isFinished() override { return false; }

  void end(bool /*interrupted*/) override {
    if (!m_ended && m_live != nullptr) {
      --*m_live;
    }
    m_ended = true;
    if (m_end_flag != nullptr) {
      ++*m_end_flag;
    }
  }

  std::vector<Subsystem*> getRequirements() override {
    return {m_requirement};
  }

private:
  Subsystem* m_requirement = nullptr;
  int* m_ran_flag = nullptr;
  int* m_end_flag = nullptr;
  bool m_ended = false;
  int* m_live = nullptr;
  int* m_peak_live = nullptr;
};

/// Run the scheduler until the routine finishes, with a hard tick cap so a
/// broken routine fails the assertions instead of hanging the suite.
int runToCompletion(Routine& routine, int max_ticks) {
  CommandScheduler::schedule(&routine);

  int ticks = 0;
  while (routine.scheduled() && ticks < max_ticks) {
    CommandScheduler::run();
    ++ticks;
  }

  return ticks;
}

// ---------------------------------------------------------------------------
void testTriggerSharingTheReservation() {
  std::printf("-- trigger() sharing the routine's reservation\n");

  SharedSubsystem chassis;
  chassis.setName("chassis");

  int first_ran = 0;
  int second_ran = 0;
  int third_ran = 0;
  int triggered_ran = 0;
  int triggered_ended = 0;

  Routine routine;
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, &first_ran));
  routine.trigger(
      std::make_unique<ForeverCommand>(&chassis, &triggered_ran, &triggered_ended));
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, &second_ran));
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, &third_ran));

  // Four steps in, four steps expected to run.
  CHECK_EQ(static_cast<double>(routine.size()), 4.0);

  // The routine reserves the shared subsystem, exactly as it does the chassis.
  CHECK_EQ(static_cast<double>(routine.getRequirements().size()), 1.0);

  const int ticks = runToCompletion(routine, 200);

  CHECK(!routine.scheduled());
  CHECK(ticks < 200);

  // The triggered command started...
  CHECK_EQ(static_cast<double>(triggered_ran), 1.0);
  // ...and the two steps behind it still ran. This is the regression: they used
  // to be silently skipped because the routine had been cancelled.
  CHECK_EQ(static_cast<double>(first_ran), 1.0);
  CHECK_EQ(static_cast<double>(second_ran), 1.0);
  CHECK_EQ(static_cast<double>(third_ran), 1.0);

  // And the routine stopped the fire-and-forget command on its way out.
  CHECK_EQ(static_cast<double>(triggered_ended), 1.0);
  CHECK(!CommandScheduler::getRequiring(&chassis).has_value());

  CommandScheduler::forgetCommand(&routine);
}

// ---------------------------------------------------------------------------
void testTriggerOnAnUnreservedSubsystem() {
  std::printf("-- trigger() on a subsystem the routine does not hold\n");

  SharedSubsystem chassis;
  SharedSubsystem intake;
  chassis.setName("chassis");
  intake.setName("intake");

  int step_ran = 0;
  int triggered_ran = 0;
  int triggered_ended = 0;

  Routine routine;
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, &step_ran));
  routine.trigger(
      std::make_unique<ForeverCommand>(&intake, &triggered_ran, &triggered_ended));

  runToCompletion(routine, 200);

  CHECK_EQ(static_cast<double>(step_ran), 1.0);
  CHECK_EQ(static_cast<double>(triggered_ran), 1.0);
  CHECK_EQ(static_cast<double>(triggered_ended), 1.0);

  // The intake is not part of the routine's reservation, so the triggered
  // command claims it the normal way and shows up in the requirement map while
  // it runs. By the time the routine ends it has been cancelled and released.
  CHECK(!CommandScheduler::getRequiring(&intake).has_value());

  CommandScheduler::forgetCommand(&routine);
}

// ---------------------------------------------------------------------------
void testTwoTriggersOnTheSameSubsystem() {
  std::printf("-- two trigger()s that both want the reserved subsystem\n");

  SharedSubsystem chassis;
  chassis.setName("chassis");

  int step_ran = 0;
  int first_ran = 0;
  int first_ended = 0;
  int second_ran = 0;
  int second_ended = 0;

  int live = 0;
  int peak_live = 0;

  auto first = std::make_unique<ForeverCommand>(&chassis, &first_ran, &first_ended);
  auto second = std::make_unique<ForeverCommand>(&chassis, &second_ran, &second_ended);
  ForeverCommand* first_raw = first.get();
  ForeverCommand* second_raw = second.get();
  first_raw->countConcurrencyIn(&live, &peak_live);
  second_raw->countConcurrencyIn(&live, &peak_live);

  Routine routine;
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, &step_ran));
  routine.trigger(std::move(first));
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, nullptr));
  routine.trigger(std::move(second));
  routine.add(std::make_unique<CountedCommand>(&chassis, 2, nullptr));

  runToCompletion(routine, 200);

  CHECK_EQ(static_cast<double>(step_ran), 1.0);
  CHECK_EQ(static_cast<double>(first_ran), 1.0);
  CHECK_EQ(static_cast<double>(second_ran), 1.0);

  // The scheduler cannot arbitrate the chassis here: it never saw either inner
  // command claim it. The routine does it instead, on the scheduler's own
  // CancelRunning rule, so starting the second trigger stopped the first. Never
  // two commands writing to the same chassis at once.
  CHECK_EQ(static_cast<double>(peak_live), 1.0);
  CHECK_EQ(static_cast<double>(live), 0.0);

  CHECK_EQ(static_cast<double>(first_ended), 1.0);
  CHECK_EQ(static_cast<double>(first_raw->executes_after_end), 0.0);

  // And the second one really did run, and outlived the first.
  CHECK(second_raw->executes > 0);
  CHECK_EQ(static_cast<double>(second_ended), 1.0);

  CommandScheduler::forgetCommand(&routine);
}

}  // namespace

// ---------------------------------------------------------------------------
// Link-time stand-ins.
//
// autonomous_routine.cpp names these, so the linker wants them even though no
// test here builds a ChassisController. Their real definitions live in
// chassis_controller.cpp, which needs PROS. Every routine in this file is
// built with no chassis, so none of them is ever called; if one ever is, the
// null return fails loudly rather than quietly.
// ---------------------------------------------------------------------------

extern "C" void delay(std::uint32_t /*milliseconds*/) {}

namespace mclib {

void ChassisController::driveDistance(QLength, QTime, bool, QVoltage) {}

void ChassisController::cancel() {}

std::unique_ptr<Command> ChassisController::makeDriveDistanceCommand(QLength, QTime, bool, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeTurnToHeadingCommand(QAngle, QTime, bool, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeTurnToAngleCommand(QAngle, QTime, bool, QVoltage, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeDriveToCommand(QLength, QTime, bool, QVoltage, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeCurveCircleCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage, bool) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeCurveCircleReverseCommand(QAngle, QLength, QTime, bool, QVoltage, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeSwingCommand(QAngle, double, QTime, bool, QVoltage, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeWallResetCommand(QLength, QLength, QAngle, QVoltage, QTime, QCurrent, QAngularVelocity) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeTurnToPointCommand(QLength, QLength, int, QTime, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeMoveToPointCommand(QLength, QLength, int, QTime, bool, QVoltage, bool, QVoltage) {
  return nullptr;
}

std::unique_ptr<Command> ChassisController::makeBoomerangCommand(QLength, QLength, int, QAngle, double, QTime, bool, QVoltage, bool, QVoltage) {
  return nullptr;
}

}  // namespace mclib

int main() {
  testTriggerSharingTheReservation();
  testTriggerOnAnUnreservedSubsystem();
  testTwoTriggersOnTheSameSubsystem();

  return mclib::test::summary("routine_trigger");
}
