// mclib
//
// Object lifetime around CommandScheduler.
//
//   1. A MechanismManager that goes out of scope must not leave the scheduler
//      pointing at the commands it owned. The next run() used to call into
//      freed memory.
//   2. Replacing a subsystem's default command from inside another command's
//      execute() must still run the old default's end(). cancel() plus
//      forgetCommand() inside the run loop dropped the deferred cancel on the
//      floor, so a startEnd() default never fired its on_end.

#include "mclib/command/commandScheduler.h"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/mechanism/mechanism_manager.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <memory>
#include <vector>

using mclib::mechanism::MechanismManager;

namespace {

/// A subsystem that counts its periodic() ticks and nothing else.
class CountingSubsystem : public Subsystem {
public:
  int ticks = 0;

  void periodic() override { ++ticks; }
};

/// A command that runs forever and records which callbacks it saw. Poisons its
/// own counter on destruction, so a use-after-free shows up as a wrong number
/// rather than as luck.
class TracerCommand : public Command {
public:
  explicit TracerCommand(Subsystem* subsystem) : m_subsystem(subsystem) {}

  int initializes = 0;
  int executes = 0;
  int ends = 0;
  bool ended_interrupted = false;

  void initialize() override { ++initializes; }
  void execute() override { ++executes; }
  bool isFinished() override { return false; }

  void end(bool interrupted) override {
    ++ends;
    ended_interrupted = interrupted;
  }

  std::vector<Subsystem*> getRequirements() override {
    return {m_subsystem};
  }

private:
  Subsystem* m_subsystem = nullptr;
};

/// The manager's destructor has to reach the commands it owns, so the tracer
/// has to be observable after the manager dies. The manager owns a thin
/// forwarder; the counters live out here.
class ForwardingCommand : public Command {
public:
  ForwardingCommand(TracerCommand* target, Subsystem* subsystem)
      : m_target(target), m_subsystem(subsystem) {}

  void initialize() override { m_target->initialize(); }
  void execute() override { m_target->execute(); }
  bool isFinished() override { return m_target->isFinished(); }
  void end(bool interrupted) override { m_target->end(interrupted); }

  std::vector<Subsystem*> getRequirements() override {
    return {m_subsystem};
  }

private:
  TracerCommand* m_target = nullptr;
  Subsystem* m_subsystem = nullptr;
};

// ---------------------------------------------------------------------------
// 1. A scoped MechanismManager
// ---------------------------------------------------------------------------
void testScopedManager() {
  std::printf("-- scoped MechanismManager\n");

  CountingSubsystem mechanism;
  mechanism.setName("mechanism");

  TracerCommand tracer(&mechanism);

  {
    MechanismManager manager;
    CHECK(manager.add(&mechanism,
                      std::make_unique<ForwardingCommand>(&tracer, &mechanism),
                      "mechanism"));
    CHECK_EQ(static_cast<double>(manager.registerAll()), 1.0);

    CommandScheduler::run();
    CommandScheduler::run();

    // The default command is scheduled and running.
    CHECK_EQ(static_cast<double>(tracer.initializes), 1.0);
    CHECK(tracer.executes > 0);
    CHECK(CommandScheduler::getRequiring(&mechanism).has_value());
  }

  // The manager is gone and so is the command it owned. Its end() ran, and the
  // scheduler no longer holds it.
  CHECK_EQ(static_cast<double>(tracer.ends), 1.0);
  CHECK(tracer.ended_interrupted);
  CHECK(!CommandScheduler::getRequiring(&mechanism).has_value());

  const int executes_before = tracer.executes;
  const int ticks_before = mechanism.ticks;

  // The run that used to reach freed memory. The subsystem is still registered,
  // so it still ticks, but nothing calls the destroyed command.
  CommandScheduler::run();
  CommandScheduler::run();

  CHECK_EQ(static_cast<double>(tracer.executes),
           static_cast<double>(executes_before));
  CHECK_EQ(static_cast<double>(tracer.initializes), 1.0);
  CHECK_EQ(static_cast<double>(mechanism.ticks),
           static_cast<double>(ticks_before + 2));

  CommandScheduler::unregisterSubsystem(&mechanism);
}

// ---------------------------------------------------------------------------
// 2. Replacing a default command from inside a run loop
// ---------------------------------------------------------------------------

/// Set by the startEnd() default command's on_end lambda.
int g_on_end_calls = 0;

/// Drives one subsystem and, on its second execute(), swaps the default command
/// of another subsystem out from under the scheduler.
class SwapperCommand : public Command {
public:
  SwapperCommand(Subsystem* self, Subsystem* target)
      : m_self(self), m_target(target) {}

  void execute() override {
    ++m_executes;
    if (m_executes == 2 && !m_swapped) {
      m_swapped = true;
      m_target->setDefaultCommand(m_target->runOnce([]() {}));
    }
  }

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override { return {m_self}; }

private:
  Subsystem* m_self = nullptr;
  Subsystem* m_target = nullptr;
  int m_executes = 0;
  bool m_swapped = false;
};

void testDefaultCommandSwapInsideRunLoop() {
  std::printf("-- default command replaced from inside execute()\n");

  g_on_end_calls = 0;

  CountingSubsystem target;
  CountingSubsystem driver;
  target.setName("target");
  driver.setName("driver");

  // startEnd() never finishes, so on_end can only come from an interruption.
  target.setDefaultCommand(
      target.startEnd([]() {}, []() { ++g_on_end_calls; }));
  target.registerSelf();
  driver.registerSelf();

  auto swapper = std::make_unique<SwapperCommand>(&driver, &target);
  CommandScheduler::schedule(swapper.get());

  CommandScheduler::run();  // default command scheduled, swapper executes once
  CHECK_EQ(static_cast<double>(g_on_end_calls), 0.0);

  CommandScheduler::run();  // swapper's second execute() swaps the default

  // The old default was destroyed. Its on_end had to run before that.
  CHECK_EQ(static_cast<double>(g_on_end_calls), 1.0);

  // And the scheduler survived: another pass runs cleanly.
  CommandScheduler::run();
  CHECK_EQ(static_cast<double>(g_on_end_calls), 1.0);

  CommandScheduler::cancel(swapper.get());
  CommandScheduler::forgetCommand(swapper.get());
  CommandScheduler::unregisterSubsystem(&target);
  CommandScheduler::unregisterSubsystem(&driver);
}

// ---------------------------------------------------------------------------
// 3. Unregistering a subsystem from inside a run loop
// ---------------------------------------------------------------------------

/// Unregisters another subsystem on its second execute(), i.e. from inside the
/// scheduler's run loop.
class UnregisterCommand : public Command {
public:
  UnregisterCommand(Subsystem* self, Subsystem* target)
      : m_self(self), m_target(target) {}

  void execute() override {
    ++m_executes;
    if (m_executes == 2 && !m_done) {
      m_done = true;
      CommandScheduler::unregisterSubsystem(m_target);
    }
  }

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override { return {m_self}; }

private:
  Subsystem* m_self = nullptr;
  Subsystem* m_target = nullptr;
  int m_executes = 0;
  bool m_done = false;
};

void testUnregisterInsideRunLoop() {
  std::printf("-- unregisterSubsystem from inside execute()\n");

  CountingSubsystem target;
  CountingSubsystem driver;
  target.setName("target");
  driver.setName("driver");

  TracerCommand tracer(&target);
  target.setDefaultCommand(
      std::make_unique<ForwardingCommand>(&tracer, &target));
  target.registerSelf();
  driver.registerSelf();

  auto unregisterer = std::make_unique<UnregisterCommand>(&driver, &target);
  CommandScheduler::schedule(unregisterer.get());

  CommandScheduler::run();  // target's default command starts
  CHECK_EQ(static_cast<double>(tracer.initializes), 1.0);
  CHECK_EQ(static_cast<double>(tracer.ends), 0.0);

  CommandScheduler::run();  // the unregister lands inside the run loop

  // unregisterSubsystem documents that it cancels what was running. It has to
  // do that here too, not only outside the run loop.
  CHECK_EQ(static_cast<double>(tracer.ends), 1.0);
  CHECK(tracer.ended_interrupted);
  CHECK(!CommandScheduler::getRequiring(&target).has_value());

  const int ticks_before = target.ticks;
  CommandScheduler::run();
  CHECK_EQ(static_cast<double>(target.ticks),
           static_cast<double>(ticks_before));

  CommandScheduler::endAndForget(unregisterer.get());
  CommandScheduler::unregisterSubsystem(&driver);
  CommandScheduler::unregisterSubsystem(&target);
}

}  // namespace

int main() {
  testScopedManager();
  testDefaultCommandSwapInsideRunLoop();
  testUnregisterInsideRunLoop();
  return mclib::test::summary("scheduler_lifetime");
}
