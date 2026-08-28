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
#include <optional>
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

// ---------------------------------------------------------------------------
// 4. A command forgotten while the deferred queues are draining
// ---------------------------------------------------------------------------

/// Cancels `victim` from inside the run loop, so the cancel is deferred.
class DeferredCancelCommand : public Command {
public:
  DeferredCancelCommand(Subsystem* self, Command* victim)
      : m_self(self), m_victim(victim) {}

  void execute() override {
    if (!m_done) {
      m_done = true;
      CommandScheduler::cancel(m_victim);
    }
  }

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override { return {m_self}; }

private:
  Subsystem* m_self = nullptr;
  Command* m_victim = nullptr;
  bool m_done = false;
};

/// Schedules `passenger` from execute() (so it lands in the deferred schedule
/// queue) and forgets it again from end(), the way Routine::end() drops the
/// fire-and-forget commands trigger() left running.
class PassengerOwnerCommand : public Command {
public:
  PassengerOwnerCommand(Subsystem* self, Command* passenger)
      : m_self(self), m_passenger(passenger) {}

  void execute() override { m_passenger->schedule(); }

  bool isFinished() override { return false; }

  void end(bool /*interrupted*/) override {
    CommandScheduler::endAndForget(m_passenger);
  }

  std::vector<Subsystem*> getRequirements() override { return {m_self}; }

private:
  Subsystem* m_self = nullptr;
  Command* m_passenger = nullptr;
};

void testForgetDuringDeferredDrain() {
  std::printf("-- a command forgotten while the deferred queues drain\n");

  CountingSubsystem owner_subsystem;
  CountingSubsystem driver;
  CountingSubsystem passenger_subsystem;
  owner_subsystem.setName("owner");
  driver.setName("driver");
  passenger_subsystem.setName("passenger");

  TracerCommand passenger(&passenger_subsystem);

  PassengerOwnerCommand owner(&owner_subsystem, &passenger);
  DeferredCancelCommand canceller(&driver, &owner);

  owner_subsystem.registerSelf();
  driver.registerSelf();
  passenger_subsystem.registerSelf();

  CommandScheduler::schedule(&owner);
  CommandScheduler::schedule(&canceller);

  // In this one pass: owner->execute() queues the passenger onto toSchedule and
  // canceller->execute() queues the owner onto toCancel. Draining toCancel runs
  // owner->end(), which forgets the passenger. The passenger must not then be
  // scheduled anyway out of a stale copy of toSchedule.
  CommandScheduler::run();

  CHECK_EQ(static_cast<double>(passenger.initializes), 0.0);
  CHECK(!CommandScheduler::scheduled(&passenger));

  const int executes_before = passenger.executes;
  CommandScheduler::run();
  CHECK_EQ(static_cast<double>(passenger.executes),
           static_cast<double>(executes_before));

  CommandScheduler::endAndForget(&canceller);
  CommandScheduler::endAndForget(&owner);
  CommandScheduler::unregisterSubsystem(&owner_subsystem);
  CommandScheduler::unregisterSubsystem(&driver);
  CommandScheduler::unregisterSubsystem(&passenger_subsystem);
}

// ---------------------------------------------------------------------------
// 5. A default command that replaces itself from its own execute()
// ---------------------------------------------------------------------------

/// Set by the replaced default command's on_end lambda.
int g_self_replace_ends = 0;

void testDefaultCommandReplacesItself() {
  std::printf("-- a default command that replaces itself from execute()\n");

  g_self_replace_ends = 0;

  CountingSubsystem subsystem;
  subsystem.setName("self-replacer");

  bool swapped = false;

  // The command hands its own subsystem a new default command from inside its
  // own execute(), which frees the command mid-run. run() must not go on to ask
  // the freed object isFinished().
  subsystem.setDefaultCommand(std::make_unique<FunctionalCommand>(
      []() {},
      [&subsystem, &swapped]() {
        if (!swapped) {
          swapped = true;
          subsystem.setDefaultCommand(subsystem.startEnd([]() {}, []() {}));
        }
      },
      [](bool) { ++g_self_replace_ends; },
      []() { return false; },
      std::initializer_list<Subsystem*>{&subsystem}));
  subsystem.registerSelf();

  CommandScheduler::run();  // schedules the first default command
  CommandScheduler::run();  // it replaces itself from inside execute()

  CHECK(swapped);
  CHECK_EQ(static_cast<double>(g_self_replace_ends), 1.0);

  // The scheduler survived and carries on with the replacement.
  CommandScheduler::run();
  CHECK_EQ(static_cast<double>(g_self_replace_ends), 1.0);
  CHECK(CommandScheduler::getRequiring(&subsystem).has_value());

  CommandScheduler::unregisterSubsystem(&subsystem);
}

// ---------------------------------------------------------------------------
// 6. A finished command must not have end() run a second time
// ---------------------------------------------------------------------------

/// Finishes on its first execute(). Counts every end() it is given.
class FinishesImmediatelyCommand : public Command {
public:
  explicit FinishesImmediatelyCommand(Subsystem* subsystem)
      : m_subsystem(subsystem) {}

  int ends = 0;

  void execute() override { m_finished = true; }
  bool isFinished() override { return m_finished; }
  void end(bool /*interrupted*/) override { ++ends; }

  std::vector<Subsystem*> getRequirements() override { return {m_subsystem}; }

private:
  Subsystem* m_subsystem = nullptr;
  bool m_finished = false;
};

/// Unregisters `target` from its own execute(), which reaches the target's
/// default command through endAndForget.
class LateUnregisterCommand : public Command {
public:
  LateUnregisterCommand(Subsystem* self, Subsystem* target)
      : m_self(self), m_target(target) {}

  void execute() override {
    if (!m_done) {
      m_done = true;
      CommandScheduler::unregisterSubsystem(m_target);
    }
  }

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override { return {m_self}; }

private:
  Subsystem* m_self = nullptr;
  Subsystem* m_target = nullptr;
  bool m_done = false;
};

void testFinishedCommandEndsOnce() {
  std::printf("-- a finished command is not ended twice in one pass\n");

  CountingSubsystem target;
  CountingSubsystem driver;
  target.setName("target");
  driver.setName("driver");

  // Registered directly, so the command outlives the scheduler's raw pointer.
  FinishesImmediatelyCommand finisher(&target);
  CommandScheduler::registerSubsystem(&target, &finisher);

  driver.registerSelf();

  // Pass 1 only schedules the finisher, via the default-command pass.
  CommandScheduler::run();
  CHECK(CommandScheduler::scheduled(&finisher));
  CHECK_EQ(static_cast<double>(finisher.ends), 0.0);

  // Scheduled after the finisher, so it executes after it in the next pass.
  LateUnregisterCommand late(&driver, &target);
  CommandScheduler::schedule(&late);

  // In pass 2 the finisher finishes and gets end(false); later in that same
  // pass `late` unregisters the target and reaches the finisher again through
  // endAndForget. It must not get a second end().
  CommandScheduler::run();

  CHECK_EQ(static_cast<double>(finisher.ends), 1.0);

  CommandScheduler::endAndForget(&late);
  CommandScheduler::endAndForget(&finisher);
  CommandScheduler::unregisterSubsystem(&driver);
  CommandScheduler::unregisterSubsystem(&target);
}

// ---------------------------------------------------------------------------
// 7. A replacement scheduled from inside end() must not re-end the dying command
// ---------------------------------------------------------------------------

/// Schedules a replacement wanting the same subsystem from inside its own
/// end(). schedule() picks who to interrupt out of the requirements map, so a
/// dying command still listed there was handed end(true) again, which scheduled
/// again, until the stack ran out.
class ReschedulingCommand : public Command {
public:
  ReschedulingCommand(Subsystem* subsystem, Command** replacement)
      : m_subsystem(subsystem), m_replacement(replacement) {}

  int ends = 0;

  bool isFinished() override { return false; }

  void end(bool /*interrupted*/) override {
    ++ends;
    // Guarded, so a scheduler that does recurse blows the check rather than the
    // stack, and the test reports a number instead of a crash.
    if (ends < 20 && m_replacement != nullptr && *m_replacement != nullptr) {
      CommandScheduler::schedule(*m_replacement);
    }
  }

  std::vector<Subsystem*> getRequirements() override { return {m_subsystem}; }

private:
  Subsystem* m_subsystem = nullptr;
  Command** m_replacement = nullptr;
};

void testEndSchedulingAReplacementDoesNotRecurse() {
  std::printf("-- a replacement scheduled from end() does not re-end its host\n");

  CountingSubsystem subsystem;
  subsystem.setName("shared");
  subsystem.registerSelf();

  TracerCommand replacement(&subsystem);
  Command* replacement_ptr = &replacement;

  ReschedulingCommand dying(&subsystem, &replacement_ptr);
  CommandScheduler::schedule(&dying);
  CHECK(CommandScheduler::scheduled(&dying));

  // Cancel it. Its end() schedules the replacement, which wants the same
  // subsystem. The dying command must already be out of the requirements map by
  // then, so schedule() finds nothing to interrupt.
  CommandScheduler::cancel(&dying);

  CHECK_EQ(static_cast<double>(dying.ends), 1.0);
  CHECK(!CommandScheduler::scheduled(&dying));

  // The replacement really did take over.
  CHECK(CommandScheduler::scheduled(&replacement));
  CHECK_EQ(static_cast<double>(replacement.initializes), 1.0);

  std::optional<Command*> owner = CommandScheduler::getRequiring(&subsystem);
  CHECK(owner.has_value());
  CHECK(owner.has_value() && *owner == &replacement);

  CommandScheduler::endAndForget(&replacement);
  CommandScheduler::unregisterSubsystem(&subsystem);
}

void testEndAndForgetSchedulingAReplacementDoesNotRecurse() {
  std::printf("-- the same, through endAndForget\n");

  CountingSubsystem subsystem;
  subsystem.setName("shared");
  subsystem.registerSelf();

  TracerCommand replacement(&subsystem);
  Command* replacement_ptr = &replacement;

  ReschedulingCommand dying(&subsystem, &replacement_ptr);
  CommandScheduler::schedule(&dying);

  CommandScheduler::endAndForget(&dying);

  CHECK_EQ(static_cast<double>(dying.ends), 1.0);
  CHECK(CommandScheduler::scheduled(&replacement));

  CommandScheduler::endAndForget(&replacement);
  CommandScheduler::unregisterSubsystem(&subsystem);
}

// ---------------------------------------------------------------------------
// 8. A default command that replaces itself must survive its own execute()
// ---------------------------------------------------------------------------

/// Replaces itself from execute() and then keeps touching its own members, the
/// shape that read freed memory when setDefaultCommand destroyed it outright.
class SelfReplacingCommand : public Command {
public:
  SelfReplacingCommand(Subsystem* subsystem, int* after_swap)
      : m_subsystem(subsystem), m_after_swap(after_swap) {}

  void execute() override {
    if (m_swapped) {
      return;
    }
    m_swapped = true;

    m_subsystem->setDefaultCommand(m_subsystem->startEnd([]() {}, []() {}));

    // Every one of these is a read of *this* after the swap.
    ++m_touches;
    if (m_after_swap != nullptr) {
      *m_after_swap = m_touches;
    }
  }

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override { return {m_subsystem}; }

private:
  Subsystem* m_subsystem = nullptr;
  int* m_after_swap = nullptr;
  int m_touches = 0;
  bool m_swapped = false;
};

void testSelfReplacingCommandSurvivesItsOwnFrame() {
  std::printf("-- a self-replacing default command survives its own frame\n");

  CountingSubsystem subsystem;
  subsystem.setName("self-replacer");

  int after_swap = 0;

  subsystem.setDefaultCommand(
      std::make_unique<SelfReplacingCommand>(&subsystem, &after_swap));
  subsystem.registerSelf();

  CommandScheduler::run();  // schedules it
  CommandScheduler::run();  // it replaces itself and then reads its own members

  // Under ASan this is where the use-after-free fired. The value proves the
  // read happened and landed in live memory.
  CHECK_EQ(static_cast<double>(after_swap), 1.0);

  // The replacement took over cleanly.
  CommandScheduler::run();
  CHECK(CommandScheduler::getRequiring(&subsystem).has_value());
  CHECK(subsystem.getDefaultCommand() != nullptr);

  CommandScheduler::unregisterSubsystem(&subsystem);
}

// ---------------------------------------------------------------------------
// 9. schedule() must not end the same command twice while interrupting
// ---------------------------------------------------------------------------

/// Retires another command from inside its own end(), the way Routine::end()
/// reaches the fire-and-forget commands trigger() left running.
class RetiresAnotherOnEndCommand : public Command {
public:
  RetiresAnotherOnEndCommand(Subsystem* subsystem, Command** victim)
      : m_subsystem(subsystem), m_victim(victim) {}

  int ends = 0;

  bool isFinished() override { return false; }

  void end(bool /*interrupted*/) override {
    ++ends;
    // Capped, so a scheduler that does recurse fails the check rather than
    // running out of stack.
    if (ends < 10 && m_victim != nullptr && *m_victim != nullptr) {
      CommandScheduler::endAndForget(*m_victim);
    }
  }

  std::vector<Subsystem*> getRequirements() override { return {m_subsystem}; }

private:
  Subsystem* m_subsystem = nullptr;
  Command** m_victim = nullptr;
};

/// Wants both subsystems, so scheduling it interrupts both holders at once.
class WantsBothCommand : public Command {
public:
  WantsBothCommand(Subsystem* first, Subsystem* second)
      : m_first(first), m_second(second) {}

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override {
    return {m_first, m_second};
  }

private:
  Subsystem* m_first = nullptr;
  Subsystem* m_second = nullptr;
};

void testInterruptingTwoCommandsEndsEachOnce() {
  std::printf("-- interrupting two commands ends each of them once\n");

  CountingSubsystem x;
  CountingSubsystem y;
  x.setName("x");
  y.setName("y");

  // A holds x, B holds y, and each retires the other from its own end(). Both
  // are in the intersection schedule() collects, so whichever the loop reaches
  // first retires the other, and the loop then reaches a command that has
  // already been retired.
  //
  // Mutual rather than one-directional on purpose. schedule() walks the
  // requirements map, whose order over two subsystem pointers is unspecified,
  // so a one-directional version only reproduces on about half of the runs.
  // This one does not care which way round the map yields them.
  Command* a_ptr = nullptr;
  Command* b_ptr = nullptr;

  RetiresAnotherOnEndCommand a(&x, &b_ptr);
  RetiresAnotherOnEndCommand b(&y, &a_ptr);
  a_ptr = &a;
  b_ptr = &b;

  CommandScheduler::schedule(&a);
  CommandScheduler::schedule(&b);
  CHECK(CommandScheduler::scheduled(&a));
  CHECK(CommandScheduler::scheduled(&b));

  // Wants both, so scheduling it interrupts both holders in one loop.
  WantsBothCommand incoming(&x, &y);
  CommandScheduler::schedule(&incoming);

  // The one that mattered: without the guard inside retire(), whichever command
  // the loop reaches second gets end() twice.
  CHECK_EQ(static_cast<double>(a.ends), 1.0);
  CHECK_EQ(static_cast<double>(b.ends), 1.0);

  CHECK(CommandScheduler::scheduled(&incoming));
  CHECK(!CommandScheduler::scheduled(&a));
  CHECK(!CommandScheduler::scheduled(&b));

  CommandScheduler::endAndForget(&incoming);
  CommandScheduler::unregisterSubsystem(&x);
  CommandScheduler::unregisterSubsystem(&y);
}

// ---------------------------------------------------------------------------
// 10. A retired command must survive a callback that nests inside its end()
// ---------------------------------------------------------------------------

/// Set by the outgoing default command after its own end() has nested.
int g_nested_reads = 0;

/// Schedules a replacement from its own end(), and keeps reading its members
/// afterwards. The replacement's initialize() calls setDefaultCommand on the
/// same subsystem, which is the nesting that made activeCommand() name the
/// replacement rather than this command.
class NestingOnEndCommand : public Command {
public:
  NestingOnEndCommand(Subsystem* subsystem, Command** replacement)
      : m_subsystem(subsystem), m_replacement(replacement) {}

  bool isFinished() override { return false; }

  void end(bool /*interrupted*/) override {
    if (m_replacement != nullptr && *m_replacement != nullptr) {
      CommandScheduler::schedule(*m_replacement);
    }

    // Reads of *this* after the nested callback returned. If the nested
    // setDefaultCommand freed this command, these are freed memory.
    ++m_touches;
    g_nested_reads = m_touches;
  }

  std::vector<Subsystem*> getRequirements() override { return {m_subsystem}; }

private:
  Subsystem* m_subsystem = nullptr;
  Command** m_replacement = nullptr;
  int m_touches = 0;
};

/// Calls setDefaultCommand on the given subsystem from its own initialize().
class SwapsDefaultOnInitCommand : public Command {
public:
  explicit SwapsDefaultOnInitCommand(Subsystem* subsystem)
      : m_subsystem(subsystem) {}

  void initialize() override {
    if (m_done) {
      return;
    }
    m_done = true;
    m_subsystem->setDefaultCommand(m_subsystem->startEnd([]() {}, []() {}));
  }

  bool isFinished() override { return false; }

  std::vector<Subsystem*> getRequirements() override { return {}; }

private:
  Subsystem* m_subsystem = nullptr;
  bool m_done = false;
};

void testRetiredCommandSurvivesANestedCallback() {
  std::printf("-- a retiring command survives a callback nested in its end()\n");

  g_nested_reads = 0;

  CountingSubsystem subsystem;
  subsystem.setName("nesting");

  SwapsDefaultOnInitCommand replacement(&subsystem);
  Command* replacement_ptr = &replacement;

  // The subsystem's default command is the one that nests. Retiring it runs its
  // end(), which schedules the replacement; the replacement's initialize() then
  // calls setDefaultCommand on this same subsystem. At that moment the
  // scheduler's innermost active command is the replacement, so a check against
  // activeCommand() alone would have freed the command still running its end().
  subsystem.setDefaultCommand(
      std::make_unique<NestingOnEndCommand>(&subsystem, &replacement_ptr));
  subsystem.registerSelf();

  CommandScheduler::run();  // schedules the nesting default command

  Command* nesting = subsystem.getDefaultCommand();
  CHECK(nesting != nullptr);
  CHECK(CommandScheduler::scheduled(nesting));

  // Retire it. Everything above happens inside this call.
  CommandScheduler::endAndForget(nesting);

  // Under ASan this is where the use-after-free fired. The value proves the
  // post-nesting read happened and landed in live memory.
  CHECK_EQ(static_cast<double>(g_nested_reads), 1.0);

  CommandScheduler::endAndForget(&replacement);
  CommandScheduler::unregisterSubsystem(&subsystem);
}

}  // namespace

int main() {
  testScopedManager();
  testDefaultCommandSwapInsideRunLoop();
  testUnregisterInsideRunLoop();
  testForgetDuringDeferredDrain();
  testDefaultCommandReplacesItself();
  testFinishedCommandEndsOnce();
  testEndSchedulingAReplacementDoesNotRecurse();
  testEndAndForgetSchedulingAReplacementDoesNotRecurse();
  testSelfReplacingCommandSurvivesItsOwnFrame();
  testInterruptingTwoCommandsEndsEachOnce();
  testRetiredCommandSurvivesANestedCallback();
  return mclib::test::summary("scheduler_lifetime");
}
