// mclib
#include "mclib/command/subsystem.h"

#include "mclib/command/commandScheduler.h"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/runCommand.h"

#include <utility>

std::unique_ptr<Command> Subsystem::run(std::function<void()> on_execute) {
	return std::make_unique<RunCommand>(std::move(on_execute),
	                                    std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Subsystem::runOnce(std::function<void()> on_initialize) {
	return std::make_unique<InstantCommand>(std::move(on_initialize),
	                                        std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Subsystem::startEnd(std::function<void()> on_initialize,
                                             std::function<void()> on_end) {
	return std::make_unique<FunctionalCommand>(
	    std::move(on_initialize),
	    []() {},
	    [on_end = std::move(on_end)](bool) { on_end(); },
	    []() { return false; },
	    std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Subsystem::runUntil(std::function<void()> on_execute,
                                             std::function<bool()> is_finished) {
	return std::make_unique<FunctionalCommand>(
	    []() {},
	    std::move(on_execute),
	    [](bool) {},
	    std::move(is_finished),
	    std::initializer_list<Subsystem*>{this});
}

std::unique_ptr<Command> Subsystem::idleCommand() {
	return run([]() {});
}

void Subsystem::setDefaultCommand(std::unique_ptr<Command> command) {
	Command* previous = default_command.get();

	// The old default command is about to be destroyed. End it cleanly if the
	// scheduler is not mid-run, then scrub every remaining reference to it so the
	// scheduler is never left holding a dangling pointer.
	if (previous != nullptr) {
		CommandScheduler::cancel(previous);
		CommandScheduler::forgetCommand(previous);
	}

	// Point an existing registration at the new command. A no-op if this
	// subsystem was never registered.
	CommandScheduler::setDefaultCommand(this, command.get());

	default_command = std::move(command);
}

Command* Subsystem::getDefaultCommand() const {
	return default_command.get();
}

void Subsystem::registerSelf() {
	CommandScheduler::registerSubsystem(this);
}

Subsystem::~Subsystem() {
	// Drop the scheduler's references before default_command is destroyed.
	// forgetSubsystem and forgetCommand deliberately skip end() callbacks, running
	// user code against a half destroyed subsystem would be worse than skipping it.
	CommandScheduler::forgetSubsystem(this);
	CommandScheduler::forgetCommand(default_command.get());
}
