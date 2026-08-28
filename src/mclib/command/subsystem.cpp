// mclib
#include "mclib/command/subsystem.h"

#include "mclib/command/commandScheduler.h"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/runCommand.h"

#include <algorithm>
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

	// The old default command is on its way out. End it cleanly and then scrub
	// every remaining reference to it, so the scheduler is never left holding a
	// dangling pointer.
	//
	// endAndForget, not cancel() plus forgetCommand(). Called from inside a
	// command's execute(), cancel() only queues the command and forgetCommand()
	// erases that queue entry again, so end(true) would never run: a startEnd()
	// default would never fire its on_end.
	if (previous != nullptr) {
		CommandScheduler::endAndForget(previous);
	}

	// Point an existing registration at the new command. A no-op if this
	// subsystem was never registered.
	CommandScheduler::setDefaultCommand(this, command.get());

	// Retire rather than destroy. A default command is allowed to call this from
	// inside its own execute(), and destroying it here would leave that frame
	// reading freed memory the moment it touched a member after the call.
	if (default_command != nullptr) {
		retired_default_commands.push_back(std::move(default_command));
	}

	default_command = std::move(command);

	// Release everything retired whose callback is no longer on the stack. On
	// the ordinary path, where the caller is not the outgoing command, that is
	// the command just retired and it goes immediately. On the self-replacing
	// path it is held until a later call, by which time the frame has returned.
	//
	// isActive, not activeCommand. The chain can nest: retiring A runs A->end(),
	// which can schedule a replacement whose initialize() lands back here. The
	// innermost active command is then the replacement, not A, and comparing
	// against it would free A while A's own end() frame was still running.
	std::erase_if(retired_default_commands,
	              [](const std::unique_ptr<Command>& retired) {
		              return !CommandScheduler::isActive(retired.get());
	              });
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

	// Retired commands were already ended and forgotten on the way out, but a
	// caller can have re-scheduled one since. Cheap to be certain.
	for (const std::unique_ptr<Command>& retired : retired_default_commands) {
		CommandScheduler::forgetCommand(retired.get());
	}
}
