// mclib
#include "mclib/command/subsystem.h"

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
