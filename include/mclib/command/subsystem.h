// mclib
#pragma once

#include <functional>
#include <memory>

class Command;

/**
 * @brief Abstract class for subsystem behaviors. Look at the [annotated intake example](../tutorials/intakeExample.md)
 * for a more in depth solution
 */
class Subsystem {
public:
	/**
	 * Period is run every frame by the \refitem CommandScheduler useful for debugging tasks and feedback controllers
	 * that need to run every frame
	 */
	virtual void periodic() {}

	std::unique_ptr<Command> run(std::function<void()> on_execute);
	std::unique_ptr<Command> runOnce(std::function<void()> on_initialize);
	std::unique_ptr<Command> startEnd(std::function<void()> on_initialize,
	                                  std::function<void()> on_end);
	std::unique_ptr<Command> runUntil(std::function<void()> on_execute,
	                                  std::function<bool()> is_finished);
	std::unique_ptr<Command> idleCommand();

	virtual ~Subsystem() = default;
};
