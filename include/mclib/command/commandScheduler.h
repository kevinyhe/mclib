// mclib
#pragma once

#include <algorithm>
#include <cassert>
#include <optional>
#include <unordered_map>
#include <vector>
#include "mclib/command/command.h"
#include "mclib/command/subsystem.h"
#include "mclib/command/eventLoop.h"
#include "pros/misc.hpp"

// Like WPILib's CommandScheduler class
class CommandScheduler
{
private:
	std::unordered_map<Subsystem *, Command *> subsystems;
	std::unordered_map<Subsystem *, Command *> requirements;
	std::vector<Command *> scheduledCommands;

	EventLoop teleopEventLoop{};
	EventLoop eventLoop{};

	bool inRunLoop = false;

	std::vector<Command *> toSchedule;
	std::vector<Command *> toCancel;

	CommandScheduler() = default;

	// Snapshot of the registered subsystems, so callbacks can register or
	// unregister subsystems without invalidating an in-flight iteration
	static std::vector<Subsystem *> registeredSubsystems()
	{
		CommandScheduler &instance = getInstance();

		std::vector<Subsystem *> subsystems;
		subsystems.reserve(instance.subsystems.size());

		for (const auto &pair : instance.subsystems)
		{
			subsystems.push_back(pair.first);
		}

		return subsystems;
	}

	// Give back only the subsystems still owned by command. An entry that some
	// other command claimed in the meantime, from inside an end() callback, must
	// be left alone.
	static void releaseRequirements(Command *command, const std::vector<Subsystem *> &held)
	{
		CommandScheduler &instance = getInstance();

		for (auto requirement : held)
		{
			auto owner = instance.requirements.find(requirement);

			if (owner != instance.requirements.end() && owner->second == command)
			{
				instance.requirements.erase(owner);
			}
		}
	}

public:
	// Singleton pattern
	static CommandScheduler &getInstance()
	{
		static CommandScheduler instance;
		return instance;
	}

	/**
	 * @brief Register a subsystem and the default command to run on it
	 *
	 * @details The scheduler calls \refitem Subsystem::runPeriodic on every
	 * registered subsystem each frame, and re-schedules the default command
	 * whenever nothing else requires the subsystem. The scheduler does NOT take
	 * ownership of default_command, the caller must keep it alive. Prefer
	 * \refitem Subsystem::setDefaultCommand plus \refitem Subsystem::registerSelf,
	 * which makes the subsystem the owner.
	 *
	 * ```C
	 * // command must outlive the scheduler registration
	 * std::unique_ptr<Command> intake_idle = intake.makeDisableCommand();
	 * CommandScheduler::registerSubsystem(&intake, intake_idle.get());
	 * ```
	 *
	 * @param subsystem The subsystem to register. Ignored if null or already
	 * registered.
	 * @param default_command Non owning pointer to the default command. May be
	 * null, the subsystem is still registered and still gets runPeriodic() every
	 * frame, it just has no default command.
	 */
	static void registerSubsystem(Subsystem *subsystem, Command *default_command)
	{
		CommandScheduler &instance = getInstance();

		// Ignore null subsystems and double registration instead of asserting,
		// asserts compile out in release builds
		if (subsystem == nullptr || instance.subsystems.contains(subsystem))
		{
			return;
		}

		// A null default command still registers the subsystem, runPeriodic()
		// matters even with no command attached
		instance.subsystems[subsystem] = default_command;
	}

	/**
	 * @brief Register a subsystem using the default command it already owns
	 *
	 * @details Reads \refitem Subsystem::getDefaultCommand, so call \refitem
	 * Subsystem::setDefaultCommand first if you want a default command. A subsystem
	 * with no default command is still registered, its \refitem
	 * Subsystem::runPeriodic runs every frame with no command attached.
	 *
	 * ```C
	 * intake.setDefaultCommand(intake.makeDisableCommand());
	 * CommandScheduler::registerSubsystem(&intake);
	 * ```
	 *
	 * @param subsystem The subsystem to register. Ignored if null or already
	 * registered.
	 */
	static void registerSubsystem(Subsystem *subsystem)
	{
		CommandScheduler &instance = getInstance();

		if (subsystem == nullptr || instance.subsystems.contains(subsystem))
		{
			return;
		}

		instance.subsystems[subsystem] = subsystem->getDefaultCommand();
	}

	/**
	 * @brief Drop every reference the scheduler holds to a command WITHOUT calling
	 * end() on it
	 *
	 * @details Removes the command from the scheduled list, from the deferred
	 * schedule and cancel queues, and from any requirement entry that still points
	 * at it. Use this when the command object is about to be destroyed and running
	 * its end() callback would be unsafe, for example from a destructor. Prefer
	 * cancel() when the command is still alive and should end cleanly.
	 *
	 * @param command The command to forget. Null is a no-op.
	 */
	static void forgetCommand(Command *command)
	{
		CommandScheduler &instance = getInstance();

		if (command == nullptr)
		{
			return;
		}

		std::erase(instance.scheduledCommands, command);
		std::erase(instance.toSchedule, command);
		std::erase(instance.toCancel, command);

		for (auto it = instance.requirements.begin(); it != instance.requirements.end();)
		{
			if (it->second == command)
			{
				it = instance.requirements.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

	/**
	 * @brief End a command NOW and then drop every reference the scheduler holds
	 * to it
	 *
	 * @details The safe way to retire a command that is about to be destroyed
	 * while it is still fully alive, for example the old default command inside
	 * \refitem Subsystem::setDefaultCommand.
	 *
	 * cancel() followed by forgetCommand() looks like it does this but does not.
	 * Inside \refitem CommandScheduler::run, cancel() only queues the command on
	 * the deferred cancel list, and the forgetCommand() call right after erases
	 * that queue entry again, so end(true) never runs at all: a startEnd()
	 * default never fires its on_end, an async command never stops its task.
	 * This runs end(true) immediately instead, which is safe inside the run loop
	 * because run() iterates a copy of the scheduled list and re-checks
	 * scheduled() before executing each entry.
	 *
	 * Use forgetCommand() on its own when running end() would be unsafe, such as
	 * from a subsystem destructor.
	 *
	 * @param command The command to end and forget. Null, or a command that is
	 * not scheduled, still has every reference to it dropped.
	 */
	static void endAndForget(Command *command)
	{
		CommandScheduler &instance = getInstance();

		if (command == nullptr)
		{
			return;
		}

		if (scheduled(command))
		{
			// Same ordering rule as cancel(): read the requirements before
			// end(true), which may schedule a replacement that claims some of them
			auto held = command->getRequirements();

			command->end(true);

			std::erase(instance.scheduledCommands, command);

			releaseRequirements(command, held);
		}

		forgetCommand(command);
	}

	/**
	 * @brief Drop every reference the scheduler holds to a subsystem WITHOUT
	 * cancelling anything
	 *
	 * @details Removes the subsystem's registration and requirement entries, so
	 * \refitem Subsystem::runPeriodic stops being called on it. Commands that
	 * required it are left alone. Use this from a destructor, where running command
	 * callbacks against a half destroyed object is unsafe. Prefer
	 * unregisterSubsystem otherwise.
	 *
	 * @param subsystem The subsystem to forget. Null is a no-op.
	 */
	static void forgetSubsystem(Subsystem *subsystem)
	{
		CommandScheduler &instance = getInstance();

		if (subsystem == nullptr)
		{
			return;
		}

		instance.requirements.erase(subsystem);
		instance.subsystems.erase(subsystem);
	}

	/**
	 * @brief Point an already registered subsystem at a different default command
	 *
	 * @details Only updates the stored pointer. It does not cancel the old default
	 * command and it does not register an unregistered subsystem, \refitem
	 * Subsystem::setDefaultCommand handles both around this call.
	 *
	 * @param subsystem The registered subsystem
	 * @param default_command Non owning pointer to the new default command, may be
	 * null
	 */
	static void setDefaultCommand(Subsystem *subsystem, Command *default_command)
	{
		CommandScheduler &instance = getInstance();

		if (subsystem == nullptr)
		{
			return;
		}

		auto registration = instance.subsystems.find(subsystem);

		if (registration != instance.subsystems.end())
		{
			registration->second = default_command;
		}
	}

	/**
	 * @brief Remove a subsystem from the scheduler
	 *
	 * @details Cancels the command currently requiring the subsystem and the
	 * subsystem's default command, drops the subsystem's requirement entry, and
	 * stops its \refitem Subsystem::runPeriodic from being called. Safe to call on
	 * a subsystem that was never registered.
	 *
	 * @param subsystem The subsystem to unregister
	 */
	static void unregisterSubsystem(Subsystem *subsystem)
	{
		CommandScheduler &instance = getInstance();

		if (subsystem == nullptr)
		{
			return;
		}

		auto requiring = instance.requirements.find(subsystem);

		if (requiring != instance.requirements.end())
		{
			// Copy the pointer, ending the command erases the map entry we are
			// looking at. endAndForget rather than cancel() plus forgetCommand():
			// called from inside run(), that pair queues the cancel and then erases
			// the queue entry, so end(true) never runs.
			Command *command = requiring->second;

			endAndForget(command);
		}

		auto registration = instance.subsystems.find(subsystem);

		if (registration != instance.subsystems.end() && registration->second != nullptr)
		{
			// A default command with no declared requirements never shows up in
			// the requirements map, so cancel it explicitly
			Command *default_command = registration->second;

			endAndForget(default_command);
		}

		forgetSubsystem(subsystem);
	}

	static void schedule(Command *command)
	{
		CommandScheduler &instance = getInstance();

		// Return if the command is already scheduled
		if (command == nullptr || scheduled(command))
		{
			return;
		}

		// return if competition is disabled
		if (pros::competition::is_disabled())
		{
			return;
		}

		if (instance.inRunLoop)
		{
			if (std::find(instance.toSchedule.begin(), instance.toSchedule.end(), command) == instance.toSchedule.end())
			{
				instance.toSchedule.emplace_back(command);
			}
			return;
		}

		std::vector<Command *> intersection;

		bool all_interruptible = true;

		auto requirements = command->getRequirements();

		for (auto requirement : instance.requirements)
		{
			if (std::find(requirements.begin(), requirements.end(), requirement.first) != requirements.end())
			{
				all_interruptible &= requirement.second->getCancelBehavior() == CommandCancelBehavior::CancelRunning;

				// One command can hold several of the required subsystems, only
				// interrupt it once
				if (std::find(intersection.begin(), intersection.end(), requirement.second) == intersection.end())
				{
					intersection.push_back(requirement.second);
				}
			}
		}

		if (all_interruptible)
		{
			for (auto intersect : intersection)
			{
				// Read the requirements BEFORE end(true). end() can schedule a
				// replacement command that claims some of them, and we must not
				// erase the entries that replacement just took.
				auto held = intersect->getRequirements();

				intersect->end(true);

				std::erase(instance.scheduledCommands, intersect);

				// Release EVERY subsystem the interrupted command held, not just
				// the ones the incoming command wants. Otherwise a subsystem it
				// held alone stays owned by a dead command forever and its
				// default command never restarts.
				releaseRequirements(intersect, held);
			}

			for (auto requirement : requirements)
			{
				instance.requirements[requirement] = command;
			}

			command->initialize();

			instance.scheduledCommands.push_back(command);
		}
	}

	static std::optional<Command *> getRequiring(Subsystem *subsystem)
	{
		CommandScheduler &instance = getInstance();

		if (instance.requirements.find(subsystem) != instance.requirements.end())
		{
			return instance.requirements[subsystem];
		}

		return std::nullopt;
	}

	static void run()
	{
		CommandScheduler &instance = getInstance();

		// Run the periodic for all registered subsystems. runPeriodic is non
		// virtual and skips disabled subsystems before calling periodic().
		// Iterate over a copy, a periodic() is allowed to unregister subsystems.
		for (auto subsystem : registeredSubsystems())
		{
			subsystem->runPeriodic();
		}

		// Poll user set event loops
		instance.eventLoop.poll();

		// Only poll teleop tasks when the robot controller is active (Controller buttons)
		if (!pros::competition::is_autonomous() && !pros::competition::is_disabled())
		{
			instance.teleopEventLoop.poll();
		}

		instance.inRunLoop = true;

		// Iterate over a copy, the loop body erases from scheduledCommands and
		// mutating the vector we are ranging over is undefined behavior
		std::vector<Command *> running = instance.scheduledCommands;

		std::vector<Command *> finished;

		for (auto command : running)
		{
			// A command may have been cancelled or forgotten by an earlier command
			// in this same pass. Cancels inside the run loop are deferred to
			// toCancel, so check that queue too or we would execute a command that
			// has already been cancelled.
			if (!scheduled(command) ||
			    std::find(instance.toCancel.begin(), instance.toCancel.end(), command) != instance.toCancel.end())
			{
				continue;
			}

			command->execute();

			if (command->isFinished())
			{
				// Same ordering rule as the interrupt path, read the requirements
				// before end() gets a chance to hand them to another command
				auto held = command->getRequirements();

				command->end(false);

				releaseRequirements(command, held);

				finished.push_back(command);
			}
		}

		for (auto command : finished)
		{
			std::erase(instance.scheduledCommands, command);
		}

		instance.inRunLoop = false;

		// Drain into locals before running anything. With inRunLoop false again
		// these cancel() and schedule() calls run user callbacks, and a callback
		// that cancels, schedules or forgets a command writes to the very vectors
		// a range-for would be walking. Swapping first leaves the callbacks a
		// fresh, empty queue to append to, which is picked up on the next pass.
		std::vector<Command *> pending_cancel;
		std::vector<Command *> pending_schedule;

		pending_cancel.swap(instance.toCancel);
		pending_schedule.swap(instance.toSchedule);

		for (const auto command : pending_cancel)
		{
			// cancel() is a no-op on a command an earlier cancel already forgot
			cancel(command);
		}

		for (const auto command : pending_schedule)
		{
			schedule(command);
		}

		// Copy again, schedule() runs command callbacks that may unregister a
		// subsystem and invalidate this iteration
		for (auto subsystem : registeredSubsystems())
		{
			auto registration = instance.subsystems.find(subsystem);

			if (registration == instance.subsystems.end())
			{
				continue;
			}

			Command *command = registration->second;

			// A subsystem registered before its default command was set stores a
			// null entry, fall back to whatever it owns now
			if (command == nullptr)
			{
				command = subsystem->getDefaultCommand();
			}

			if (command != nullptr && !instance.requirements.contains(subsystem))
			{
				schedule(command);
			}
		}
	}

	static bool scheduled(const Command *command)
	{
		CommandScheduler &instance = getInstance();

		return std::find(instance.scheduledCommands.begin(), instance.scheduledCommands.end(), command) != instance.scheduledCommands.end();
	}

	static EventLoop *getEventLoop()
	{
		CommandScheduler &instance = getInstance();

		return &instance.eventLoop;
	}

	static EventLoop *getTeleopEventLoop()
	{
		CommandScheduler &instance = getInstance();

		return &instance.teleopEventLoop;
	}

	static void cancel(Command *command)
	{
		CommandScheduler &instance = getInstance();

		if (!scheduled(command))
		{
			return;
		}

		if (instance.inRunLoop)
		{
			if (std::find(instance.toCancel.begin(), instance.toCancel.end(), command) == instance.toCancel.end())
			{
				instance.toCancel.emplace_back(command);
			}
			return;
		}

		// Read the requirements before end(true), it may schedule a replacement
		// that legitimately claims some of them
		auto held = command->getRequirements();

		command->end(true);

		std::erase(instance.scheduledCommands, command);

		releaseRequirements(command, held);
	}
};

inline void Command::schedule()
{
	CommandScheduler::schedule(this);
}

inline void Command::cancel()
{
	CommandScheduler::cancel(this);
}

inline bool Command::scheduled() const
{
	return CommandScheduler::scheduled(this);
}
