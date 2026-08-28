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

	// Every command the scheduler is currently inside a callback of, outermost
	// first. A stack rather than one pointer because the two questions asked of
	// it are different: activeCommand() wants the innermost, isActive() wants to
	// know whether a command is anywhere on it
	std::vector<Command *> activeStack;

	CommandScheduler() = default;

	// Marks a command active for the duration of one callback into it. Nests,
	// because a callback is allowed to schedule or cancel other commands
	class ActiveScope
	{
	public:
		explicit ActiveScope(Command *command)
		{
			getInstance().activeStack.push_back(command);
		}

		~ActiveScope() { getInstance().activeStack.pop_back(); }

		ActiveScope(const ActiveScope &) = delete;
		ActiveScope &operator=(const ActiveScope &) = delete;
		ActiveScope(ActiveScope &&) = delete;
		ActiveScope &operator=(ActiveScope &&) = delete;
	};

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

	/**
	 * @brief Take a command out of the scheduler and then run its end()
	 *
	 * @details The one ordering every retirement path uses. Both maps are
	 * cleared BEFORE end() is called, and that order is the whole point:
	 *
	 * - scheduledCommands first, so anything end() reaches that asks whether
	 *   this command is scheduled sees it as already gone and cannot give it a
	 *   second end().
	 * - requirements second, so schedule() cannot find the dying command as the
	 *   owner of a subsystem. It picks who to interrupt out of the requirements
	 *   map, not out of scheduledCommands, so a replacement scheduled from
	 *   inside end() used to find the command that was still ending, call
	 *   end(true) on it again, and recurse until the stack ran out.
	 *
	 * Releasing first also removes the reason the release was deferred: a
	 * replacement scheduled from end() claims its requirements into an entry
	 * nothing here touches afterwards.
	 *
	 * end() runs only if the command was still scheduled on the way in. That
	 * check belongs HERE and not in the callers, because not every caller can
	 * make it: schedule() interrupts a list of commands it collected before any
	 * of them ran, and the first one's end() is free to retire a later one in
	 * that same list, directly or through Subsystem::setDefaultCommand. Putting
	 * the guard in the one place they all share covers every path, including the
	 * next one somebody adds.
	 *
	 * The map cleanup runs either way. It is idempotent, and a command somehow
	 * out of scheduledCommands while still owning a subsystem is exactly the
	 * state that leaves a dead command holding it forever.
	 *
	 * @param command Must be non-null. Need not be scheduled.
	 * @param interrupted Passed straight to end().
	 */
	static void retire(Command *command, bool interrupted)
	{
		CommandScheduler &instance = getInstance();

		// Read before anything is released, so the release covers what the
		// command actually held on the way in
		auto held = command->getRequirements();

		// Read before the erase below, which is what would make it false
		const bool was_scheduled = scheduled(command);

		std::erase(instance.scheduledCommands, command);

		releaseRequirements(command, held);

		if (!was_scheduled)
		{
			// Already retired by an end() that ran earlier in this same pass.
			// Ending it again would fire a startEnd()'s on_end twice and stop an
			// async command's task twice.
			return;
		}

		ActiveScope active(command);

		command->end(interrupted);
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
	 * This runs end(true) immediately instead. That is safe inside the run loop
	 * as far as the SCHEDULER is concerned: run() iterates a copy of the
	 * scheduled list and re-checks scheduled() both before and after execute(),
	 * so a command retired here is skipped rather than executed or asked
	 * isFinished() afterwards.
	 *
	 * It says nothing about the command OBJECT. If the caller destroys it, that
	 * is the caller's problem to sequence - see \refitem
	 * Subsystem::setDefaultCommand, which keeps the outgoing command alive
	 * precisely because it can be called from inside that command's own
	 * execute().
	 *
	 * Use forgetCommand() on its own when running end() would be unsafe, such as
	 * from a subsystem destructor.
	 *
	 * @param command The command to end and forget. Null, or a command that is
	 * not scheduled, still has every reference to it dropped.
	 */
	static void endAndForget(Command *command)
	{
		if (command == nullptr)
		{
			return;
		}

		// retire() is a no-op on a command that is not scheduled
		retire(command, true);

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
				// retire() releases EVERY subsystem the interrupted command held,
				// not just the ones the incoming command wants. Otherwise a
				// subsystem it held alone stays owned by a dead command forever
				// and its default command never restarts. It also clears both maps
				// before end(true), so a replacement scheduled from inside that
				// end() cannot find this command and interrupt it again.
				retire(intersect, true);
			}

			for (auto requirement : requirements)
			{
				instance.requirements[requirement] = command;
			}

			{
				ActiveScope active(command);

				command->initialize();
			}

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

			{
				ActiveScope active(command);

				command->execute();
			}

			// execute() may have retired this command and freed it. A default
			// command that replaces itself, intake.setDefaultCommand(...) from
			// inside its own execute(), is the shape: endAndForget runs its end()
			// and the unique_ptr assignment right after destroys it. Re-check
			// before touching the pointer again. scheduled() only compares
			// pointer values, so it is safe on a command that is already gone.
			if (!scheduled(command))
			{
				continue;
			}

			if (command->isFinished())
			{
				// retire() drops it from both maps before end(false) runs. Doing
				// that inside the loop rather than after it is what stops a later
				// command in this same pass reaching a finished command through
				// endAndForget and giving it a second end(): a startEnd() on_end
				// firing twice, an async command stopped twice.
				retire(command, false);
			}
		}

		instance.inRunLoop = false;

		// Drain destructively, one entry at a time. With inRunLoop false again
		// these cancel() and schedule() calls run user callbacks, and a callback
		// is allowed to cancel, schedule or forget a command. Popping the front
		// keeps forgetCommand() able to scrub entries that have not run yet:
		// copying the queue aside would leave this loop acting on commands that
		// were forgotten, or destroyed, part-way through the drain. It also never
		// walks a vector that is being written.
		//
		// Neither queue can grow here. With inRunLoop false, schedule() and
		// cancel() act immediately instead of queueing, so each pop shrinks the
		// queue for good and the loops terminate.
		while (!instance.toCancel.empty())
		{
			Command *command = instance.toCancel.front();

			instance.toCancel.erase(instance.toCancel.begin());

			// A no-op on a command an earlier cancel already forgot
			cancel(command);
		}

		while (!instance.toSchedule.empty())
		{
			Command *command = instance.toSchedule.front();

			instance.toSchedule.erase(instance.toSchedule.begin());

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

	/**
	 * @brief The INNERMOST command the scheduler is currently inside a callback of
	 *
	 * @details Set for the duration of the initialize(), execute() and end()
	 * calls the scheduler itself makes. Those nest - an end() may schedule a
	 * replacement, whose initialize() runs inside it - and this returns the
	 * innermost of them, the one whose callback is actually running.
	 *
	 * A command the scheduler did not drive into never appears. A Routine runs
	 * its steps by calling their execute() itself, so inside a step it is still
	 * the Routine that is active. That is exactly what the triggered commands
	 * want: the innermost command the SCHEDULER reached is the one holding the
	 * reservation, however deeply the Routine running the trigger is nested.
	 *
	 * Use isActive() instead to ask whether a particular command is anywhere in
	 * the current callback chain. That is a different question, and answering it
	 * with this would be wrong the moment a callback nests.
	 *
	 * @return The innermost active command, or nullptr outside any scheduler
	 * callback.
	 */
	static Command *activeCommand()
	{
		CommandScheduler &instance = getInstance();

		return instance.activeStack.empty() ? nullptr : instance.activeStack.back();
	}

	/**
	 * @brief Is this command anywhere in the callback chain running right now
	 *
	 * @details True while any of the scheduler's calls into @p command is still
	 * on the stack, including when a nested callback is running inside it. That
	 * is what a caller about to DESTROY a command needs to know, and it is not
	 * what activeCommand() answers: during
	 * `retire(A) -> A->end() -> schedule(R) -> R->initialize()` the active
	 * command is R, but A's frame is still live and freeing A there would leave
	 * end() running on freed memory.
	 *
	 * \refitem Subsystem::setDefaultCommand uses this to decide whether the
	 * outgoing default command can be destroyed yet.
	 *
	 * @param command The command to look for. Null is never active.
	 * @return true if the scheduler is currently inside a callback of it.
	 */
	static bool isActive(const Command *command)
	{
		CommandScheduler &instance = getInstance();

		if (command == nullptr)
		{
			return false;
		}

		return std::find(instance.activeStack.begin(), instance.activeStack.end(), command) !=
		       instance.activeStack.end();
	}

	/**
	 * @brief The default command a registered subsystem is currently pointed at
	 *
	 * @details The raw pointer stored by registerSubsystem or setDefaultCommand,
	 * which is NOT necessarily what \refitem Subsystem::getDefaultCommand
	 * returns: the two-argument registerSubsystem takes any command, and a
	 * registration made by somebody else wins over a later duplicate.
	 *
	 * Owners use this to tell "my command is the one registered here" from "the
	 * subsystem was already registered by someone else and my call did nothing",
	 * which matters before touching a registration on the way out.
	 *
	 * @param subsystem The subsystem to look up.
	 * @return The registered default command, or nullptr if the subsystem is not
	 * registered or was registered with no command.
	 */
	static Command *getDefaultCommand(Subsystem *subsystem)
	{
		CommandScheduler &instance = getInstance();

		auto registration = instance.subsystems.find(subsystem);

		return registration == instance.subsystems.end() ? nullptr : registration->second;
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

		retire(command, true);
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
