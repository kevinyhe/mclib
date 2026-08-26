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

public:
	// Singleton pattern
	static CommandScheduler &getInstance()
	{
		static CommandScheduler instance;
		return instance;
	}

	static void registerSubsystem(Subsystem *subsystem, Command *default_command)
	{
		CommandScheduler &instance = getInstance();

		// Make sure the subsystem isn't already registered
		assert(!instance.subsystems.contains(subsystem));

		// Make sure the default command isn't null
		assert(default_command != nullptr);

		instance.subsystems[subsystem] = default_command;
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
				intersection.push_back(requirement.second);
			}
		}

		if (all_interruptible)
		{
			for (auto intersect : intersection)
			{
				intersect->end(true);
				std::erase(instance.scheduledCommands, intersect);
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

		// Run the periodic for all registered subsystems
		for (const auto &pair : instance.subsystems)
		{
			pair.first->periodic();
		}

		// Poll user set event loops
		instance.eventLoop.poll();

		// Only poll teleop tasks when the robot controller is active (Controller buttons)
		if (!pros::competition::is_autonomous() && !pros::competition::is_disabled())
		{
			instance.teleopEventLoop.poll();
		}

		instance.inRunLoop = true;

		for (auto command : instance.scheduledCommands)
		{
			command->execute();

			if (command->isFinished())
			{
				command->end(false);

				for (auto requirement : command->getRequirements())
				{
					instance.requirements.erase(requirement);
				}

				std::erase(instance.scheduledCommands, command);
			}
		}

		instance.inRunLoop = false;

		for (const auto command : instance.toCancel)
		{
			cancel(command);
		}

		for (const auto command : instance.toSchedule)
		{
			schedule(command);
		}

		instance.toCancel.clear();
		instance.toSchedule.clear();

		for (auto [subsystem, command] : instance.subsystems)
		{
			if (!instance.requirements.contains(subsystem))
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

		command->end(true);

		std::erase(instance.scheduledCommands, command);

		for (auto requirement : command->getRequirements())
		{
			instance.requirements.erase(requirement);
		}
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
