// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/command.h"

#include <memory>

/**
 * @brief Makes a \refitem Command repeat each time after it is run
 */
class RepeatCommand : public Command {
private:
	Command* command;
public:
	/**
	 * @brief Create a RepeatCommand with a \refitem Command pointer
	 *
	 * @param command The \refitem Command to make into a RepeatCommand
	 */
	explicit RepeatCommand(Command* command) {
		this->command = command;
	}

	/**
	 * @brief Just initializes the \refitem Command
	 */
	void initialize() override {
		command->initialize();
	}

	/**
	 * @brief Executes the \refitem Command, if it is finished it restarts the \refitem Command
	 */
	void execute() override {
		command->execute();

		if (command->isFinished()) {
			command->end(false);
			command->initialize();
		}
	}

	/**
	 * @brief Ends the \refitem Command with interrupted set to true
	 *
	 * @param interrupted Ignored, if the command is ended it must be interrupted because it is always restarting
	 */
	void end(bool /*interrupted*/) override {
		command->end(true);
	}

	/**
	 * @brief Passes on the requirements of the \refitem Command that was passed in
	 *
	 * @return The requirements of the \refitem Command that was passed in
	 */
	std::vector<Subsystem *> getRequirements() override {
		return command->getRequirements();
	}

	~RepeatCommand() override = default;
};

inline std::unique_ptr<Command> Command::repeatedly() {
	return std::make_unique<RepeatCommand>(this);
}

