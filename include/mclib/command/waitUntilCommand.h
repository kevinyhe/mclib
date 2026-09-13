// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/command/parallelRaceGroup.h"

#include <memory>

/**
 * @brief WaitUntilCommand creates a command that ends once a condition is finished. This command has no requirements.
 */
class WaitUntilCommand : public FunctionalCommand {
public:
	/**
	 * @brief Create a new WaitUntilCommand object with an isFinish conditional
	 *
	 * @param is_finish The conditional to end with. Once it is true the command will finish
	 */
	explicit WaitUntilCommand(const std::function<bool()> &is_finish)
		: FunctionalCommand([]() {}, []() {}, [](bool /*interrupted*/) {}, is_finish, {}) {
	}

	~WaitUntilCommand() override = default;
};

inline std::unique_ptr<Command> Command::until(const std::function<bool()>& isFinish) {
	// Same ownership story as withTimeout(): the group owns the helper.
	auto wait = std::make_unique<WaitUntilCommand>(isFinish);
	std::unique_ptr<ParallelRaceGroup> group(new ParallelRaceGroup({wait.get(), this}));
	group->ownedHelpers.push_back(std::move(wait));
	return group;
}
