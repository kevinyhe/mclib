// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/functionalCommand.h"

/**
 * \refitem Command that runs and initialize function instantly, then ends immediately after first update
 */
class InstantCommand : public FunctionalCommand {
public:
	/**
	 * Create a new InstantCommand
	 *
	 * @param on_init Function to run once upon function start
	 * @param requirements Subsystem requirements for this command
	 */
	InstantCommand(const std::function<void()> &on_init, const std::initializer_list<Subsystem *> &requirements)
		: FunctionalCommand(on_init, [] {}, [](bool /*interrupted*/) {}, [] { return true; } , requirements) {
	}

	~InstantCommand() override = default;
};
