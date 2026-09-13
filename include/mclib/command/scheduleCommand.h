// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/command.h"
#include "mclib/command/instantCommand.h"

/**
 * @brief This \refitem InstantCommand schedules a \refitem Command on object initialization
 */
class ScheduleCommand : public InstantCommand {
public:
	/**
	 * @brief Creates a \refitem InstantCommand that schedules a \refitem Command on initalization
	 * @param command The \refitem Command to schedule on initialization
	 */
	explicit ScheduleCommand(Command* command)
		: InstantCommand([command]() { command->schedule(); }, {}) {
	}

	~ScheduleCommand() override = default;
};
