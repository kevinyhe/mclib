// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/command.h"
#include "mclib/command/parallelRaceGroup.h"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include <memory>

/**
 * @brief Creates a \refitem Command with no requirements that finishes after a user-specified duration
 */
class WaitCommand : public Command {
	QTime startTime;
	QTime duration;
public:
	/**
	 * @brief Creates a new WaitCommand that runs for a user-specified duration
	 *
	 * @param duration The duration in QTime to run this \refitem Command
	 */
	explicit WaitCommand(const QTime &duration)
		: duration(duration) {
	}

	/**
	 * @brief Initializes the WaitCommand and sets the start time of the WaitCommand
	 */
	void initialize() override {
		startTime = mclib::time::now();
	}

	/**
	 * @brief Returns when the WaitCommand's duration has passed
	 *
	 * @return Returns true if the duration has passed, false otherwise
	 */
	bool isFinished() override {
		return mclib::time::now() - startTime > duration;
	}

	~WaitCommand() override = default;
};

inline std::unique_ptr<Command> Command::withTimeout(const QTime duration) {
	// The WaitCommand has no other owner, so the group it goes into has
	// to be the thing that destroys it.
	auto wait = std::make_unique<WaitCommand>(duration);
	std::unique_ptr<ParallelRaceGroup> group(new ParallelRaceGroup({wait.get(), this}));
	group->ownedHelpers.push_back(std::move(wait));
	return group;
}

