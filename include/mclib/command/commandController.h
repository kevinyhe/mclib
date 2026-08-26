// mclib
#pragma once

#include "mclib/command/commandScheduler.h"
#include "mclib/command/trigger.h"
#include "mclib/device/controller.hpp"

/**
 * @brief Command Controller wraps a controller and exposes Trigger helpers for command-based code.
 * based code
 *
 * ```c
 * // Initialize CommandController
 * CommandController primary(mclib::device::ControllerId::Master);
 *
 * Command *command;
 *
 * // Register trigger to toggle command on pressing R2
 * primary.getTrigger(mclib::device::DigitalButton::R2)->toggleOnTrue(command);
 * ```
 */
class CommandController {
public:
	/**
	 * @brief Construct a new CommandController
	 *
	 * @param id Master, Partner controller ID
	 */
	explicit CommandController(mclib::device::ControllerId id = mclib::device::ControllerId::Master)
		: controller(id) {}

	std::int32_t getAnalog(mclib::device::AnalogAxis axis) const {
		return controller.getAnalog(axis);
	}

	bool getDigital(mclib::device::DigitalButton button) const {
		return controller.getDigital(button);
	}

	/**
	 * @brief Create trigger from button
	 *
	 * @param button The button to create a \refitem Trigger on
	 * @return \refitem Trigger with the boolean as the desired button
	 */
	Trigger *getTrigger(mclib::device::DigitalButton button) {
		return new Trigger([this, button]() { return this->getDigital(button); },
						   CommandScheduler::getTeleopEventLoop());
	}

private:
	mclib::device::Controller controller;
};
