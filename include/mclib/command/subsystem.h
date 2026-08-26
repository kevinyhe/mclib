// mclib
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

class Command;

/**
 * @brief Abstract class for subsystem behaviors. Look at the [annotated intake example](../tutorials/intakeExample.md)
 * for a more in depth solution
 *
 * @details A Subsystem owns a piece of hardware and, optionally, the default
 * \refitem Command that runs on it whenever nothing else has claimed it. Because
 * every command factory returns a `std::unique_ptr<Command>` while the \refitem
 * CommandScheduler stores raw pointers, the subsystem is the natural owner of the
 * default command: hand it over with setDefaultCommand and it stays alive for as
 * long as the subsystem does.
 *
 * ```C
 * // Preferred registration pattern
 * Intake intake{...};
 *
 * void initialize() {
 *	intake.setName("intake");
 *	intake.setDefaultCommand(intake.makeDisableCommand());
 *	intake.registerSelf();
 * }
 * ```
 */
class Subsystem {
public:
	/**
	 * Period is run every frame by the \refitem CommandScheduler useful for debugging tasks and feedback controllers
	 * that need to run every frame
	 */
	virtual void periodic() {}

	/**
	 * @brief Entry point the \refitem CommandScheduler uses to tick this subsystem
	 *
	 * @details Non virtual on purpose. It checks isEnabled() and only then calls
	 * the virtual periodic(), so a disabled subsystem is skipped even when a
	 * subclass overrides periodic() (for example \refitem StateMechanism, which
	 * pushes its state to hardware from periodic()).
	 */
	void runPeriodic() {
		if (!enabled) {
			return;
		}

		periodic();
	}

	std::unique_ptr<Command> run(std::function<void()> on_execute);
	std::unique_ptr<Command> runOnce(std::function<void()> on_initialize);
	std::unique_ptr<Command> startEnd(std::function<void()> on_initialize,
	                                  std::function<void()> on_end);
	std::unique_ptr<Command> runUntil(std::function<void()> on_execute,
	                                  std::function<bool()> is_finished);
	std::unique_ptr<Command> idleCommand();

	/**
	 * @brief Give this subsystem ownership of its default command
	 *
	 * @details The subsystem keeps the command alive. Any previously held default
	 * command is cancelled and scrubbed from the \refitem CommandScheduler before
	 * it is destroyed, and an existing registration is repointed at the new
	 * command, so this is safe to call at any time.
	 *
	 * ```C
	 * intake.setDefaultCommand(intake.makeDisableCommand());
	 * ```
	 *
	 * @param command The command to own and run whenever nothing else requires
	 * this subsystem. May be nullptr to clear it.
	 */
	void setDefaultCommand(std::unique_ptr<Command> command);

	/**
	 * @brief Get the default command owned by this subsystem
	 *
	 * @return A non owning pointer to the default command, or nullptr if none was
	 * set. The subsystem keeps ownership, do not delete it.
	 */
	[[nodiscard]] Command *getDefaultCommand() const;

	/**
	 * @brief Register this subsystem with the \refitem CommandScheduler using its
	 * own stored default command
	 *
	 * @details Equivalent to `CommandScheduler::registerSubsystem(this)`. Call
	 * setDefaultCommand first if you want a default command, registering without
	 * one is allowed and simply means periodic() runs with no command attached.
	 * Registering twice is a no-op, use setDefaultCommand to change the default
	 * command afterwards.
	 *
	 * ```C
	 * arm.setDefaultCommand(arm.makeStopCommand());
	 * arm.registerSelf();
	 * ```
	 */
	void registerSelf();

	/**
	 * @brief Set a human readable name for this subsystem, useful for logging
	 *
	 * @param subsystem_name The name to store
	 */
	void setName(std::string subsystem_name) { name = std::move(subsystem_name); }

	/**
	 * @brief Get the human readable name of this subsystem
	 *
	 * @return The name set by setName, empty if it was never set
	 */
	[[nodiscard]] const std::string &getName() const { return name; }

	/**
	 * @brief Enable or disable this subsystem
	 *
	 * @details A disabled subsystem skips periodic(). Commands can still be
	 * scheduled against it, they simply have no effect until it is enabled again.
	 *
	 * @warning This does NOT stop the hardware. PROS motors latch the last voltage
	 * they were given, so a disabled \refitem StateMechanism keeps driving at
	 * whatever it last wrote. Command a safe state first if you want the mechanism
	 * to stop. Subsystems whose periodic() also integrates sensor deltas, such as
	 * \refitem ChassisController and its odometry, will miss everything that
	 * happens while disabled and fold it into one step when re-enabled.
	 *
	 * @param is_enabled true to run periodic(), false to skip it
	 */
	void setEnabled(bool is_enabled) { enabled = is_enabled; }

	/**
	 * @brief Check whether this subsystem runs its periodic()
	 *
	 * @return true if enabled (the default), false otherwise
	 */
	[[nodiscard]] bool isEnabled() const { return enabled; }

	/**
	 * @brief Unregisters from the \refitem CommandScheduler and destroys the
	 * default command it owns
	 */
	virtual ~Subsystem();

private:
	std::unique_ptr<Command> default_command;
	std::string name;
	bool enabled = true;
};
