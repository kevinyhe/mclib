// mclib
#pragma once

#include "mclib/command/command.h"
#include "mclib/command/subsystem.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief Owns mechanisms' default commands and registers them with the
 * CommandScheduler.
 *
 * The scheduler stores raw `Command*` but every mechanism factory returns a
 * `std::unique_ptr<Command>`, so a default command has to be kept alive by
 * somebody for as long as the scheduler can reach it. Without a manager that
 * means one global `std::unique_ptr<Command>` per mechanism. This class holds
 * those pointers instead: it takes ownership in add() and keeps them until the
 * manager itself is destroyed.
 *
 * The manager owns the commands the scheduler points at, so its destructor
 * ends them and scrubs them out of the scheduler. Program-long storage is still
 * the usual shape, but a manager held as a class member or a local is safe from
 * the scheduler's side: a CommandScheduler::run() after it dies no longer
 * reaches freed commands.
 *
 * It does move the ordering requirement rather than remove it. The destructor
 * runs end(true) on each owned command, and those commands hold the mechanism,
 * so every registered Subsystem must still outlive the manager. See the
 * declaration order below.
 *
 * Declare the mechanisms BEFORE the manager. Destruction runs in reverse, so
 * that order is what keeps them alive through the manager's destructor, which
 * calls end() on commands that write to them. Getting it backwards is only
 * visible at shutdown, and across two translation units it is not even ordered:
 * globals in different TUs are destroyed in an unspecified order, so keep the
 * manager and its mechanisms in the same TU.
 *
 * @code
 * // Same TU, mechanisms first: they outlive the manager that ends their
 * // commands.
 * Arm   arm{...};
 * Wings wings{...};
 *
 * mclib::mechanism::MechanismManager mechanisms;
 *
 * void initialize() {
 *   mechanisms.add(&arm, arm.makeStopCommand(), "arm");
 *   mechanisms.add(&wings, wings.makeRetractCommand(), "wings");
 *   mechanisms.registerAll();
 * }
 * @endcode
 */
class MechanismManager {
public:
  /**
   * @brief One managed mechanism: the subsystem, its owned default command,
   * and the bookkeeping the Subsystem base class has no room for.
   */
  struct Entry {
    Subsystem* subsystem = nullptr;
    std::unique_ptr<Command> default_command;
    std::string name;
    bool enabled = true;
    bool registered = false;
  };

  MechanismManager() = default;

  // Owns the commands the scheduler holds raw pointers to. Copying and moving
  // would both put those commands somewhere the scheduler cannot see, so
  // neither is allowed.
  MechanismManager(const MechanismManager&) = delete;
  MechanismManager& operator=(const MechanismManager&) = delete;
  MechanismManager(MechanismManager&&) = delete;
  MechanismManager& operator=(MechanismManager&&) = delete;

  /**
   * @brief End every owned command and drop the scheduler's references to it.
   *
   * The commands are about to be destroyed while CommandScheduler still holds
   * raw pointers to them, so a manager that went out of scope used to hand the
   * next CommandScheduler::run() a call into freed memory. Each entry is ended
   * and forgotten first, and the subsystem's registration is repointed at null,
   * so the subsystem itself keeps getting runPeriodic() with no command
   * attached.
   */
  ~MechanismManager();

  /**
   * @brief Take ownership of a mechanism's default command.
   *
   * @param subsystem The mechanism. Must not be null and must not already be
   *        held by this manager.
   * @param default_command The command the scheduler runs whenever nothing
   *        else requires the subsystem. Must not be null.
   * @param name Lookup name. Defaults to "mechanism<index>". Must be unique
   *        within this manager.
   * @return true if the entry was added. On false nothing is stored and
   *         @p default_command is destroyed.
   */
  bool add(Subsystem* subsystem,
           std::unique_ptr<Command> default_command,
           std::string name = {});

  /**
   * @brief Register every enabled, not-yet-registered entry with the
   * CommandScheduler.
   *
   * Safe to call more than once: entries already registered are skipped, so
   * the scheduler never sees a duplicate. (Its own duplicate check is an
   * `assert`, which does nothing in a release build.)
   *
   * An entry whose subsystem somebody else had already registered does NOT
   * count and is not marked registered. The scheduler keeps the first
   * registration, so the manager's call did nothing and claiming otherwise
   * would have the destructor tearing down a registration it does not own.
   *
   * @return How many entries this call actually registered.
   */
  std::size_t registerAll();

  /// @brief Number of held entries, registered or not.
  std::size_t size() const;

  /// @brief True when no entries are held.
  bool empty() const;

  /// @brief True when an entry with this name is held.
  bool contains(const std::string& name) const;

  /**
   * @brief The entry with this name, or nullptr. Entries are stored
   * indirectly, so the pointer stays valid across later add() calls and until
   * the manager is destroyed.
   */
  const Entry* getEntry(const std::string& name) const;

  /// @brief The entry at this index, or nullptr if the index is past the end.
  const Entry* getEntry(std::size_t index) const;

  /// @brief The named mechanism, or nullptr if there is no such entry.
  Subsystem* getSubsystem(const std::string& name) const;

  /**
   * @brief The named mechanism's default command, or nullptr if there is no
   * such entry. The manager keeps ownership.
   */
  Command* getDefaultCommand(const std::string& name) const;

  /// @brief True when the named entry has been handed to the scheduler.
  bool isRegistered(const std::string& name) const;

  /// @brief True when the named entry is enabled. False if there is no entry.
  bool isEnabled(const std::string& name) const;

  /**
   * @brief Enable or disable one entry.
   *
   * Disabling only stops registerAll() from registering the entry later. It
   * does not undo a registration that already happened, so disabling an
   * already-registered entry changes nothing but the record and describe().
   * Call CommandScheduler::unregisterSubsystem yourself if you mean to drop it.
   *
   * @return false if there is no entry with that name.
   */
  bool setEnabled(const std::string& name, bool enabled);

  /// @brief Apply setEnabled() to every entry.
  void setAllEnabled(bool enabled);

  /// @brief Names of every entry, in insertion order.
  std::vector<std::string> getNames() const;

  /**
   * @brief One line of telemetry per entry: name, enabled, registered, and
   * which command currently holds the scheduler's requirement on that
   * subsystem (`default`, `other`, or `none`).
   */
  std::string describe() const;

private:
  Entry* findEntry(const std::string& name);
  const Entry* findEntry(const std::string& name) const;

  std::vector<std::unique_ptr<Entry>> m_entries;
};

}  // namespace mechanism
}  // namespace mclib
