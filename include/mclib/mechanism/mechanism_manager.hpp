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
 * Because the manager owns the commands the scheduler points at, it must
 * outlive the scheduler's use of them. Give it static or program-long storage.
 *
 * @code
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

  // Owns the commands the scheduler holds raw pointers to, and the scheduler
  // has no unregister call. Copying and moving would both put those commands
  // somewhere the scheduler cannot see, so neither is allowed.
  MechanismManager(const MechanismManager&) = delete;
  MechanismManager& operator=(const MechanismManager&) = delete;
  MechanismManager(MechanismManager&&) = delete;
  MechanismManager& operator=(MechanismManager&&) = delete;

  ~MechanismManager() = default;

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
   * @return How many entries were registered by this call.
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
   * Disabling only stops registerAll() from registering the entry later. The
   * scheduler has no way to drop a subsystem once registered, so disabling an
   * already-registered entry changes nothing but the record and describe().
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
