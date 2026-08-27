// mclib
#include "mclib/mechanism/mechanism_manager.hpp"

#include "mclib/command/commandScheduler.h"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>

namespace mclib {
namespace mechanism {

bool MechanismManager::add(Subsystem* subsystem,
                           std::unique_ptr<Command> default_command,
                           std::string name) {
  // The scheduler only asserts on these, and asserts are gone in release.
  if (subsystem == nullptr || default_command == nullptr) {
    return false;
  }

  for (const std::unique_ptr<Entry>& entry : m_entries) {
    if (entry->subsystem == subsystem) {
      return false;
    }
  }

  if (name.empty()) {
    std::size_t suffix = m_entries.size();
    do {
      name = "mechanism" + std::to_string(suffix);
      ++suffix;
    } while (contains(name));
  } else if (contains(name)) {
    return false;
  }

  auto entry = std::make_unique<Entry>();
  entry->subsystem = subsystem;
  entry->default_command = std::move(default_command);
  entry->name = std::move(name);
  entry->enabled = true;
  entry->registered = false;

  m_entries.push_back(std::move(entry));
  return true;
}

std::size_t MechanismManager::registerAll() {
  std::size_t registered_count = 0;

  for (const std::unique_ptr<Entry>& entry : m_entries) {
    if (entry->registered || !entry->enabled) {
      continue;
    }

    CommandScheduler::registerSubsystem(entry->subsystem,
                                        entry->default_command.get());
    entry->registered = true;
    ++registered_count;
  }

  return registered_count;
}

std::size_t MechanismManager::size() const {
  return m_entries.size();
}

bool MechanismManager::empty() const {
  return m_entries.empty();
}

bool MechanismManager::contains(const std::string& name) const {
  return findEntry(name) != nullptr;
}

const MechanismManager::Entry* MechanismManager::getEntry(
    const std::string& name) const {
  return findEntry(name);
}

const MechanismManager::Entry* MechanismManager::getEntry(
    std::size_t index) const {
  if (index >= m_entries.size()) {
    return nullptr;
  }

  return m_entries[index].get();
}

Subsystem* MechanismManager::getSubsystem(const std::string& name) const {
  const Entry* entry = findEntry(name);
  return entry == nullptr ? nullptr : entry->subsystem;
}

Command* MechanismManager::getDefaultCommand(const std::string& name) const {
  const Entry* entry = findEntry(name);
  return entry == nullptr ? nullptr : entry->default_command.get();
}

bool MechanismManager::isRegistered(const std::string& name) const {
  const Entry* entry = findEntry(name);
  return entry != nullptr && entry->registered;
}

bool MechanismManager::isEnabled(const std::string& name) const {
  const Entry* entry = findEntry(name);
  return entry != nullptr && entry->enabled;
}

bool MechanismManager::setEnabled(const std::string& name, bool enabled) {
  Entry* entry = findEntry(name);

  if (entry == nullptr) {
    return false;
  }

  entry->enabled = enabled;
  return true;
}

void MechanismManager::setAllEnabled(bool enabled) {
  for (const std::unique_ptr<Entry>& entry : m_entries) {
    entry->enabled = enabled;
  }
}

std::vector<std::string> MechanismManager::getNames() const {
  std::vector<std::string> names;
  names.reserve(m_entries.size());

  for (const std::unique_ptr<Entry>& entry : m_entries) {
    names.push_back(entry->name);
  }

  return names;
}

std::string MechanismManager::describe() const {
  std::string description;

  for (const std::unique_ptr<Entry>& entry : m_entries) {
    const char* requirement = "none";

    if (entry->registered) {
      std::optional<Command*> requiring =
          CommandScheduler::getRequiring(entry->subsystem);

      if (requiring.has_value()) {
        requirement =
            *requiring == entry->default_command.get() ? "default" : "other";
      }
    }

    description += entry->name;
    description += " enabled=";
    description += entry->enabled ? "yes" : "no";
    description += " registered=";
    description += entry->registered ? "yes" : "no";
    description += " requirement=";
    description += requirement;
    description += "\n";
  }

  return description;
}

MechanismManager::Entry* MechanismManager::findEntry(const std::string& name) {
  auto it = std::find_if(m_entries.begin(), m_entries.end(),
                         [&name](const std::unique_ptr<Entry>& entry) {
                           return entry->name == name;
                         });

  return it == m_entries.end() ? nullptr : it->get();
}

const MechanismManager::Entry* MechanismManager::findEntry(
    const std::string& name) const {
  auto it = std::find_if(m_entries.begin(), m_entries.end(),
                         [&name](const std::unique_ptr<Entry>& entry) {
                           return entry->name == name;
                         });

  return it == m_entries.end() ? nullptr : it->get();
}

}  // namespace mechanism
}  // namespace mclib
