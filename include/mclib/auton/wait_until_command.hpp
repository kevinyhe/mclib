// mclib
#pragma once

#include "mclib/command/command.h"
#include "mclib/time.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <utility>

namespace mclib {
namespace auton {

/**
 * @brief Waits for a predicate, but gives up after a timeout.
 *
 * `WaitUntilCommand` waits forever. In an autonomous that is a hang: a limit
 * switch that never closes because the intake jammed, or a sensor read that
 * stays false because the cable came out, stops the whole routine with the
 * drive wherever it was and every later step unreached. This one ends anyway
 * when the timeout passes, and says which of the two happened.
 *
 * A timeout that is not positive means "wait forever", the old behaviour,
 * which a caller now has to ask for on purpose.
 */
class WaitUntilTimeoutCommand : public Command {
public:
  WaitUntilTimeoutCommand(std::function<bool()> condition, QTime timeout)
      : m_condition(std::move(condition)), m_timeout(timeout) {}

  void initialize() override {
    m_start = time::now();
    m_timed_out = false;
  }

  bool isFinished() override {
    if (m_condition && m_condition()) {
      return true;
    }
    if (m_timeout <= QTime{}) {
      return false;
    }
    if (time::now() - m_start >= m_timeout) {
      m_timed_out = true;
      return true;
    }
    return false;
  }

  /// @brief True when the command ended on the timeout, not the predicate.
  bool timedOut() const { return m_timed_out; }

  /// @brief The timeout this command was built with.
  QTime timeout() const { return m_timeout; }

  ~WaitUntilTimeoutCommand() override = default;

private:
  std::function<bool()> m_condition;
  QTime m_timeout{};
  QTime m_start{};
  bool m_timed_out = false;
};

}  // namespace auton
}  // namespace mclib
