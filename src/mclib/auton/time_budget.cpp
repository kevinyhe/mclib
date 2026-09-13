// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/auton/time_budget.hpp"

namespace mclib {
namespace auton {

namespace {
constexpr QTime kZero{};

/// @brief @p value, or zero when it is negative.
QTime atLeastZero(QTime value) {
  return value > kZero ? value : kZero;
}
}  // namespace

void TimeBudget::configure(const TimeBudgetConfig& config) {
  m_config = config;
}

void TimeBudget::start(QTime start_time) {
  m_start = start_time;
  m_started = true;
  m_in_grace = false;
}

void TimeBudget::reset() {
  m_start = kZero;
  m_started = false;
  m_in_grace = false;
}

QTime TimeBudget::graceReserve() const {
  if (m_config.policy != DeadlinePolicy::FinishMustRun) {
    return kZero;
  }
  const QTime grace = atLeastZero(m_config.grace);
  return grace < m_config.total ? grace : m_config.total;
}

QTime TimeBudget::driveAllowance() const {
  return atLeastZero(m_config.total - graceReserve());
}

QTime TimeBudget::elapsedAt(QTime instant) const {
  if (!m_started) {
    return kZero;
  }
  return atLeastZero(instant - m_start);
}

QTime TimeBudget::remainingAt(QTime instant) const {
  if (!active()) {
    return kZero;
  }

  const QTime allowance = m_in_grace ? m_config.total : driveAllowance();

  if (!m_started) {
    return allowance;
  }
  return atLeastZero(allowance - elapsedAt(instant));
}

bool TimeBudget::expiredAt(QTime instant) const {
  if (!active() || !m_started || m_in_grace) {
    return false;
  }
  return elapsedAt(instant) >= driveAllowance();
}

bool TimeBudget::graceExpiredAt(QTime instant) const {
  if (!m_in_grace || !m_started) {
    return false;
  }
  return elapsedAt(instant) >= m_config.total;
}

QTime clampStepTimeout(QTime requested, QTime remaining, QTime floor) {
  const QTime least = atLeastZero(floor);

  if (remaining <= least) {
    return least;
  }
  if (requested <= kZero || requested > remaining) {
    return remaining;
  }
  return requested > least ? requested : least;
}

QTime stepTimeoutFor(QTime requested, const TimeBudget& budget) {
  if (!budget.active()) {
    return requested;
  }
  return clampStepTimeout(requested, budget.remaining());
}

std::size_t nextStepAfterDeadline(const std::vector<bool>& must_run,
                                  std::size_t from,
                                  DeadlinePolicy policy) {
  const std::size_t count = must_run.size();

  if (policy != DeadlinePolicy::FinishMustRun) {
    return count;
  }

  for (std::size_t index = from; index < count; ++index) {
    if (must_run[index]) {
      return index;
    }
  }

  return count;
}

}  // namespace auton
}  // namespace mclib
