// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/async_motion.hpp"

#include "api.h"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/math.hpp"
#include "mclib/sync.hpp"

#include <atomic>
#include <cstdint>
#include <utility>

#if defined(MCLIB_HOST_BUILD)
#include <thread>
#endif

namespace mclib {
namespace control {

namespace detail {
struct AsyncMotionState {
  std::atomic_bool done{false};
  std::atomic<int> result{static_cast<int>(MotionResult::Cancelled)};
  std::uint32_t start_ms = 0;
  Pose2D start_pose{};
  // start() and a handle can both try to reap the same motion from
  // different tasks.
  sync::Mutex reap_mutex;
#if defined(MCLIB_HOST_BUILD)
  std::thread thread;
#else
  std::unique_ptr<pros::Task> task;
#endif
};
}  // namespace detail

namespace {

using detail::AsyncMotionState;

/// @brief How long cancel() waits for the motion to notice the flag. Every
///        loop in motion.cpp checks it at a 10 ms boundary, so this is the
///        ceiling, not the expected cost. Same as ChassisController's.
constexpr std::uint32_t kCancelJoinTimeoutMs = 500;
constexpr std::uint32_t kPollMs = 10;

// The motion started last, so start() can cancel it. Guarded because start()
// and a handle's destructor can run on different tasks.
sync::Mutex g_current_mutex;
std::weak_ptr<AsyncMotionState> g_current;

MotionResult resultOf(const AsyncMotionState& state) {
  return static_cast<MotionResult>(state.result.load());
}

// Called once the motion has stopped. On the host the thread must be joined
// before the state can be destroyed.
void reap(AsyncMotionState& state) {
  sync::LockGuard lock(state.reap_mutex);
#if defined(MCLIB_HOST_BUILD)
  if (state.thread.joinable()) state.thread.join();
#else
  state.task.reset();
#endif
}

void waitDone(AsyncMotionState& state) {
  while (!state.done.load()) pros::delay(kPollMs);
  reap(state);
}

MotionResult cancelState(AsyncMotionState& state) {
  if (!state.done.load()) {
    requestCancel(CancelToken::Motion);
#if defined(MCLIB_HOST_BUILD)
    // A thread cannot be killed, so wait for it however long it takes.
    while (!state.done.load()) pros::delay(2);
#else
    const std::uint32_t start = pros::millis();
    while (!state.done.load() &&
           static_cast<std::uint32_t>(pros::millis() - start) < kCancelJoinTimeoutMs) {
      pros::delay(2);
    }
    if (!state.done.load()) {
      // The motion ignored the flag. Nothing after remove() may take an
      // mclib lock; see AsyncControlCommand::stopRunningTask().
      state.task->remove();
      stopChassis(device::BrakeMode::Hold);
      state.result.store(static_cast<int>(MotionResult::Cancelled));
      state.done.store(true);
    }
#endif
    clearCancel(CancelToken::Motion);
  }
  if (state.done.load()) reap(state);
  return resultOf(state);
}

}  // namespace

AsyncMotion::AsyncMotion(AsyncMotion&& other) noexcept
    : m_state(std::move(other.m_state)) {}

AsyncMotion& AsyncMotion::operator=(AsyncMotion&& other) noexcept {
  if (this != &other) {
    if (m_state) cancelState(*m_state);
    m_state = std::move(other.m_state);
  }
  return *this;
}

AsyncMotion::~AsyncMotion() {
  if (m_state) cancelState(*m_state);
}

AsyncMotion AsyncMotion::start(std::function<MotionResult()> motion) {
  {
    std::shared_ptr<AsyncMotionState> previous;
    {
      sync::LockGuard lock(g_current_mutex);
      previous = g_current.lock();
    }
    if (previous) cancelState(*previous);
  }

  auto state = std::make_shared<AsyncMotionState>();
  state->start_ms = pros::millis();
  state->start_pose = robotState().pose();
  clearCancel(CancelToken::Motion);

  // The task holds its own reference, so the state outlives a handle that is
  // destroyed while the motion runs.
  auto body = [state, motion = std::move(motion)]() {
    const MotionResult result = motion ? motion() : MotionResult::InvalidValue;
    state->result.store(static_cast<int>(result));
    state->done.store(true);
  };
#if defined(MCLIB_HOST_BUILD)
  state->thread = std::thread(std::move(body));
#else
  state->task = std::make_unique<pros::Task>(std::move(body), "mclib motion");
#endif

  {
    sync::LockGuard lock(g_current_mutex);
    g_current = state;
  }
  return AsyncMotion(std::move(state));
}

bool AsyncMotion::isRunning() const {
  return m_state && !m_state->done.load();
}

std::optional<MotionResult> AsyncMotion::result() const {
  if (!m_state || !m_state->done.load()) return std::nullopt;
  return resultOf(*m_state);
}

MotionResult AsyncMotion::wait() {
  if (!m_state) return MotionResult::NoDrive;
  waitDone(*m_state);
  return resultOf(*m_state);
}

bool AsyncMotion::waitUntil(const std::function<bool()>& condition) {
  if (!m_state || !condition) return false;
  while (!m_state->done.load()) {
    if (condition()) return true;
    pros::delay(kPollMs);
  }
  return false;
}

bool AsyncMotion::waitUntilElapsed(QTime time) {
  if (!m_state) return false;
  const std::uint32_t start = m_state->start_ms;
  const double limit = time.ms();
  return waitUntil([start, limit] {
    return static_cast<double>(static_cast<std::uint32_t>(pros::millis() - start)) >= limit;
  });
}

bool AsyncMotion::waitUntilTravelled(QLength distance) {
  if (!m_state) return false;
  const Pose2D start = m_state->start_pose;
  const double limit = distance.in();
  return waitUntil([start, limit] {
    return robotState().pose().distanceTo(start) >= limit;
  });
}

MotionResult AsyncMotion::cancel() {
  if (!m_state) return MotionResult::NoDrive;
  return cancelState(*m_state);
}

}  // namespace control
}  // namespace mclib
