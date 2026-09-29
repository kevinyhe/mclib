// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/motion.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <memory>
#include <optional>

/**
 * @file async_motion.hpp
 * @brief Run a blocking motion on its own task and keep going.
 *
 * Every function in `motion.hpp` blocks until it finishes. `AsyncMotion`
 * runs one on a separate task so the autonomous can do something partway
 * through:
 *
 * @code
 * using mclib::control::AsyncMotion;
 *
 * AsyncMotion drive = AsyncMotion::start([] { return driveTo(30_in, 2_s); });
 * drive.waitUntilTravelled(12_in);
 * intake.setVoltage(12_V);
 * if (drive.wait() != MotionResult::Reached) return;
 * @endcode
 *
 * ## Rules
 *
 * - One motion at a time. Starting a new `AsyncMotion` cancels the one that
 *   is running first, and waits for it to stop.
 * - Cancelling is cooperative, as in `ChassisController`'s commands: it sets
 *   `CancelToken::Motion`, and the motion's loop stops at its next 10 ms
 *   tick and brakes the drive. Only a motion that ignores the flag for
 *   500 ms has its task killed.
 * - Destroying a handle while its motion runs cancels it. Keep the handle
 *   for as long as the motion should run.
 * - Do not also call blocking motions or schedule chassis commands while an
 *   `AsyncMotion` runs. They share the drive and the cancel flag.
 */

namespace mclib {
namespace control {

namespace detail {
struct AsyncMotionState;
}  // namespace detail

/**
 * @brief Handle to a motion running on its own task.
 *
 * Move-only. A default-constructed handle has no motion: `valid()` is false,
 * `wait()` and `cancel()` return `MotionResult::NoDrive` at once.
 */
class AsyncMotion {
 public:
  AsyncMotion() = default;
  AsyncMotion(AsyncMotion&& other) noexcept;
  AsyncMotion& operator=(AsyncMotion&& other) noexcept;
  AsyncMotion(const AsyncMotion&) = delete;
  AsyncMotion& operator=(const AsyncMotion&) = delete;
  /// @brief Cancels the motion if it is still running.
  ~AsyncMotion();

  /**
   * @brief Start @p motion on its own task and return at once.
   *
   * Cancels and waits for any `AsyncMotion` already running.
   *
   * @param motion A call to one of the `motion.hpp` routines, e.g.
   *        `[] { return driveTo(24_in, 2_s); }`.
   */
  static AsyncMotion start(std::function<MotionResult()> motion);

  /// @brief True when this handle was returned by start().
  bool valid() const { return m_state != nullptr; }
  /// @brief True while the motion is still running.
  bool isRunning() const;
  /// @brief The result, once the motion has finished.
  std::optional<MotionResult> result() const;

  /// @brief Block until the motion finishes and return how it ended.
  MotionResult wait();

  /**
   * @brief Block until @p time has passed since start(), or the motion ends.
   * @return True if the time passed while the motion was still running.
   */
  bool waitUntilElapsed(QTime time);

  /**
   * @brief Block until the robot is @p distance from where it was at
   *        start(), or the motion ends.
   *
   * Straight-line distance on the odometry pose, not distance along a
   * curve. For `driveTo()` the two are the same.
   *
   * @return True if the distance was reached while the motion was running.
   */
  bool waitUntilTravelled(QLength distance);

  /**
   * @brief Block until @p condition returns true, or the motion ends.
   *
   * Checked every 10 ms on the calling task.
   *
   * @return True if the condition became true while the motion was running.
   */
  bool waitUntil(const std::function<bool()>& condition);

  /**
   * @brief Stop the motion and wait for it.
   * @return How it ended. `Cancelled`, or its own result if it had already
   *         finished.
   */
  MotionResult cancel();

 private:
  explicit AsyncMotion(std::shared_ptr<detail::AsyncMotionState> state)
      : m_state(std::move(state)) {}

  std::shared_ptr<detail::AsyncMotionState> m_state;
};

}  // namespace control
}  // namespace mclib
