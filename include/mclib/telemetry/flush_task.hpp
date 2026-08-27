// mclib
#pragma once

/**
 * @brief Drains a Logger from a low-priority background task.
 *
 * @details The control loop runs at 10 ms and an SD write is slow and jittery.
 * Blocking a control tick on file I/O would wreck the loop the log exists to
 * measure, so the producer only ever copies a Row into a ring buffer and this
 * task does the writing, below the control loop's priority.
 *
 * This is the only telemetry header whose implementation touches PROS, and
 * src/mclib/telemetry/flush_task.cpp is the only translation unit in this
 * subsystem that includes a PROS header.
 */

#include "mclib/telemetry/logger.hpp"
#include "mclib/units/units.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace mclib {
namespace telemetry {

/**
 * @brief Priority for the flush task: one step below the PROS default.
 *
 * @details Spelled as a literal rather than TASK_PRIORITY_DEFAULT so this
 * header stays PROS-free. flush_task.cpp static_asserts the two agree.
 */
inline constexpr std::uint32_t kFlushTaskPriority = 8 - 1;

/**
 * @brief Runs Logger::flush() on a timer until stopped.
 *
 * @details Construction starts the task; destruction stops it and joins, then
 * closes the logger so the trailer lands. Keep the object alive as long as the
 * logger is - a static or a member of the subsystem that owns the logger.
 */
class FlushTask {
 public:
  /**
   * @brief Start draining @p logger every @p period.
   *
   * @param logger The logger to drain. Must outlive this task.
   * @param period How often to flush. 100 ms batches roughly five rows of a
   *   20 ms log per write, which is far cheaper than one write per row.
   */
  explicit FlushTask(Logger& logger, units::QTime period = 100.0 * units::millisecond);

  ~FlushTask();

  FlushTask(const FlushTask&) = delete;
  FlushTask& operator=(const FlushTask&) = delete;

  /**
   * @brief Ask the task to finish, wait for it, and close the logger.
   *
   * @details Idempotent. Called by the destructor.
   */
  void stop();

  /** @brief Rows this task has written since it started. */
  std::size_t rowsWritten() const { return m_rows_written.load(std::memory_order_relaxed); }

 private:
  void run();

  Logger* m_logger;
  std::uint32_t m_period_ms;
  // Written by the owning thread, read by the flush task, and the other way
  // round for the counter. Atomic so neither read is torn or stale.
  std::atomic<bool> m_stop_requested{false};
  std::atomic<std::size_t> m_rows_written{0};
  void* m_task = nullptr;
};

}  // namespace telemetry
}  // namespace mclib
