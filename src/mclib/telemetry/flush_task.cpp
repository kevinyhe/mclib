// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/telemetry/flush_task.hpp"

#include "pros/rtos.h"
#include "pros/rtos.hpp"

#include <cstdint>
#include <memory>

namespace mclib {
namespace telemetry {

static_assert(kFlushTaskPriority == TASK_PRIORITY_DEFAULT - 1,
              "kFlushTaskPriority must stay one step below the PROS default");

namespace {

/**
 * @brief Keeps the pros::Task alive without putting PROS in the header.
 *
 * @details FlushTask::m_task is a void*, so the header never sees pros::Task.
 * This is the whole reason telemetry stays host-testable.
 */
struct TaskHolder {
  std::unique_ptr<pros::Task> task;
};

}  // namespace

FlushTask::FlushTask(Logger& logger, units::QTime period)
    : m_logger(&logger) {
  const double ms = period.ms();
  m_period_ms = ms >= 1.0 ? static_cast<std::uint32_t>(ms) : 1u;

  TaskHolder* holder = new TaskHolder();
  m_task = holder;
  holder->task = std::make_unique<pros::Task>(
      [this]() { run(); }, kFlushTaskPriority, TASK_STACK_DEPTH_DEFAULT, "mclib telemetry");
}

FlushTask::~FlushTask() {
  stop();
  delete static_cast<TaskHolder*>(m_task);
  m_task = nullptr;
}

void FlushTask::run() {
  std::uint32_t next = pros::millis();
  while (!m_stop_requested.load(std::memory_order_relaxed)) {
    const std::size_t written = m_logger->flush();
    if (written != 0) {
      m_rows_written.fetch_add(written, std::memory_order_relaxed);
    }
    pros::Task::delay_until(&next, m_period_ms);
  }
}

void FlushTask::stop() {
  if (m_task == nullptr || m_stop_requested.load(std::memory_order_relaxed)) {
    return;
  }
  m_stop_requested.store(true, std::memory_order_relaxed);
  TaskHolder* holder = static_cast<TaskHolder*>(m_task);
  if (holder->task != nullptr) {
    holder->task->join();
    holder->task.reset();
  }
  // Drain whatever the producer committed after the last pass and write the
  // trailer. Done on the caller's thread, with the task already joined, so
  // there is no second consumer racing us.
  m_logger->close();
}

}  // namespace telemetry
}  // namespace mclib
