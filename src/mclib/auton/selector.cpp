// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/auton/selector.hpp"

#include "pros/colors.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

namespace mclib {
namespace auton {

namespace {

constexpr pros::Color kBackground = pros::Color::black;
constexpr pros::Color kButton = pros::Color::dim_gray;
constexpr pros::Color kSelectedButton = pros::Color::dodger_blue;
constexpr pros::Color kBorder = pros::Color::white;
constexpr pros::Color kText = pros::Color::white;

/** @brief Height of one E_TEXT_MEDIUM line, used to centre names vertically. */
constexpr int kMediumTextHeight = 20;

/**
 * @brief Characters on one controller screen line.
 *
 * @details The controller shows about 15. Writing a fixed-width line, padded
 * with spaces, wipes the tail of a longer previous name instead of leaving it
 * on screen after the pick changes to a shorter one.
 */
constexpr std::size_t kControllerLineWidth = 15;

std::int16_t clip16(int value) {
  return static_cast<std::int16_t>(value);
}

}  // namespace

AutonSelector::~AutonSelector() { stopPolling(); }

int AutonSelector::add(std::string name, std::function<void()> run) {
  const int index = m_model.add(std::move(name), std::move(run));
  markTextDirty();
  return index;
}

void AutonSelector::select(int index) {
  m_model.select(index);
  markTextDirty();
}

void AutonSelector::next() {
  m_model.next();
  markTextDirty();
}

void AutonSelector::previous() {
  m_model.previous();
  markTextDirty();
}

void AutonSelector::drawBrainScreen() {
  pros::screen::set_eraser(kBackground);
  pros::screen::erase();

  if (m_model.size() == 0) {
    pros::screen::set_pen(kText);
    pros::screen::print(pros::E_TEXT_MEDIUM, clip16(m_layout.padding),
                        clip16(m_layout.top + m_layout.padding), "No autonomous routines");
    return;
  }

  const int selected = m_model.selected();
  for (std::size_t i = 0; i < m_model.size(); ++i) {
    const SelectorModel::ButtonRect rect = m_model.buttonRect(static_cast<int>(i), m_layout);
    if (rect.w <= 0 || rect.h <= 0) {
      continue;
    }
    const std::int16_t x0 = clip16(rect.x);
    const std::int16_t y0 = clip16(rect.y);
    const std::int16_t x1 = clip16(rect.x + rect.w - 1);
    const std::int16_t y1 = clip16(rect.y + rect.h - 1);

    pros::screen::set_pen(static_cast<int>(i) == selected ? kSelectedButton : kButton);
    pros::screen::fill_rect(x0, y0, x1, y1);
    pros::screen::set_pen(kBorder);
    pros::screen::draw_rect(x0, y0, x1, y1);

    pros::screen::set_pen(kText);
    const int text_y = rect.y + (rect.h - kMediumTextHeight) / 2;
    pros::screen::print(pros::E_TEXT_MEDIUM, clip16(rect.x + m_layout.padding), clip16(text_y),
                        "%s", m_model.entries()[i].name.c_str());
  }
}

void AutonSelector::pollBrainTouch() {
  const pros::screen_touch_status_s_t status = pros::screen::touch_status();
  const bool down =
      status.touch_status == pros::E_TOUCH_PRESSED || status.touch_status == pros::E_TOUCH_HELD;
  const bool new_press = down && !m_touch_was_down;
  m_touch_was_down = down;
  if (!new_press) {
    return;
  }
  const int hit = m_model.hitTest(status.x, status.y, m_layout);
  if (hit >= 0) {
    select(hit);
  }
  // Redraw even on a miss: a touch is the user asking to see the screen.
  drawBrainScreen();
}

void AutonSelector::bindController(device::Controller& controller,
                                   device::DigitalButton previous_button,
                                   device::DigitalButton next_button) {
  m_controller = &controller;
  m_previous_button = previous_button;
  m_next_button = next_button;
  // Read the buttons once so a button already held at bind time does not
  // count as a press on the first poll.
  m_previous_was_down = controller.getDigital(previous_button);
  m_next_was_down = controller.getDigital(next_button);
  markTextDirty();
}

void AutonSelector::refreshControllerText(bool force) {
  if (m_controller == nullptr || (!m_text_dirty && !force)) {
    return;
  }
  const std::uint32_t now = pros::millis();
  if (!force && now - m_last_text_ms < kControllerTextIntervalMs) {
    return;
  }
  char line[kControllerLineWidth + 1];
  std::snprintf(line, sizeof(line), "%-*.*s", static_cast<int>(kControllerLineWidth),
                static_cast<int>(kControllerLineWidth),
                m_model.size() == 0 ? "no autons" : m_model.selectedName().c_str());
  m_controller->setText(0, 0, line);
  m_last_text_ms = now;
  m_text_dirty = false;
}

void AutonSelector::pollController() {
  if (m_controller == nullptr) {
    return;
  }
  const bool previous_down = m_controller->getDigital(m_previous_button);
  const bool next_down = m_controller->getDigital(m_next_button);
  const bool previous_pressed = previous_down && !m_previous_was_down;
  const bool next_pressed = next_down && !m_next_was_down;
  m_previous_was_down = previous_down;
  m_next_was_down = next_down;

  if (previous_pressed) {
    previous();
  }
  if (next_pressed) {
    next();
  }
  if (previous_pressed || next_pressed) {
    drawBrainScreen();
  }
  refreshControllerText(false);
}

void AutonSelector::poll() {
  pollBrainTouch();
  pollController();
}

bool AutonSelector::saveSelection(const char* path) const {
  if (path == nullptr || m_model.selected() < 0) {
    return false;
  }
  std::FILE* file = std::fopen(path, "w");
  if (file == nullptr) {
    return false;
  }
  const bool ok = std::fputs(m_model.selectedName().c_str(), file) >= 0 && std::fputc('\n', file) != EOF;
  std::fclose(file);
  return ok;
}

bool AutonSelector::loadSelection(const char* path) {
  if (path == nullptr) {
    return false;
  }
  std::FILE* file = std::fopen(path, "r");
  if (file == nullptr) {
    return false;
  }
  char line[128] = {};
  const bool read = std::fgets(line, sizeof(line), file) != nullptr;
  std::fclose(file);
  if (!read) {
    return false;
  }
  // Strip the newline and any carriage return a hand-edited file may carry.
  std::size_t length = std::strlen(line);
  while (length > 0 && (line[length - 1] == '\n' || line[length - 1] == '\r')) {
    line[--length] = '\0';
  }
  if (length == 0) {
    return false;
  }
  const int index = m_model.indexOf(std::string(line, length));
  if (index < 0) {
    return false;
  }
  select(index);
  return true;
}

void AutonSelector::runPollLoop(std::uint32_t period_ms) {
  drawBrainScreen();
  refreshControllerText(true);
  std::uint32_t now = pros::millis();
  while (m_should_run.load()) {
    if (pros::competition::is_autonomous()) {
      // Hand the screen and controller over to the routine. Clearing the flag
      // here, not in stopPolling(), is what lets startPolling() run again.
      m_should_run.store(false);
      break;
    }
    poll();
    pros::Task::delay_until(&now, period_ms);
  }
  // One last draw so the screen shows the pick that is about to run, and the
  // controller line is not left behind a rate-limited update.
  drawBrainScreen();
  refreshControllerText(true);
}

bool AutonSelector::startPolling(units::QTime period) {
  // Claim the flag and check it in one step, so two callers racing to start
  // cannot both create a task and leak the first one.
  bool expected = false;
  if (!m_should_run.compare_exchange_strong(expected, true)) {
    return false;
  }
  // A task that stopped itself on autonomous is finished but not yet joined.
  if (m_task != nullptr) {
    m_task->join();
    m_task.reset();
  }
  const double ms = period.ms();
  const std::uint32_t period_ms = ms >= 1.0 ? static_cast<std::uint32_t>(ms) : 1u;
  m_task = std::make_unique<pros::Task>([this, period_ms]() { runPollLoop(period_ms); },
                                        "mclib auton selector");
  return true;
}

void AutonSelector::stopPolling() {
  m_should_run.store(false);
  if (m_task != nullptr) {
    m_task->join();
    m_task.reset();
  }
}

}  // namespace auton
}  // namespace mclib
