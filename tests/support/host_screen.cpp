// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host stand-ins for the PROS calls src/mclib/auton/selector.cpp makes: the
// brain screen, pros::Controller, pros::millis() and pros::Task. On the robot
// these come from libpros. Each one here records what it was asked to do in
// host_screen.hpp, so a test can check what the selector drew and wrote.
//
// pros::Task runs its function to completion inside the constructor. The
// selector's poll loop only returns once the field reports autonomous, so a
// test drives it through on_delay, which runs after every delay_until().
//
// Only auton_selector_test links this (see the Makefile). pros::millis()
// would clash with the one motion_safety_test defines.

#include "support/host_screen.hpp"

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <cstdarg>
#include <cstdio>

namespace mclib {
namespace test {
namespace host_screen {

int erase_count = 0;
std::vector<FilledRect> filled;
std::vector<std::string> printed;
bool touch_down = false;
int touch_x = 0;
int touch_y = 0;
std::map<int, bool> button_down;
std::vector<std::string> controller_text;
std::uint32_t now_ms = 0;
std::function<void()> on_delay;
int tasks_started = 0;

namespace {
std::uint32_t g_pen = 0;
}

void clearDrawing() {
  erase_count = 0;
  filled.clear();
  printed.clear();
}

void reset() {
  clearDrawing();
  touch_down = false;
  touch_x = 0;
  touch_y = 0;
  button_down.clear();
  controller_text.clear();
  now_ms = 0;
  on_delay = nullptr;
  tasks_started = 0;
  g_pen = 0;
}

}  // namespace host_screen
}  // namespace test
}  // namespace mclib

namespace hs = mclib::test::host_screen;

namespace pros {

namespace screen {

std::uint32_t set_pen(pros::Color color) {
  hs::g_pen = static_cast<std::uint32_t>(color);
  return 1;
}
std::uint32_t set_eraser(pros::Color) { return 1; }
std::uint32_t erase() {
  ++hs::erase_count;
  return 1;
}
std::uint32_t fill_rect(const std::int16_t x0, const std::int16_t y0, const std::int16_t x1,
                        const std::int16_t y1) {
  hs::filled.push_back({hs::g_pen, x0, y0, x1, y1});
  return 1;
}
std::uint32_t draw_rect(const std::int16_t, const std::int16_t, const std::int16_t,
                        const std::int16_t) {
  return 1;
}
screen_touch_status_s_t touch_status() {
  screen_touch_status_s_t status{};
  status.touch_status = hs::touch_down ? E_TOUCH_PRESSED : E_TOUCH_RELEASED;
  status.x = static_cast<std::int16_t>(hs::touch_x);
  status.y = static_cast<std::int16_t>(hs::touch_y);
  return status;
}

}  // namespace screen

namespace c {
extern "C" std::uint32_t screen_print_at(text_format_e_t, const std::int16_t, const std::int16_t,
                                         const char* text, ...) {
  char buf[256];
  va_list args;
  va_start(args, text);
  std::vsnprintf(buf, sizeof(buf), text, args);
  va_end(args);
  hs::printed.emplace_back(buf);
  return 1;
}
}  // namespace c

extern "C" std::uint32_t millis() { return hs::now_ms; }

Controller::Controller(controller_id_e_t id) : _id(id) {}

std::int32_t Controller::get_digital(controller_digital_e_t button) {
  const auto found = hs::button_down.find(static_cast<int>(button));
  return found != hs::button_down.end() && found->second ? 1 : 0;
}

std::int32_t Controller::get_analog(controller_analog_e_t) { return 0; }

std::int32_t Controller::set_text(std::uint8_t, std::uint8_t, const char* str) {
  hs::controller_text.emplace_back(str);
  return 1;
}

Task::Task(task_fn_t function, void* parameters, std::uint32_t, std::uint16_t, const char*) {
  ++hs::tasks_started;
  function(parameters);
}

void Task::join() {}

void Task::delay_until(std::uint32_t* const prev_time, const std::uint32_t delta) {
  *prev_time += delta;
  hs::now_ms = *prev_time;
  if (hs::on_delay) {
    hs::on_delay();
  }
}

}  // namespace pros
