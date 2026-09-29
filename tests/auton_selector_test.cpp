// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// AutonSelector, the brain-screen and controller wrapper around
// SelectorModel. selector_model_test covers the list logic; this covers what
// the wrapper adds: drawing, touch and button handling, the controller line,
// the polling task, and saving the pick to a file.
//
// The PROS screen, clock and task calls are the stand-ins in
// tests/support/host_screen.cpp. The controller is the shared stand-in in
// tests/support/host_devices.cpp.

#include "mclib/auton/selector.hpp"
#include "mclib/device/controller.hpp"
#include "pros/colors.hpp"
#include "pros/misc.h"
#include "support/host_devices.hpp"
#include "support/host_pros.hpp"
#include "support/host_screen.hpp"
#include "test_assert.hpp"

#include <cstdio>
#include <filesystem>
#include <string>
#include <unistd.h>

namespace {

namespace hd = mclib::test::host_devices;
namespace hs = mclib::test::host_screen;
using mclib::auton::AutonSelector;
using mclib::auton::SelectorModel;
using mclib::device::Controller;
using mclib::device::DigitalButton;

constexpr std::uint32_t kSelectedPen = static_cast<std::uint32_t>(pros::Color::dodger_blue);
constexpr std::uint32_t kButtonPen = static_cast<std::uint32_t>(pros::Color::dim_gray);

double d(long value) { return static_cast<double>(value); }

/// Three routines that record which one ran.
struct Routines {
  std::string ran;
  void addTo(AutonSelector& selector) {
    selector.add("left", [this] { ran = "left"; });
    selector.add("right", [this] { ran = "right"; });
    selector.add("skills", [this] { ran = "skills"; });
  }
};

/// Put a finger on the centre of button @p index.
void touchButton(const AutonSelector& selector, int index) {
  const SelectorModel::ButtonRect rect = selector.model().buttonRect(index, SelectorModel::Layout{});
  hs::touch_x = rect.x + rect.w / 2;
  hs::touch_y = rect.y + rect.h / 2;
  hs::touch_down = true;
}

void setButton(DigitalButton button, bool down) {
  hd::button_down[static_cast<int>(mclib::device::toProsDigitalButton(button))] = down;
}

/// add(), select(), next(), previous() and runSelected() pass through to the
/// model, and runSelected() runs the chosen routine.
void reportsChosenRoutine() {
  std::printf("-- runSelected() runs the chosen routine\n");
  hs::reset();
  hd::reset();
  AutonSelector selector;
  CHECK(!selector.runSelected());

  Routines routines;
  routines.addTo(selector);
  CHECK_EQ(d(static_cast<long>(selector.size())), 3.0);
  CHECK_EQ(selector.selected(), 0.0);
  CHECK(selector.runSelected());
  CHECK(routines.ran == "left");

  selector.select(2);
  CHECK(selector.selectedName() == "skills");
  CHECK(selector.runSelected());
  CHECK(routines.ran == "skills");

  selector.next();
  CHECK(selector.selectedName() == "left");
  selector.previous();
  CHECK(selector.selectedName() == "skills");

  // An entry with no function reports false instead of crashing.
  selector.select(selector.add("empty", nullptr));
  CHECK(!selector.runSelected());
}

/// One filled button per entry, the selected one in the highlight colour,
/// and every name printed.
void drawsButtons() {
  std::printf("-- drawBrainScreen() highlights the selection\n");
  hs::reset();
  hd::reset();
  AutonSelector selector;
  selector.drawBrainScreen();
  CHECK_EQ(d(hs::erase_count), 1.0);
  CHECK(hs::filled.empty());
  CHECK(hs::printed.size() == 1 && hs::printed[0] == "No autonomous routines");

  Routines routines;
  routines.addTo(selector);
  selector.select(1);
  hs::clearDrawing();
  selector.drawBrainScreen();
  CHECK_EQ(d(static_cast<long>(hs::filled.size())), 3.0);
  if (hs::filled.size() == 3) {
    CHECK(hs::filled[0].pen == kButtonPen);
    CHECK(hs::filled[1].pen == kSelectedPen);
    CHECK(hs::filled[2].pen == kButtonPen);
    // The rectangle drawn is the one hitTest() uses.
    const SelectorModel::ButtonRect rect = selector.model().buttonRect(1, SelectorModel::Layout{});
    CHECK_EQ(hs::filled[1].x0, rect.x);
    CHECK_EQ(hs::filled[1].y0, rect.y);
    CHECK_EQ(hs::filled[1].x1, rect.x + rect.w - 1);
    CHECK_EQ(hs::filled[1].y1, rect.y + rect.h - 1);
  }
  CHECK(hs::printed.size() == 3 && hs::printed[0] == "left" && hs::printed[1] == "right" &&
        hs::printed[2] == "skills");
}

/// Only the press edge of a touch selects. Holding or dragging does not.
/// A touch between buttons selects nothing but still redraws.
void touchSelectsOnPress() {
  std::printf("-- brain touch selects on the press edge only\n");
  hs::reset();
  hd::reset();
  AutonSelector selector;
  Routines routines;
  routines.addTo(selector);

  touchButton(selector, 1);
  selector.pollBrainTouch();
  CHECK_EQ(selector.selected(), 1.0);
  CHECK_EQ(d(hs::erase_count), 1.0);

  // Still held, dragged onto button 2: no change, no redraw.
  touchButton(selector, 2);
  selector.pollBrainTouch();
  CHECK_EQ(selector.selected(), 1.0);
  CHECK_EQ(d(hs::erase_count), 1.0);

  // Lift, then press on button 2.
  hs::touch_down = false;
  selector.pollBrainTouch();
  CHECK_EQ(selector.selected(), 1.0);
  touchButton(selector, 2);
  selector.pollBrainTouch();
  CHECK_EQ(selector.selected(), 2.0);
  CHECK_EQ(d(hs::erase_count), 2.0);

  // A press above the buttons (the title strip) selects nothing, but redraws.
  hs::touch_down = false;
  selector.pollBrainTouch();
  hs::touch_x = 5;
  hs::touch_y = 5;
  hs::touch_down = true;
  selector.pollBrainTouch();
  CHECK_EQ(selector.selected(), 2.0);
  CHECK_EQ(d(hs::erase_count), 3.0);

  selector.runSelected();
  CHECK(routines.ran == "skills");
}

/// Controller buttons step the selection on the press edge. A button already
/// held when bindController() runs does not count as a press.
void controllerButtonsStep() {
  std::printf("-- controller buttons step the selection\n");
  hs::reset();
  hd::reset();
  AutonSelector selector;
  Routines routines;
  routines.addTo(selector);
  Controller controller;

  setButton(DigitalButton::Right, true);
  selector.bindController(controller);
  selector.pollController();
  CHECK_EQ(selector.selected(), 0.0);

  // Release, press: one step forward, and the brain screen is redrawn.
  setButton(DigitalButton::Right, false);
  selector.pollController();
  const int erases_before = hs::erase_count;
  setButton(DigitalButton::Right, true);
  selector.pollController();
  CHECK_EQ(selector.selected(), 1.0);
  CHECK_EQ(d(hs::erase_count), d(erases_before + 1));

  // Held across polls: still one step.
  selector.pollController();
  selector.pollController();
  CHECK_EQ(selector.selected(), 1.0);

  // Left steps back, and wraps from 0 to the last entry.
  setButton(DigitalButton::Right, false);
  setButton(DigitalButton::Left, true);
  selector.pollController();
  CHECK_EQ(selector.selected(), 0.0);
  setButton(DigitalButton::Left, false);
  selector.pollController();
  setButton(DigitalButton::Left, true);
  selector.pollController();
  CHECK_EQ(selector.selected(), 2.0);

  selector.runSelected();
  CHECK(routines.ran == "skills");
}

/// Custom buttons replace the default Left/Right.
void controllerCustomButtons() {
  std::printf("-- bindController() with custom buttons\n");
  hs::reset();
  hd::reset();
  AutonSelector selector;
  Routines routines;
  routines.addTo(selector);
  Controller controller;
  selector.bindController(controller, DigitalButton::Down, DigitalButton::Up);

  setButton(DigitalButton::Right, true);
  selector.pollController();
  CHECK_EQ(selector.selected(), 0.0);
  setButton(DigitalButton::Up, true);
  selector.pollController();
  CHECK_EQ(selector.selected(), 1.0);
  setButton(DigitalButton::Down, true);
  selector.pollController();
  CHECK_EQ(selector.selected(), 0.0);
}

/// The controller line shows the pick padded to 15 characters, and is not
/// rewritten more often than every 50 ms.
void controllerTextIsPaddedAndRateLimited() {
  std::printf("-- controller line is padded and rate limited\n");
  hs::reset();
  hd::reset();
  hs::now_ms = 1000;
  AutonSelector selector;
  Controller controller;
  selector.bindController(controller);

  selector.pollController();
  CHECK(hd::controller_text.size() == 1 && hd::controller_text.back() == "no autons      ");

  selector.add("a very long routine name", nullptr);
  hs::now_ms += 50;
  selector.pollController();
  CHECK(hd::controller_text.size() == 2 && hd::controller_text.back() == "a very long rou");

  selector.add("x", nullptr);
  selector.select(1);
  // 10 ms later: too soon, nothing written yet.
  hs::now_ms += 10;
  selector.pollController();
  CHECK_EQ(d(static_cast<long>(hd::controller_text.size())), 2.0);
  // 50 ms after the last write: the pending change goes out, padded so the
  // tail of the longer name is wiped.
  hs::now_ms += 40;
  selector.pollController();
  CHECK(hd::controller_text.size() == 3 && hd::controller_text.back() == "x              ");

  // Nothing changed: nothing written, however long it has been.
  hs::now_ms += 500;
  selector.pollController();
  CHECK_EQ(d(static_cast<long>(hd::controller_text.size())), 3.0);
}

/// startPolling() runs until the field reports autonomous, then stops itself
/// and draws once more. It can be started again afterwards.
void pollingStopsOnAutonomous() {
  std::printf("-- the polling task stops itself on autonomous\n");
  hs::reset();
  hd::reset();
  AutonSelector selector;
  Routines routines;
  routines.addTo(selector);
  Controller controller;
  selector.bindController(controller);

  // Press Right on the first poll period, let go on the second, switch the
  // field to autonomous on the third.
  int delays = 0;
  hs::on_delay = [&] {
    ++delays;
    if (delays == 1) setButton(DigitalButton::Right, true);
    if (delays == 2) setButton(DigitalButton::Right, false);
    if (delays == 3) mclib::test::setCompetitionStatus(2);
  };
  mclib::test::setCompetitionStatus(0);
  CHECK(selector.startPolling(20.0 * mclib::units::millisecond));
  CHECK_EQ(d(hs::tasks_started), 1.0);
  CHECK(!selector.isPolling());
  CHECK_EQ(d(delays), 3.0);
  CHECK_EQ(d(hs::now_ms), 60.0);
  CHECK_EQ(selector.selected(), 1.0);
  // Final forced write shows the pick that is about to run.
  CHECK(!hd::controller_text.empty() && hd::controller_text.back() == "right          ");

  // Already autonomous: the new task exits on its first check.
  hs::on_delay = nullptr;
  CHECK(selector.startPolling());
  CHECK_EQ(d(hs::tasks_started), 2.0);
  CHECK(!selector.isPolling());
  selector.stopPolling();
  selector.stopPolling();

  selector.runSelected();
  CHECK(routines.ran == "right");
  mclib::test::setCompetitionStatus(0);
}

/// saveSelection() writes the name; loadSelection() finds it again even
/// after the list is reordered, and leaves the pick alone on a bad file.
void saveAndLoad() {
  std::printf("-- saveSelection() and loadSelection()\n");
  hs::reset();
  hd::reset();
  const std::string path =
      (std::filesystem::temp_directory_path() /
       ("mclib_selector_test_" + std::to_string(::getpid()) + ".txt"))
          .string();

  AutonSelector first;
  CHECK(!first.saveSelection(path.c_str()));  // nothing selected
  Routines routines;
  routines.addTo(first);
  first.select(1);
  CHECK(first.saveSelection(path.c_str()));

  AutonSelector reordered;
  reordered.add("skills", nullptr);
  reordered.add("right", nullptr);
  reordered.add("left", nullptr);
  CHECK(reordered.loadSelection(path.c_str()));
  CHECK(reordered.selectedName() == "right");
  CHECK_EQ(reordered.selected(), 1.0);

  AutonSelector other;
  other.add("elims", nullptr);
  CHECK(!other.loadSelection(path.c_str()));
  CHECK_EQ(other.selected(), 0.0);

  // A hand-edited file with a Windows line ending still loads.
  if (std::FILE* file = std::fopen(path.c_str(), "w")) {
    std::fputs("left\r\n", file);
    std::fclose(file);
  }
  CHECK(reordered.loadSelection(path.c_str()));
  CHECK(reordered.selectedName() == "left");

  std::remove(path.c_str());
  CHECK(!reordered.loadSelection(path.c_str()));
  CHECK(!reordered.loadSelection(nullptr));
  CHECK(reordered.selectedName() == "left");
}

}  // namespace

int main() {
  reportsChosenRoutine();
  drawsButtons();
  touchSelectsOnPress();
  controllerButtonsStep();
  controllerCustomButtons();
  controllerTextIsPaddedAndRateLimited();
  pollingStopsOnAutonomous();
  saveAndLoad();
  return mclib::test::summary("auton_selector");
}
