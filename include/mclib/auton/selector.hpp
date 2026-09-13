// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @brief Pick an autonomous routine on the brain screen or the controller.
 *
 * @details Build it in `initialize()`, add one entry per routine, and call
 * `startPolling()`. Until autonomous starts, a task redraws the buttons,
 * takes touches, watches two controller buttons, and shows the pick on the
 * controller's top line. In `autonomous()`, `runSelected()` runs the pick.
 *
 * The list logic lives in SelectorModel (selector_model.hpp) and is tested
 * on the host. This class only adds the PROS screen, controller, task and
 * file calls.
 */

#include "mclib/auton/selector_model.hpp"
#include "mclib/device/controller.hpp"
#include "mclib/device/types.hpp"
#include "mclib/units/units.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace mclib {
namespace auton {

/** @brief Where saveSelection() and loadSelection() keep the pick. */
inline constexpr const char* kSelectionPath = "/usd/auton_selection.txt";

/**
 * @brief Fastest the controller screen is rewritten.
 *
 * @details The V5 controller drops text updates that arrive faster than about
 * every 50 ms, so a selector that wrote on every poll would show nothing.
 */
inline constexpr std::uint32_t kControllerTextIntervalMs = 50;

class AutonSelector {
 public:
  AutonSelector() = default;
  ~AutonSelector();

  AutonSelector(const AutonSelector&) = delete;
  AutonSelector& operator=(const AutonSelector&) = delete;

  /** @brief Append a routine. Returns its index. */
  int add(std::string name, std::function<void()> run);

  void select(int index);
  void next();
  void previous();
  int selected() const { return m_model.selected(); }
  const std::string& selectedName() const { return m_model.selectedName(); }
  std::size_t size() const { return m_model.size(); }

  /**
   * @brief Run the selected routine on the calling thread.
   * @return false when there is no entry or it has no function.
   */
  bool runSelected() const { return m_model.runSelected(); }

  const SelectorModel& model() const { return m_model; }

  /** @brief Change where the buttons go. Takes effect on the next draw. */
  void setLayout(const SelectorModel::Layout& layout) { m_layout = layout; }

  /** @brief Draw every button, the selected one highlighted. */
  void drawBrainScreen();

  /**
   * @brief Turn a new touch into a selection and redraw.
   *
   * @details Only the press edge counts. A finger held or dragged across the
   * screen does not keep re-selecting whatever it is over.
   */
  void pollBrainTouch();

  /**
   * @brief Watch two controller buttons and mirror the pick on line 0.
   *
   * @param controller Kept by pointer; must outlive this selector.
   * @param previous_button Steps the selection back. Default Left.
   * @param next_button Steps it forward. Default Right.
   */
  void bindController(device::Controller& controller,
                      device::DigitalButton previous_button = device::DigitalButton::Left,
                      device::DigitalButton next_button = device::DigitalButton::Right);

  /** @brief Edge-detect the bound buttons and refresh the controller text. */
  void pollController();

  /** @brief pollBrainTouch() then pollController(). */
  void poll();

  /**
   * @brief Write the selected entry's name to @p path.
   *
   * @details The name, not the index, so reordering the routine list in code
   * never silently changes which one the saved file picks.
   * @return false with nothing selected or when the file cannot be written.
   */
  bool saveSelection(const char* path = kSelectionPath) const;

  /**
   * @brief Select the entry whose name is in @p path.
   * @return false when the file is missing or names no entry; the selection
   *   is left alone in that case.
   */
  bool loadSelection(const char* path = kSelectionPath);

  /**
   * @brief Start a task that polls every @p period until stopped or until
   * the field switches to autonomous.
   *
   * @details Stops itself on `pros::competition::is_autonomous()` so the
   * poll never fights the routine for the controller or screen. Draws once
   * more on the way out so the screen shows the final pick.
   * @return false if already running.
   */
  bool startPolling(units::QTime period = 50.0 * units::millisecond);

  /** @brief Stop the polling task and wait for it. Safe to call twice. */
  void stopPolling();

  bool isPolling() const { return m_should_run.load(); }

 private:
  void runPollLoop(std::uint32_t period_ms);
  void markTextDirty() { m_text_dirty = true; }
  void refreshControllerText(bool force);

  SelectorModel m_model;
  SelectorModel::Layout m_layout;

  bool m_touch_was_down = false;

  device::Controller* m_controller = nullptr;
  device::DigitalButton m_previous_button = device::DigitalButton::Left;
  device::DigitalButton m_next_button = device::DigitalButton::Right;
  bool m_previous_was_down = false;
  bool m_next_was_down = false;
  bool m_text_dirty = true;
  std::uint32_t m_last_text_ms = 0;

  std::atomic_bool m_should_run{false};
  std::unique_ptr<pros::Task> m_task;
};

}  // namespace auton
}  // namespace mclib
