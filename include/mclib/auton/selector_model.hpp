// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @brief The list behind an autonomous selector, with no screen attached.
 *
 * @details Everything a selector has to get right - which entry is picked,
 * what "next" does at the end of the list, which button a touch landed on -
 * is arithmetic on a vector. Keeping it here, with no PROS include, means a
 * host test can check the wraparound and the hit-testing without a brain.
 * `AutonSelector` in selector.hpp wraps this and does the drawing.
 */

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace mclib {
namespace auton {

/** @brief One selectable routine: what to call it and what to run. */
struct SelectorEntry {
  std::string name;
  std::function<void()> run;
};

/**
 * @brief An ordered list of routines with one of them selected.
 *
 * @details Selection is by index; `next()` and `previous()` wrap around so a
 * controller button can cycle through the whole list in either direction.
 * Every call is a no-op on an empty model - there is nothing to select and
 * nothing to run, so nothing here can index out of range.
 */
class SelectorModel {
 public:
  /**
   * @brief Where the buttons go on the brain screen.
   *
   * @details The V5 screen is 480 x 240 and PROS keeps the top 32 rows for
   * its status bar, so buttons start at `top`. Entries fill a grid of
   * `columns` columns, as many rows as needed, with `padding` pixels between
   * buttons and around the edge. Row heights shrink as entries are added, so
   * every entry always fits on the one screen.
   */
  struct Layout {
    int screen_width = 480;
    int screen_height = 240;
    int top = 32;
    int columns = 2;
    int padding = 8;
  };

  /** @brief A button's pixel rectangle. `w` and `h` are 0 for no button. */
  struct ButtonRect {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
  };

  /**
   * @brief Append an entry. The first one added becomes the selection.
   * @return The new entry's index.
   */
  int add(std::string name, std::function<void()> run) {
    m_entries.push_back(SelectorEntry{std::move(name), std::move(run)});
    if (m_selected < 0) {
      m_selected = 0;
    }
    return static_cast<int>(m_entries.size()) - 1;
  }

  std::size_t size() const { return m_entries.size(); }

  /** @brief Index of the selected entry, or -1 when the model is empty. */
  int selected() const { return m_selected; }

  /** @brief Name of the selected entry, or "" when the model is empty. */
  const std::string& selectedName() const {
    static const std::string kEmpty;
    return m_selected < 0 ? kEmpty : m_entries[static_cast<std::size_t>(m_selected)].name;
  }

  /** @brief Select by index. Out-of-range indices are ignored. */
  void select(int index) {
    if (index >= 0 && static_cast<std::size_t>(index) < m_entries.size()) {
      m_selected = index;
    }
  }

  /** @brief Index of the entry called @p name, or -1 when there is none. */
  int indexOf(const std::string& name) const {
    for (std::size_t i = 0; i < m_entries.size(); ++i) {
      if (m_entries[i].name == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  /** @brief Move the selection one forward, wrapping from last to first. */
  void next() {
    if (m_entries.empty()) {
      return;
    }
    m_selected = (m_selected + 1) % static_cast<int>(m_entries.size());
  }

  /** @brief Move the selection one back, wrapping from first to last. */
  void previous() {
    if (m_entries.empty()) {
      return;
    }
    const int count = static_cast<int>(m_entries.size());
    m_selected = (m_selected + count - 1) % count;
  }

  /**
   * @brief Call the selected entry's function.
   * @return false when the model is empty or the entry has no function.
   */
  bool runSelected() const {
    if (m_selected < 0) {
      return false;
    }
    const SelectorEntry& entry = m_entries[static_cast<std::size_t>(m_selected)];
    if (!entry.run) {
      return false;
    }
    entry.run();
    return true;
  }

  /** @brief Rows the grid needs for the current entry count. */
  int rows(const Layout& layout) const {
    if (m_entries.empty() || layout.columns <= 0) {
      return 0;
    }
    const int count = static_cast<int>(m_entries.size());
    return (count + layout.columns - 1) / layout.columns;
  }

  /**
   * @brief Pixel rectangle of entry @p index's button.
   *
   * @details Returns an all-zero rectangle for an index that has no entry, or
   * a layout with no room, so a caller drawing it draws nothing.
   */
  ButtonRect buttonRect(int index, const Layout& layout) const {
    const int row_count = rows(layout);
    if (index < 0 || static_cast<std::size_t>(index) >= m_entries.size() || row_count == 0) {
      return ButtonRect{};
    }
    const int usable_height = layout.screen_height - layout.top;
    const int w = (layout.screen_width - layout.padding * (layout.columns + 1)) / layout.columns;
    const int h = (usable_height - layout.padding * (row_count + 1)) / row_count;
    if (w <= 0 || h <= 0) {
      return ButtonRect{};
    }
    const int column = index % layout.columns;
    const int row = index / layout.columns;
    return ButtonRect{layout.padding + column * (w + layout.padding),
                      layout.top + layout.padding + row * (h + layout.padding), w, h};
  }

  /**
   * @brief Which entry a touch at (@p x, @p y) lands on, or -1.
   *
   * @details Touches in the padding between buttons and in the status bar
   * above them are -1, so a finger dragged across the screen only ever
   * selects a button it is actually on.
   */
  int hitTest(int x, int y, const Layout& layout) const {
    for (std::size_t i = 0; i < m_entries.size(); ++i) {
      const ButtonRect rect = buttonRect(static_cast<int>(i), layout);
      if (rect.w <= 0 || rect.h <= 0) {
        continue;
      }
      if (x >= rect.x && x < rect.x + rect.w && y >= rect.y && y < rect.y + rect.h) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  const std::vector<SelectorEntry>& entries() const { return m_entries; }

 private:
  std::vector<SelectorEntry> m_entries;
  int m_selected = -1;
};

}  // namespace auton
}  // namespace mclib
