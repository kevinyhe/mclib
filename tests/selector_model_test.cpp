// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @brief Host tests for SelectorModel: selection, wraparound, and the
 * brain-screen hit-testing. No PROS, no screen.
 */

#include "mclib/auton/selector_model.hpp"

#include "test_assert.hpp"

#include <string>

namespace {

using mclib::auton::SelectorModel;

/** @brief An empty model has nothing selected and every call is a no-op. */
void testEmptyModel() {
  SelectorModel model;
  CHECK_EQ(static_cast<double>(model.size()), 0.0);
  CHECK_EQ(model.selected(), -1.0);
  CHECK(model.selectedName().empty());
  model.next();
  model.previous();
  model.select(0);
  CHECK_EQ(model.selected(), -1.0);
  CHECK(!model.runSelected());
  CHECK_EQ(model.hitTest(100, 100, SelectorModel::Layout{}), -1.0);
  const SelectorModel::ButtonRect rect = model.buttonRect(0, SelectorModel::Layout{});
  CHECK_EQ(rect.w, 0.0);
  CHECK_EQ(rect.h, 0.0);
}

/** @brief add() returns indices in order and the first entry is selected. */
void testAddAndSelect() {
  SelectorModel model;
  CHECK_EQ(model.add("left", nullptr), 0.0);
  CHECK_EQ(model.add("right", nullptr), 1.0);
  CHECK_EQ(model.add("skills", nullptr), 2.0);
  CHECK_EQ(static_cast<double>(model.size()), 3.0);
  CHECK_EQ(model.selected(), 0.0);
  CHECK(model.selectedName() == "left");

  model.select(2);
  CHECK(model.selectedName() == "skills");
  // Out of range is ignored, not clamped.
  model.select(3);
  CHECK_EQ(model.selected(), 2.0);
  model.select(-1);
  CHECK_EQ(model.selected(), 2.0);

  CHECK_EQ(model.indexOf("right"), 1.0);
  CHECK_EQ(model.indexOf("missing"), -1.0);
}

/** @brief next() and previous() wrap at both ends. */
void testWraparound() {
  SelectorModel model;
  model.add("a", nullptr);
  model.add("b", nullptr);
  model.add("c", nullptr);

  model.next();
  CHECK_EQ(model.selected(), 1.0);
  model.next();
  CHECK_EQ(model.selected(), 2.0);
  model.next();
  CHECK_EQ(model.selected(), 0.0);

  model.previous();
  CHECK_EQ(model.selected(), 2.0);
  model.previous();
  CHECK_EQ(model.selected(), 1.0);

  // A single entry cycles onto itself.
  SelectorModel one;
  one.add("only", nullptr);
  one.next();
  CHECK_EQ(one.selected(), 0.0);
  one.previous();
  CHECK_EQ(one.selected(), 0.0);
}

/** @brief runSelected() calls the picked function and reports a missing one. */
void testRunSelected() {
  SelectorModel model;
  int ran = 0;
  model.add("counts", [&ran] { ++ran; });
  model.add("no function", nullptr);

  CHECK(model.runSelected());
  CHECK_EQ(ran, 1.0);
  model.next();
  CHECK(!model.runSelected());
  CHECK_EQ(ran, 1.0);
  model.previous();
  CHECK(model.runSelected());
  CHECK_EQ(ran, 2.0);
}

/** @brief Default layout: four entries make a 2 x 2 grid below the status bar. */
void testButtonRectDefaultLayout() {
  SelectorModel model;
  for (int i = 0; i < 4; ++i) {
    model.add("entry " + std::to_string(i), nullptr);
  }
  const SelectorModel::Layout layout;
  CHECK_EQ(model.rows(layout), 2.0);

  // Width: (480 - 3 * 8) / 2 = 228. Height: (208 - 3 * 8) / 2 = 92.
  const SelectorModel::ButtonRect r0 = model.buttonRect(0, layout);
  CHECK_EQ(r0.x, 8.0);
  CHECK_EQ(r0.y, 40.0);
  CHECK_EQ(r0.w, 228.0);
  CHECK_EQ(r0.h, 92.0);

  const SelectorModel::ButtonRect r1 = model.buttonRect(1, layout);
  CHECK_EQ(r1.x, 244.0);
  CHECK_EQ(r1.y, 40.0);

  const SelectorModel::ButtonRect r2 = model.buttonRect(2, layout);
  CHECK_EQ(r2.x, 8.0);
  CHECK_EQ(r2.y, 140.0);

  const SelectorModel::ButtonRect r3 = model.buttonRect(3, layout);
  CHECK_EQ(r3.x, 244.0);
  CHECK_EQ(r3.y, 140.0);
  // The last row ends inside the screen.
  CHECK(r3.y + r3.h <= layout.screen_height);
  CHECK(r1.x + r1.w <= layout.screen_width);

  // Nothing for an index with no entry.
  CHECK_EQ(model.buttonRect(4, layout).w, 0.0);
}

/** @brief Each button's centre maps to its index; gaps and status bar to -1. */
void testHitTest() {
  SelectorModel model;
  for (int i = 0; i < 5; ++i) {
    model.add("entry " + std::to_string(i), nullptr);
  }
  const SelectorModel::Layout layout;
  CHECK_EQ(model.rows(layout), 3.0);

  for (int i = 0; i < 5; ++i) {
    const SelectorModel::ButtonRect rect = model.buttonRect(i, layout);
    CHECK_EQ(model.hitTest(rect.x + rect.w / 2, rect.y + rect.h / 2, layout), i);
    // Corners are inside; one pixel past the far edge is not this button.
    CHECK_EQ(model.hitTest(rect.x, rect.y, layout), i);
    CHECK_EQ(model.hitTest(rect.x + rect.w - 1, rect.y + rect.h - 1, layout), i);
    CHECK(model.hitTest(rect.x + rect.w, rect.y + rect.h / 2, layout) != i);
  }

  // Status bar.
  CHECK_EQ(model.hitTest(240, 0, layout), -1.0);
  CHECK_EQ(model.hitTest(240, 31, layout), -1.0);
  // Padding: left edge, the row above the first button, and the gap between
  // the two columns.
  CHECK_EQ(model.hitTest(3, 60, layout), -1.0);
  CHECK_EQ(model.hitTest(100, 36, layout), -1.0);
  const SelectorModel::ButtonRect r0 = model.buttonRect(0, layout);
  CHECK_EQ(model.hitTest(r0.x + r0.w + 2, r0.y + 10, layout), -1.0);
  // The empty cell next to the fifth entry.
  const SelectorModel::ButtonRect r4 = model.buttonRect(4, layout);
  CHECK_EQ(model.hitTest(r4.x + r4.w + layout.padding + 10, r4.y + 10, layout), -1.0);
  // Off screen.
  CHECK_EQ(model.hitTest(-1, 100, layout), -1.0);
  CHECK_EQ(model.hitTest(480, 100, layout), -1.0);
  CHECK_EQ(model.hitTest(100, 240, layout), -1.0);
}

/** @brief A one-column layout stacks every entry in a single column. */
void testSingleColumnLayout() {
  SelectorModel model;
  model.add("a", nullptr);
  model.add("b", nullptr);
  SelectorModel::Layout layout;
  layout.columns = 1;
  const SelectorModel::ButtonRect r0 = model.buttonRect(0, layout);
  const SelectorModel::ButtonRect r1 = model.buttonRect(1, layout);
  CHECK_EQ(r0.w, 464.0);
  CHECK_EQ(r0.x, r1.x);
  CHECK(r1.y > r0.y + r0.h);
  CHECK_EQ(model.hitTest(240, r1.y + 5, layout), 1.0);
}

}  // namespace

int main() {
  testEmptyModel();
  testAddAndSelect();
  testWraparound();
  testRunSelected();
  testButtonRectDefaultLayout();
  testHitTest();
  testSingleColumnLayout();
  return mclib::test::summary("selector_model_test");
}
