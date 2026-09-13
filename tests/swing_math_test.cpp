// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file swing_math_test.cpp
 * @brief Pins the sign matrix `swing()` drives its treads with.
 *
 * `control/motion.cpp` includes `api.h` and reads the IMU, so `swing()` itself
 * cannot run here. Its direction decision is pure arithmetic over the heading
 * error and `drive_direction`, so it lives in `control/swing_math.hpp` and is
 * checked directly.
 *
 * The bug this file exists to catch: the chained (`exit == false`) path and
 * the settling (`exit == true`) path used to be two hand-written copies of the
 * same decision, and they disagreed for choices 1 and 3. `swing(-90_deg, 1,
 * 2_s, false)` turned right, away from the target, for the whole time limit.
 * There is now one copy, and the invariant test below is what says so.
 *
 * Frame reminder: compass headings, clockwise positive. Left tread forward
 * with the right held raises the heading; right tread forward with the left
 * held lowers it. The turn PID's error is `target - current`, so its output is
 * negative exactly when the heading has to come down.
 */
#include "mclib/control/swing_math.hpp"

#include "test_assert.hpp"

using mclib::control::SwingCommand;
using mclib::control::swingChoice;
using mclib::control::swingCommand;

namespace {

/**
 * @brief The sign of the heading change the commanded tread produces.
 *
 * +1 the heading rises, -1 it falls, 0 nothing moves. Driving the left tread
 * forward raises the heading; driving the right tread forward lowers it.
 */
int headingSign(const SwingCommand& cmd) {
  if (cmd.voltage == 0) {
    return 0;
  }
  const int volt_sign = cmd.voltage > 0 ? 1 : -1;
  return cmd.drive_left ? volt_sign : -volt_sign;
}

/** @brief The turn PID output for a swing that has to move @p delta degrees. */
double pidOutputFor(double delta_deg) {
  // Error is target - current, and any positive gain preserves its sign. The
  // magnitude is irrelevant to every assertion here.
  return delta_deg;
}


/**
 * @brief The old `exit == true` switch from `swing()`, transcribed verbatim.
 *
 * Kept as a second, independent statement of the sign matrix so
 * `swingCommand()` is checked against something rather than against itself.
 */
SwingCommand settlingReference(int choice, double output, double drive_direction) {
  switch (choice) {
    case 1:  // holdLeftSide(); right_chassis.setVoltage(-output * dd);
      return SwingCommand{false, -output * drive_direction};
    case 2:  // left_chassis.setVoltage(output * dd); holdRightSide();
      return SwingCommand{true, output * drive_direction};
    case 3:  // left_chassis.setVoltage(-output * dd); holdRightSide();
      return SwingCommand{true, -output * drive_direction};
    default:  // holdLeftSide(); right_chassis.setVoltage(output * dd);
      return SwingCommand{false, output * drive_direction};
  }
}

}  // namespace

int main() {
  // ---------------------------------------------------------------------
  // Quadrant selection.
  // ---------------------------------------------------------------------
  CHECK_EQ(swingChoice(-90, 0, 1), 1);   // heading falls, forward
  CHECK_EQ(swingChoice(90, 0, 1), 2);    // heading rises, forward
  CHECK_EQ(swingChoice(-90, 0, -1), 3);  // heading falls, backward
  CHECK_EQ(swingChoice(90, 0, -1), 4);   // heading rises, backward
  // Entry heading is not assumed to be zero.
  CHECK_EQ(swingChoice(10, 45, 1), 1);
  CHECK_EQ(swingChoice(45, 10, -1), 4);
  // A zero delta falls into 4, which is what the original if-chain did.
  CHECK_EQ(swingChoice(30, 30, 1), 4);
  CHECK_EQ(swingChoice(30, 30, -1), 4);

  // ---------------------------------------------------------------------
  // Which tread is held.
  // ---------------------------------------------------------------------
  // 1 and 4 hold the left tread, 2 and 3 hold the right.
  CHECK(!swingCommand(1, -50, 1).drive_left);
  CHECK(swingCommand(2, 50, 1).drive_left);
  CHECK(swingCommand(3, -50, -1).drive_left);
  CHECK(!swingCommand(4, 50, -1).drive_left);

  // ---------------------------------------------------------------------
  // The sign matrix. Each row is (choice, drive_direction, delta) and the
  // commanded tread must move the heading toward the target.
  // ---------------------------------------------------------------------
  struct Row {
    int choice;
    double drive_direction;
    double delta_deg;  // target - entry
  };
  const Row rows[] = {
      {1, 1, -90},   // hold left, right forward  -> heading falls
      {2, 1, 90},    // hold right, left forward  -> heading rises
      {3, -1, -90},  // hold right, left backward -> heading falls
      {4, -1, 90},   // hold left, right backward -> heading rises
      {1, 1, -5},
      {2, 1, 5},
      {3, -1, -5},
      {4, -1, 5},
  };

  for (const Row& row : rows) {
    // The choice the routine would have picked for this row.
    CHECK_EQ(swingChoice(row.delta_deg, 0, row.drive_direction), row.choice);

    const double output = pidOutputFor(row.delta_deg);
    const SwingCommand cmd = swingCommand(row.choice, output, row.drive_direction);

    // The heading must move toward the target, not away from it.
    const int want = row.delta_deg > 0 ? 1 : -1;
    CHECK_EQ(headingSign(cmd), want);

    // Forward means the driven tread pushes the robot forward, backward means
    // back. Choices 1 and 2 drive forward, 3 and 4 drive backward.
    const int volt_sign = cmd.voltage > 0 ? 1 : -1;
    CHECK_EQ(volt_sign, row.drive_direction > 0 ? 1 : -1);
  }

  // ---------------------------------------------------------------------
  // The invariant that exposed the bug: for every (choice, drive_direction)
  // pair the chained path and the settling path must command the same tread
  // with the same sign.
  //
  // `settlingReference` below is the old `exit == true` switch, transcribed.
  // The review established that path was the correct one (choices 2 and 4
  // agreed between the two paths, which is what pinned the sign). The old
  // chained path differed for choices 1 and 3; the shared function must match
  // this reference for all four.
  // ---------------------------------------------------------------------
  const double drive_directions[] = {1.0, -1.0};
  const double outputs[] = {-12.0, -1.5, 1.5, 12.0};
  for (int choice = 1; choice <= 4; ++choice) {
    for (double dd : drive_directions) {
      for (double out : outputs) {
        const SwingCommand settling = settlingReference(choice, out, dd);
        const SwingCommand actual = swingCommand(choice, out, dd);
        CHECK_EQ(actual.drive_left, settling.drive_left);
        CHECK_EQ(actual.voltage, settling.voltage);
      }
    }
  }

  // ---------------------------------------------------------------------
  // Regression: swing(-90_deg, 1, 2_s, false) from a heading of 0.
  //
  // Before the fix the chained path commanded the right tread with
  // `output * drive_direction`, which is negative, driving the right tread
  // backward and raising the heading - the wrong way, for the full time limit.
  // ---------------------------------------------------------------------
  {
    const int choice = swingChoice(-90, 0, 1);
    CHECK_EQ(choice, 1);
    const SwingCommand cmd = swingCommand(choice, pidOutputFor(-90), 1);
    CHECK(!cmd.drive_left);      // left tread held
    CHECK(cmd.voltage > 0);      // right tread forward
    CHECK_EQ(headingSign(cmd), -1);
  }
  // The mirror case, backward. Left tread driven backward, heading falls.
  {
    const int choice = swingChoice(-90, 0, -1);
    CHECK_EQ(choice, 3);
    const SwingCommand cmd = swingCommand(choice, pidOutputFor(-90), -1);
    CHECK(cmd.drive_left);
    CHECK(cmd.voltage < 0);
    CHECK_EQ(headingSign(cmd), -1);
  }

  // ---------------------------------------------------------------------
  // Magnitude passes through untouched: only the sign is decided here.
  // ---------------------------------------------------------------------
  CHECK_EQ(swingCommand(1, -7.25, 1).voltage, 7.25);
  CHECK_EQ(swingCommand(2, 7.25, 1).voltage, 7.25);
  CHECK_EQ(swingCommand(3, -7.25, -1).voltage, -7.25);
  CHECK_EQ(swingCommand(4, 7.25, -1).voltage, -7.25);

  return mclib::test::summary("swing_math");
}
