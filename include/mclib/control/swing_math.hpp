// mclib
#pragma once

/**
 * @file swing_math.hpp
 * @brief The pure sign arithmetic behind `swing()`.
 *
 * `swing()` holds one tread and drives the other. Which tread is held, and
 * with what sign the other one is driven, is a decision over three numbers:
 * the heading error at entry, `drive_direction`, and nothing else. None of it
 * touches a sensor, so it lives here and is pinned by
 * `tests/swing_math_test.cpp` instead of being spread across eight copies in
 * `motion.cpp` that had to agree with each other by inspection - which two of
 * them did not.
 *
 * ## Frame
 *
 * Compass headings: clockwise positive. Left tread forward with the right
 * tread held makes the heading **increase**; right tread forward with the left
 * held makes it **decrease**. This is the same convention `correctHeading()`
 * uses when it drives `(output, -output)` to raise the heading.
 *
 * The turn PID's error is `target - current`, so its output is negative
 * exactly when the heading has to come down.
 */

namespace mclib {
namespace control {

/**
 * @brief One tick of swing output: which tread moves, and how hard.
 */
struct SwingCommand {
  bool drive_left;  ///< True drives the left tread and holds the right.
  double voltage;   ///< Signed volts for the driven tread.
};

/**
 * @brief The quadrant `swing()` is operating in, 1 through 4.
 *
 * 1. Heading must fall, driving forward.
 * 2. Heading must rise, driving forward.
 * 3. Heading must fall, driving backward.
 * 4. Heading must rise, driving backward.
 *
 * A zero delta falls into 4, which is what the original chain did.
 *
 * @param swing_angle_deg  Normalised absolute target heading.
 * @param entry_angle_deg  Heading held when the swing was commanded.
 * @param drive_direction  +1 forward, -1 backward.
 */
inline int swingChoice(double swing_angle_deg,
                       double entry_angle_deg,
                       double drive_direction) {
  const double delta = swing_angle_deg - entry_angle_deg;
  if (delta < 0 && drive_direction == 1) {
    return 1;
  }
  if (delta > 0 && drive_direction == 1) {
    return 2;
  }
  if (delta < 0 && drive_direction == -1) {
    return 3;
  }
  return 4;
}

/**
 * @brief The tread and signed voltage for one swing tick.
 *
 * Take choice 1: the heading has to fall, so `output` is negative, and the
 * left tread is held. The heading falls when the right tread goes *forward*,
 * so the right tread wants a positive voltage - `-output * drive_direction`.
 * Choice 3 is the same target with the drive reversed: the right tread is held
 * and the left has to go backward, which is again `-output * drive_direction`
 * once `drive_direction` flips the sign.
 *
 * Choices 2 and 4 raise the heading, `output` is positive, and the plain
 * `output * drive_direction` already points the right way.
 *
 * @param choice           From swingChoice().
 * @param output           Turn PID output, already clamped.
 * @param drive_direction  +1 forward, -1 backward.
 */
inline SwingCommand swingCommand(int choice,
                                 double output,
                                 double drive_direction) {
  switch (choice) {
    case 1:
      return SwingCommand{false, -output * drive_direction};
    case 2:
      return SwingCommand{true, output * drive_direction};
    case 3:
      return SwingCommand{true, -output * drive_direction};
    default:
      return SwingCommand{false, output * drive_direction};
  }
}

}  // namespace control
}  // namespace mclib
