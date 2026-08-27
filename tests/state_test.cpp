// mclib
#include "mclib/control/state.hpp"

#include "test_assert.hpp"

// state.cpp is nothing but definitions of shared globals. There is no
// behaviour to exercise, so this test pins down the startup values that the
// control code assumes and proves the definitions actually link.

int main() {
  CHECK(is_turning == false);
  CHECK_EQ(prev_left_output, 0.0);
  CHECK_EQ(prev_right_output, 0.0);
  CHECK_EQ(xpos, 0.0);
  CHECK_EQ(ypos, 0.0);
  CHECK_EQ(correct_angle, 0.0);

  // They are writable globals, not constants.
  xpos = 12.5;
  correct_angle = -1.25;
  is_turning = true;
  CHECK_EQ(xpos, 12.5);
  CHECK_EQ(correct_angle, -1.25);
  CHECK(is_turning == true);

  return mclib::test::summary("state");
}
