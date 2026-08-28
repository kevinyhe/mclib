// mclib
/**
 * @file motion_math_test.cpp
 * @brief Freezes the arithmetic the motion routines run, to the last bit.
 *
 * `control/motion.cpp` includes `api.h`, so none of it can run on a host. The
 * parts of it that are pure arithmetic were lifted into
 * `control/motion_math.hpp` unchanged, and this file pins them against a table
 * captured from `main` before the units migration, at `%.17g`, with CHECK_EQ.
 * A one-ulp drift fails the build.
 *
 * The tables below are generated output, not hand-typed expectations. Do not
 * "fix" a number here to make a build go green: a mismatch means the control
 * math moved, which means somebody's tuned autonomous moved with it.
 */
#include "mclib/control/motion_math.hpp"

#include "mclib/control/motion.hpp"
#include "mclib/control/scaling.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cmath>

using mclib::control::applyMinSpeedFloor;
using mclib::control::applyOverturnAndMix;
using mclib::control::applySlewClamp;
using mclib::control::applySlewLimit;
using mclib::control::clampSymmetric;
using mclib::control::exitDecel;
using mclib::control::minSpeedOutput;
using mclib::control::planSlew;
using mclib::control::SlewConfig;
using mclib::control::SlewPlan;

namespace {

/// @brief One row of the planSlew() golden table.
struct PlanRow {
  int dir_change_start;
  int dir_change_end;
  int drive_direction;
  bool exit;
  double min_speed;
  double min_speed_output;
  double max_slew_fwd;
  double max_slew_rev;
  bool apply_min_speed_floor;
};

/// @brief One row of the applySlewLimit() golden table. 1.0 up, 2.5 down.
struct SlewRow {
  double desired;
  double previous;
  double loop_dt_ms;
  double result;
};

/// @brief One row of the applyOverturnAndMix() golden table. Cap 12 V.
struct MixRow {
  double drive;
  double correction;
  bool overturn;
  double left;
  double right;
};

/**
 * @brief One row of the applySlewClamp() golden table.
 *
 * Rates are 1.0 V/tick up and 2.5 V/tick down, and the previous right output
 * is always half the previous left, so one column covers both.
 */
struct ClampRow {
  double left_in;
  double right_in;
  double prev_left;
  bool limit_decel;
  double left_out;
  double right_out;
};

// --- planSlew: four DISTINCT rates so a transposition cannot hide,
// --- accel_fwd 1, decel_fwd 2, accel_rev 3, decel_rev 4 V/tick; min_output 10 V
constexpr PlanRow kPlanRows[] = {
    {0, 0, -1, false, -1, 10, 24, 24, true},
    {0, 0, -1, false, 2.5, 2.5, 24, 24, true},
    {0, 0, -1, false, 0, 0, 24, 24, true},
    {0, 0, -1, true, -1, 10, 4, 3, false},
    {0, 0, -1, true, 2.5, 2.5, 4, 3, true},
    {0, 0, -1, true, 0, 0, 4, 3, false},
    {0, 0, 1, false, -1, 10, 24, 24, true},
    {0, 0, 1, false, 2.5, 2.5, 24, 24, true},
    {0, 0, 1, false, 0, 0, 24, 24, true},
    {0, 0, 1, true, -1, 10, 1, 2, false},
    {0, 0, 1, true, 2.5, 2.5, 1, 2, true},
    {0, 0, 1, true, 0, 0, 1, 2, false},
    {0, 1, -1, false, -1, 10, 4, 24, false},
    {0, 1, -1, false, 2.5, 2.5, 4, 24, true},
    {0, 1, -1, false, 0, 0, 4, 24, false},
    {0, 1, -1, true, -1, 10, 4, 3, false},
    {0, 1, -1, true, 2.5, 2.5, 4, 3, true},
    {0, 1, -1, true, 0, 0, 4, 3, false},
    {0, 1, 1, false, -1, 10, 24, 2, false},
    {0, 1, 1, false, 2.5, 2.5, 24, 2, true},
    {0, 1, 1, false, 0, 0, 24, 2, false},
    {0, 1, 1, true, -1, 10, 1, 2, false},
    {0, 1, 1, true, 2.5, 2.5, 1, 2, true},
    {0, 1, 1, true, 0, 0, 1, 2, false},
    {1, 0, -1, false, -1, 10, 24, 3, true},
    {1, 0, -1, false, 2.5, 2.5, 24, 3, true},
    {1, 0, -1, false, 0, 0, 24, 3, true},
    {1, 0, -1, true, -1, 10, 4, 3, false},
    {1, 0, -1, true, 2.5, 2.5, 4, 3, true},
    {1, 0, -1, true, 0, 0, 4, 3, false},
    {1, 0, 1, false, -1, 10, 1, 24, true},
    {1, 0, 1, false, 2.5, 2.5, 1, 24, true},
    {1, 0, 1, false, 0, 0, 1, 24, true},
    {1, 0, 1, true, -1, 10, 1, 2, false},
    {1, 0, 1, true, 2.5, 2.5, 1, 2, true},
    {1, 0, 1, true, 0, 0, 1, 2, false},
    {1, 1, -1, false, -1, 10, 4, 3, false},
    {1, 1, -1, false, 2.5, 2.5, 4, 3, true},
    {1, 1, -1, false, 0, 0, 4, 3, false},
    {1, 1, -1, true, -1, 10, 4, 3, false},
    {1, 1, -1, true, 2.5, 2.5, 4, 3, true},
    {1, 1, -1, true, 0, 0, 4, 3, false},
    {1, 1, 1, false, -1, 10, 1, 2, false},
    {1, 1, 1, false, 2.5, 2.5, 1, 2, true},
    {1, 1, 1, false, 0, 0, 1, 2, false},
    {1, 1, 1, true, -1, 10, 1, 2, false},
    {1, 1, 1, true, 2.5, 2.5, 1, 2, true},
    {1, 1, 1, true, 0, 0, 1, 2, false},
};

constexpr SlewRow kSlewRows[] = {
    {12, 0, 10, 1},
    {12, 0, 20, 2},
    {12, 0, 5, 0.5},
    {12, 0, 0, 1},
    {12, 11.5, 10, 12},
    {12, 11.5, 20, 12},
    {12, 11.5, 5, 12},
    {12, 11.5, 0, 12},
    {12, -11.5, 10, -10.5},
    {12, -11.5, 20, -9.5},
    {12, -11.5, 5, -11},
    {12, -11.5, 0, -10.5},
    {12, 3, 10, 4},
    {12, 3, 20, 5},
    {12, 3, 5, 3.5},
    {12, 3, 0, 4},
    {-12, 0, 10, -2.5},
    {-12, 0, 20, -5},
    {-12, 0, 5, -1.25},
    {-12, 0, 0, -2.5},
    {-12, 11.5, 10, 9},
    {-12, 11.5, 20, 6.5},
    {-12, 11.5, 5, 10.25},
    {-12, 11.5, 0, 9},
    {-12, -11.5, 10, -12},
    {-12, -11.5, 20, -12},
    {-12, -11.5, 5, -12},
    {-12, -11.5, 0, -12},
    {-12, 3, 10, 0.5},
    {-12, 3, 20, -2},
    {-12, 3, 5, 1.75},
    {-12, 3, 0, 0.5},
    {0, 0, 10, 0},
    {0, 0, 20, 0},
    {0, 0, 5, 0},
    {0, 0, 0, 0},
    {0, 11.5, 10, 9},
    {0, 11.5, 20, 6.5},
    {0, 11.5, 5, 10.25},
    {0, 11.5, 0, 9},
    {0, -11.5, 10, -10.5},
    {0, -11.5, 20, -9.5},
    {0, -11.5, 5, -11},
    {0, -11.5, 0, -10.5},
    {0, 3, 10, 0.5},
    {0, 3, 20, 0},
    {0, 3, 5, 1.75},
    {0, 3, 0, 0.5},
    {3.5, 0, 10, 1},
    {3.5, 0, 20, 2},
    {3.5, 0, 5, 0.5},
    {3.5, 0, 0, 1},
    {3.5, 11.5, 10, 9},
    {3.5, 11.5, 20, 6.5},
    {3.5, 11.5, 5, 10.25},
    {3.5, 11.5, 0, 9},
    {3.5, -11.5, 10, -10.5},
    {3.5, -11.5, 20, -9.5},
    {3.5, -11.5, 5, -11},
    {3.5, -11.5, 0, -10.5},
    {3.5, 3, 10, 3.5},
    {3.5, 3, 20, 3.5},
    {3.5, 3, 5, 3.5},
    {3.5, 3, 0, 3.5},
    {-3.5, 0, 10, -2.5},
    {-3.5, 0, 20, -3.5},
    {-3.5, 0, 5, -1.25},
    {-3.5, 0, 0, -2.5},
    {-3.5, 11.5, 10, 9},
    {-3.5, 11.5, 20, 6.5},
    {-3.5, 11.5, 5, 10.25},
    {-3.5, 11.5, 0, 9},
    {-3.5, -11.5, 10, -10.5},
    {-3.5, -11.5, 20, -9.5},
    {-3.5, -11.5, 5, -11},
    {-3.5, -11.5, 0, -10.5},
    {-3.5, 3, 10, 0.5},
    {-3.5, 3, 20, -2},
    {-3.5, 3, 5, 1.75},
    {-3.5, 3, 0, 0.5},
};

constexpr MixRow kMixRows[] = {
    {12, 0, false, 12, 12},
    {12, 0, true, 12, 12},
    {12, 5, false, 17, 7},
    {12, 5, true, 12, 2},
    {12, -5, false, 7, 17},
    {12, -5, true, 2, 12},
    {12, 9, false, 21, 3},
    {12, 9, true, 12, -6},
    {8, 0, false, 8, 8},
    {8, 0, true, 8, 8},
    {8, 5, false, 13, 3},
    {8, 5, true, 12, 2},
    {8, -5, false, 3, 13},
    {8, -5, true, 2, 12},
    {8, 9, false, 17, -1},
    {8, 9, true, 12, -6},
    {-8, 0, false, -8, -8},
    {-8, 0, true, -8, -8},
    {-8, 5, false, -3, -13},
    {-8, 5, true, -2, -12},
    {-8, -5, false, -13, -3},
    {-8, -5, true, -12, -2},
    {-8, 9, false, 1, -17},
    {-8, 9, true, 6, -12},
    {3, 0, false, 3, 3},
    {3, 0, true, 3, 3},
    {3, 5, false, 8, -2},
    {3, 5, true, 8, -2},
    {3, -5, false, -2, 8},
    {3, -5, true, -2, 8},
    {3, 9, false, 12, -6},
    {3, 9, true, 12, -6},
    {-3, 0, false, -3, -3},
    {-3, 0, true, -3, -3},
    {-3, 5, false, 2, -8},
    {-3, 5, true, 2, -8},
    {-3, -5, false, -8, 2},
    {-3, -5, true, -8, 2},
    {-3, 9, false, 6, -12},
    {-3, 9, true, 6, -12},
    {0, 0, false, 0, 0},
    {0, 0, true, 0, 0},
    {0, 5, false, 5, -5},
    {0, 5, true, 5, -5},
    {0, -5, false, -5, 5},
    {0, -5, true, -5, 5},
    {0, 9, false, 9, -9},
    {0, 9, true, 9, -9},
};

constexpr ClampRow kClampRows[] = {
    {12, 6, 0, false, 1, 1},
    {12, 6, 0, true, 1, 1},
    {12, 6, 10, false, 11, 6},
    {12, 6, 10, true, 11, 6},
    {12, 6, -10, false, -9, -4},
    {12, 6, -10, true, -9, -4},
    {12, -6, 0, false, 1, -6},
    {12, -6, 0, true, 1, -2.5},
    {12, -6, 10, false, 11, -6},
    {12, -6, 10, true, 11, 2.5},
    {12, -6, -10, false, -9, -6},
    {12, -6, -10, true, -9, -6},
    {12, 0, 0, false, 1, 0},
    {12, 0, 0, true, 1, 0},
    {12, 0, 10, false, 11, 0},
    {12, 0, 10, true, 11, 2.5},
    {12, 0, -10, false, -9, -4},
    {12, 0, -10, true, -9, -4},
    {-12, 6, 0, false, -12, 1},
    {-12, 6, 0, true, -2.5, 1},
    {-12, 6, 10, false, -12, 6},
    {-12, 6, 10, true, 7.5, 6},
    {-12, 6, -10, false, -12, -4},
    {-12, 6, -10, true, -12, -4},
    {-12, -6, 0, false, -12, -6},
    {-12, -6, 0, true, -2.5, -2.5},
    {-12, -6, 10, false, -12, -6},
    {-12, -6, 10, true, 7.5, 2.5},
    {-12, -6, -10, false, -12, -6},
    {-12, -6, -10, true, -12, -6},
    {-12, 0, 0, false, -12, 0},
    {-12, 0, 0, true, -2.5, 0},
    {-12, 0, 10, false, -12, 0},
    {-12, 0, 10, true, 7.5, 2.5},
    {-12, 0, -10, false, -12, -4},
    {-12, 0, -10, true, -12, -4},
    {4, 6, 0, false, 1, 1},
    {4, 6, 0, true, 1, 1},
    {4, 6, 10, false, 4, 6},
    {4, 6, 10, true, 7.5, 6},
    {4, 6, -10, false, -9, -4},
    {4, 6, -10, true, -9, -4},
    {4, -6, 0, false, 1, -6},
    {4, -6, 0, true, 1, -2.5},
    {4, -6, 10, false, 4, -6},
    {4, -6, 10, true, 7.5, 2.5},
    {4, -6, -10, false, -9, -6},
    {4, -6, -10, true, -9, -6},
    {4, 0, 0, false, 1, 0},
    {4, 0, 0, true, 1, 0},
    {4, 0, 10, false, 4, 0},
    {4, 0, 10, true, 7.5, 2.5},
    {4, 0, -10, false, -9, -4},
    {4, 0, -10, true, -9, -4},
    {-4, 6, 0, false, -4, 1},
    {-4, 6, 0, true, -2.5, 1},
    {-4, 6, 10, false, -4, 6},
    {-4, 6, 10, true, 7.5, 6},
    {-4, 6, -10, false, -9, -4},
    {-4, 6, -10, true, -9, -4},
    {-4, -6, 0, false, -4, -6},
    {-4, -6, 0, true, -2.5, -2.5},
    {-4, -6, 10, false, -4, -6},
    {-4, -6, 10, true, 7.5, 2.5},
    {-4, -6, -10, false, -9, -6},
    {-4, -6, -10, true, -9, -6},
    {-4, 0, 0, false, -4, 0},
    {-4, 0, 0, true, -2.5, 0},
    {-4, 0, 10, false, -4, 0},
    {-4, 0, 10, true, 7.5, 2.5},
    {-4, 0, -10, false, -9, -4},
    {-4, 0, -10, true, -9, -4},
};

}  // namespace

int main() {
  // -------------------------------------------------------------------------
  // planSlew(): the slew rates and min-speed gating every driving routine
  // starts from.
  //
  // The four rates are deliberately NOT config.cpp's tuned values, which are
  // all 1 V per tick. Four equal rates make the table blind to the one mistake
  // it most needs to catch - transposing accel/decel or forward/reverse in
  // planSlew() or in motion.cpp's slewConfig(). 1/2/3/4 tells them apart.
  // min_output is 10 V, as it is in config.cpp.
  // -------------------------------------------------------------------------
  for (const PlanRow& row : kPlanRows) {
    SlewConfig config{};
    config.accel_fwd = 1.0;
    config.decel_fwd = 2.0;
    config.accel_rev = 3.0;
    config.decel_rev = 4.0;
    config.dir_change_start = row.dir_change_start != 0;
    config.dir_change_end = row.dir_change_end != 0;

    const double min_speed_output = minSpeedOutput(row.min_speed, 10.0);
    CHECK_EQ(min_speed_output, row.min_speed_output);

    const SlewPlan plan = planSlew(config,
                                   row.drive_direction,
                                   row.exit,
                                   row.min_speed >= 0,
                                   min_speed_output);
    CHECK_EQ(plan.max_slew_fwd, row.max_slew_fwd);
    CHECK_EQ(plan.max_slew_rev, row.max_slew_rev);
    CHECK(plan.apply_min_speed_floor == row.apply_min_speed_floor);
  }

  for (const SlewRow& row : kSlewRows) {
    CHECK_EQ(applySlewLimit(row.desired, row.previous, 1.0, 2.5, row.loop_dt_ms),
             row.result);
  }

  for (const MixRow& row : kMixRows) {
    double left = row.drive;
    double right = 0.0;
    applyOverturnAndMix(left, right, row.correction, 12.0, row.overturn);
    CHECK_EQ(left, row.left);
    CHECK_EQ(right, row.right);
  }

  for (const ClampRow& row : kClampRows) {
    double left = row.left_in;
    double right = row.right_in;
    applySlewClamp(left, right, row.prev_left, row.prev_left * 0.5, 1.0, 2.5,
                   row.limit_decel);
    CHECK_EQ(left, row.left_out);
    CHECK_EQ(right, row.right_out);
  }

  // -------------------------------------------------------------------------
  // The audited default: min_speed < 0 selects min_output, which is 10 V of a
  // 12 V rail. Deliberately unchanged by the units migration; see config.hpp.
  // -------------------------------------------------------------------------
  CHECK_EQ(minSpeedOutput(-1.0, 10.0), 10.0);
  CHECK_EQ(minSpeedOutput(-0.0, 10.0), 0.0);  // -0.0 is not < 0
  CHECK_EQ(minSpeedOutput(2.5, 10.0), 2.5);
  CHECK_EQ(minSpeedOutput(-1.0, -3.0), 0.0);  // fmax floors at zero

  CHECK_EQ(applyMinSpeedFloor(0.4, 10.0), 10.0);
  CHECK_EQ(applyMinSpeedFloor(-0.4, 10.0), -10.0);
  CHECK_EQ(applyMinSpeedFloor(0.0, 10.0), 10.0);   // +0.0 >= 0 takes the +ve arm
  CHECK_EQ(applyMinSpeedFloor(-0.0, 10.0), 10.0);  // and so does -0.0
  CHECK_EQ(applyMinSpeedFloor(11.0, 10.0), 11.0);
  CHECK_EQ(applyMinSpeedFloor(0.4, 0.0), 0.4);  // a zero floor is no floor

  CHECK_EQ(clampSymmetric(13.0, 12.0), 12.0);
  CHECK_EQ(clampSymmetric(-13.0, 12.0), -12.0);
  CHECK_EQ(clampSymmetric(11.9, 12.0), 11.9);

  // exitDecel(): from 12 V the tuned 1 V/tick wins over max_output/30, so the
  // post-motion ramp reaches 0 V in 120 ms. A much slower tuning loses to the
  // floor instead, which is what keeps the ramp inside its 500 ms cap.
  CHECK_EQ(exitDecel(1.0, 1.0, 12.0), 1.0);
  CHECK_EQ(exitDecel(0.1, 0.2, 12.0), 0.4);
  CHECK_EQ(exitDecel(24.0, 24.0, 12.0), 24.0);

  // -------------------------------------------------------------------------
  // applyOverturnAndMix() floors the drive term at zero.
  //
  // The golden table above never reaches this: its largest |correction| is
  // 9 V against a 12 V cap, so `overturn_value` never exceeds |drive| and the
  // drive term never crossed zero. Every row of it is unchanged by the floor.
  // `boomerang()` passes the *uncapped* heading-PID output, so |correction|
  // larger than the cap is reachable there, and unbounded subtraction turned
  // the drive term negative - the robot pointed the right way and drove
  // backwards.
  //
  // The review's case: drive 50 V, correction 37.5 V, cap 12 V.
  // overturn_value = 50 + 37.5 - 12 = 75.5. Unfloored that gave a drive term
  // of -25.5, so left = -25.5 + 37.5 = 12 and right = -25.5 - 37.5 = -63,
  // which scaleToMax() shrank to about (+2.3, -12): a backwards point turn.
  // Floored, the drive term is 0 and the pair is (+37.5, -37.5) - a clean
  // point turn that scales to (+12, -12).
  // -------------------------------------------------------------------------
  {
    double left = 50.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, 37.5, 12.0, true);
    CHECK_EQ(left, 37.5);
    CHECK_EQ(right, -37.5);
    scaleToMax(left, right, 12.0);
    CHECK_EQ(left, 12.0);
    CHECK_EQ(right, -12.0);
  }

  // Mirror image: driving backwards, correction the other way. The drive term
  // is floored at zero from below, so it never becomes positive.
  {
    double left = -50.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, -37.5, 12.0, true);
    CHECK_EQ(left, -37.5);
    CHECK_EQ(right, 37.5);
  }

  // The boundary, one volt either side. With drive 8 and cap 12 the floor
  // starts to bite at correction 12, where overturn_value is exactly 8 and the
  // drive term lands on 0 on its own.
  {
    double left = 8.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, 12.0, 12.0, true);  // overturn_value == 8
    CHECK_EQ(left, 12.0);   // drive term 0, so left is just +correction
    CHECK_EQ(right, -12.0);
  }
  {
    double left = 8.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, 11.0, 12.0, true);  // overturn_value == 7
    CHECK_EQ(left, 12.0);   // drive term 1, so 1 + 11
    CHECK_EQ(right, -10.0);
  }
  {
    double left = 8.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, 13.0, 12.0, true);  // overturn_value == 9
    CHECK_EQ(left, 13.0);   // floored: drive term 0, not -1
    CHECK_EQ(right, -13.0);
  }

  // The drive term never crosses zero, over a sweep that includes every case
  // the old code reversed. left + right is twice the drive term, so its sign
  // must match the sign of the drive that went in.
  for (double drive = -60.0; drive <= 60.0; drive += 3.0) {
    for (double correction = -60.0; correction <= 60.0; correction += 3.0) {
      double left = drive;
      double right = 0.0;
      applyOverturnAndMix(left, right, correction, 12.0, true);
      const double drive_out = (left + right) * 0.5;
      CHECK(std::fabs(drive_out) <= std::fabs(drive) + 1e-12);
      CHECK(drive_out * drive >= 0.0);
    }
  }

  // A zero drive term stays at zero rather than being pushed off it. The old
  // `left_output > 0` test sent 0 down the `+=` arm, so a large correction
  // invented forward drive out of nothing.
  {
    double left = 0.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, 30.0, 12.0, true);  // overturn_value == 18
    CHECK_EQ(left, 30.0);
    CHECK_EQ(right, -30.0);
  }

  // overturn == false still disables the trade entirely, floor included.
  {
    double left = 50.0;
    double right = 0.0;
    applyOverturnAndMix(left, right, 37.5, 12.0, false);
    CHECK_EQ(left, 87.5);
    CHECK_EQ(right, 12.5);
  }

  // -------------------------------------------------------------------------
  // What the typed boundary costs.
  //
  // Every motion routine now unwraps its parameters on entry, so the number
  // the loop sees is `(v * unit).accessor()` rather than `v`. Quantity stores
  // SI base units, so volts, seconds and amperes round-trip exactly; inches
  // and degrees are a multiply and a divide by a non-binary constant and can
  // land one ulp away. These are pinned so the size of that error stays
  // visible and stays this small.
  // -------------------------------------------------------------------------
  using namespace mclib::units;

  CHECK_EQ((12.0 * volt).volts(), 12.0);
  CHECK_EQ((10.0 * volt).volts(), 10.0);
  CHECK_EQ((-1.0 * volt).volts(), -1.0);
  CHECK_EQ((-4.0 * volt).volts(), -4.0);
  CHECK_EQ((1000.0 * millisecond).ms(), 1000.0);
  CHECK_EQ((250.0 * millisecond).ms(), 250.0);
  CHECK_EQ((2500.0 * milliampere).mA(), 2500.0);
  CHECK_EQ((5.0 * rpm).rpm(), 5.0);
  CHECK_EQ((90.0 * degree).deg(), 90.0);
  CHECK_EQ((180.0 * degree).deg(), 180.0);
  // Two lengths that survive the inch round trip exactly. This is why
  // Chassis::degreesToInches() computes in inches: it reads
  // `wheel.diameter().in()` and multiplies there, and degreesToDistance()
  // wraps it rather than the other way round. Computing in metres and
  // converting back would have moved 43% of sampled headings by up to 3e-14 in
  // - harmless, but not what this file claims to guarantee. (The wheel these
  // numbers came from, ChassisDimensions' old 2.75 in default, is gone; the
  // shipped wheel is built from a circumference, so its diameter is itself a
  // lossy round trip. See tests/geometry_test.cpp.)
  CHECK_EQ((2.75 * inch).in(), 2.75);
  CHECK_EQ((11.5 * inch).in(), 11.5);

  // ...and two that do not. One ulp, 1.5e-16 relative, on a number a tape
  // measure gave to about 1%.
  CHECK((24.0 * inch).in() != 24.0);
  CHECK_NEAR((24.0 * inch).in(), 24.0, 1e-14);
  CHECK((30.0 * degree).deg() != 30.0);
  CHECK_NEAR((30.0 * degree).deg(), 30.0, 1e-13);

  // The wallReset() "leave the IMU alone" sentinel has to survive unwrapping.
  CHECK(std::isnan(keep_current_heading.rad()));
  CHECK(std::isnan(keep_current_heading.deg()));

  return mclib::test::summary("motion_math");
}
