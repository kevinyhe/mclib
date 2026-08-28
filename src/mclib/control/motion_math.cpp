// mclib
#include "mclib/control/motion_math.hpp"

#include <cmath>

namespace mclib {
namespace control {

SlewPlan planSlew(const SlewConfig& config,
                  int drive_direction,
                  bool exit,
                  bool min_speed_requested,
                  double min_speed_output) {
  SlewPlan plan{};
  plan.max_slew_fwd =
      drive_direction > 0 ? config.accel_fwd : config.decel_rev;
  plan.max_slew_rev =
      drive_direction > 0 ? config.decel_fwd : config.accel_rev;
  plan.apply_min_speed_floor = (min_speed_requested && min_speed_output > 0);

  if (exit) {
    return plan;
  }

  // Chaining. The three cases below are the original ones, in the original
  // order, including the fact that they are three independent `if`s rather
  // than an if/else chain - only one of them can match, but the shape is kept
  // so the reading is a diff of the original.
  if (!config.dir_change_start && config.dir_change_end) {
    plan.max_slew_fwd =
        drive_direction > 0 ? config.chain_slew : config.decel_rev;
    plan.max_slew_rev =
        drive_direction > 0 ? config.decel_fwd : config.chain_slew;
  }
  if (config.dir_change_start && !config.dir_change_end) {
    plan.max_slew_fwd =
        drive_direction > 0 ? config.accel_fwd : config.chain_slew;
    plan.max_slew_rev =
        drive_direction > 0 ? config.chain_slew : config.accel_rev;
    plan.apply_min_speed_floor = true;
  }
  if (!config.dir_change_start && !config.dir_change_end) {
    plan.max_slew_fwd = config.chain_slew;
    plan.max_slew_rev = config.chain_slew;
    plan.apply_min_speed_floor = true;
  }

  return plan;
}

double minSpeedOutput(double min_speed, double min_output) {
  return fmax(0.0, min_speed < 0 ? min_output : min_speed);
}

double applyMinSpeedFloor(double output, double min_speed_output) {
  if (min_speed_output > 0 && fabs(output) < min_speed_output) {
    return output >= 0 ? min_speed_output : -min_speed_output;
  }
  return output;
}

double clampSymmetric(double value, double max_output) {
  if (value > max_output) {
    return max_output;
  }
  if (value < -max_output) {
    return -max_output;
  }
  return value;
}

double applySlewLimit(double desired,
                      double previous,
                      double accel_limit,
                      double decel_limit,
                      double loop_dt_ms) {
  constexpr double nominal_loop_ms = 10.0;
  const double dt_scale = loop_dt_ms <= 0 ? 1.0 : loop_dt_ms / nominal_loop_ms;
  const double max_increase = accel_limit * dt_scale;
  const double max_decrease = decel_limit * dt_scale;
  const double delta = desired - previous;

  if (delta > max_increase) {
    return previous + max_increase;
  }
  if (delta < -max_decrease) {
    return previous - max_decrease;
  }
  return desired;
}

void applySlewClamp(double& left_output,
                    double& right_output,
                    double prev_left,
                    double prev_right,
                    double max_slew_fwd,
                    double max_slew_rev,
                    bool limit_decel) {
  if (limit_decel) {
    if (prev_left - left_output > max_slew_rev) {
      left_output = prev_left - max_slew_rev;
    }
    if (prev_right - right_output > max_slew_rev) {
      right_output = prev_right - max_slew_rev;
    }
  }
  if (left_output - prev_left > max_slew_fwd) {
    left_output = prev_left + max_slew_fwd;
  }
  if (right_output - prev_right > max_slew_fwd) {
    right_output = prev_right + max_slew_fwd;
  }
}

void applyOverturnAndMix(double& left_output,
                         double& right_output,
                         double correction,
                         double max_output,
                         bool overturn) {
  const double overturn_value =
      fabs(left_output) + fabs(correction) - max_output;
  if (overturn_value > 0 && overturn) {
    // Give the drive term up, but only down to zero. Subtracting unbounded let
    // it cross zero and come out reversed whenever |correction| > max_output,
    // which boomerang() reaches: it passes the uncapped heading-PID output.
    // Drive 50, correction 37.5, cap 12 gave overturn_value 75.5, so the drive
    // term went +50 -> -25.5 and the pair came out (+2.3, -12) after
    // scaleToMax - a point turn backwards, where the intent is "give up the
    // forward drive, keep the turn". Flooring at zero gives (+12, -12).
    const double kept = fmax(fabs(left_output) - overturn_value, 0.0);
    left_output = left_output < 0.0 ? -kept : kept;
  }
  right_output = left_output;
  left_output = left_output + correction;
  right_output = right_output - correction;
}

double exitDecel(double max_slew_fwd, double max_slew_rev, double max_output) {
  return fmax(fmax(max_slew_fwd, max_slew_rev), max_output / 30.0);
}

}  // namespace control
}  // namespace mclib
