// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/units.hpp"

/**
 * @file motion_config.hpp
 * @brief The one tuning record for the drivetrain.
 *
 * Every drive loop in mclib reads its gains, exit conditions, voltage limits
 * and slew rates from a `MotionConfig`: the blocking routines in
 * `control/motion.hpp`, the scheduler-driven loops in `ChassisController`,
 * and `correctHeading()`. There used to be two copies of these numbers - a
 * set of bare `double` globals in `config.cpp` that `motion.cpp` read, and a
 * `ChassisControllerConfig` that the scheduler loops read - and they could
 * disagree without anything noticing. Now there is one struct, and
 * `ChassisController::setConfig()` is the way to change it.
 *
 * This header has no PROS dependency so host tests can build it.
 */

namespace mclib {

/**
 * @brief PID coefficients for one loop.
 *
 * Deliberately `double`. A gain's dimension is output over input, and the same
 * struct serves a distance loop (volts per inch) and a turn loop (volts per
 * degree). There is no one type that is right for both.
 */
struct PIDGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
};

/**
 * @brief When a PID loop is allowed to declare itself settled.
 *
 * The two error tolerances and the derivative tolerance are in the units of
 * whichever loop this configures - inches for the distance loop, degrees for
 * the turn loop - so they stay `double` for the same reason PIDGains does. The
 * durations are times and say so.
 */
struct PIDExit {
  double small_error = 1.0;  ///< Inches or degrees, per the loop.
  double big_error = 3.0;    ///< Inches or degrees, per the loop.
  QTime small_duration = 50.0 * units::millisecond;
  QTime big_duration = 250.0 * units::millisecond;
  double derivative = 5.0;  ///< Inches or degrees per tick, per the loop.
};

/**
 * @brief Slew limits for the straight-line and point-to-point motions.
 *
 * Each is **volts per nominal 10 ms tick**. That is a rate with no name in the
 * unit system - the tick is the loop period, not a measured time - so it stays
 * a double and the doc comment carries the unit instead.
 */
struct SlewRates {
  double accel_fwd = 1.0;
  double decel_fwd = 1.0;
  double accel_rev = 1.0;
  double decel_rev = 1.0;
};

namespace control {

/**
 * @brief Everything a drivetrain motion loop needs to know that is not the
 *        hardware or the geometry.
 *
 * The defaults are the values one competition robot was tuned against. They
 * are a starting point, not a recommendation: expect to retune every gain for
 * your drive.
 */
struct MotionConfig {
  /// @brief Volts per inch of remaining distance. driveTo, moveToPoint,
  ///        boomerang and curveCircle all run on this.
  PIDGains distance_pid{0.4, 0.0, 3.0};
  /// @brief Volts per degree of heading error, for in-place turns and swings.
  PIDGains turn_pid{0.3, 0.0, 1.5};
  /// @brief Volts per degree, for the heading correction that runs *inside* a
  ///        straight or point-to-point move and for correctHeading().
  PIDGains heading_pid{0.3, 0.0, 1.5};

  /// @brief Exit conditions for the distance loops, in inches.
  PIDExit distance_exit{0.5, 1.5, 50.0 * units::millisecond,
                        250.0 * units::millisecond, 5.0};
  /// @brief Exit conditions for the turn loops, in degrees.
  PIDExit turn_exit{1.0, 3.0, 50.0 * units::millisecond,
                    250.0 * units::millisecond, 4.5};

  /// @brief Voltage cap when a motion does not name its own.
  QVoltage max_voltage = 12.0 * units::volt;

  /**
   * @brief Stiction floor for drive output.
   *
   * A drivetrain does not move at all below some voltage. As a PID converges
   * its output shrinks toward zero and at some point stops being enough to
   * break friction, so the robot stalls an inch short and the loop times out
   * there. The floor keeps the last bit of travel moving. 1.5 V of a 12 V rail
   * is about 12% - enough to creep a geared V5 drive, little enough that
   * arriving is gentle. Zero disables it.
   *
   * The blocking routines apply it on their chained (`exit == false`)
   * branches and whenever the caller passes an explicit `min_speed`; the
   * scheduler-driven driveDistance() applies it whenever it is non-zero.
   */
  QVoltage min_voltage = 1.5 * units::volt;

  /// @brief Slew limits, volts per 10 ms tick.
  SlewRates slew{};
  /// @brief The chain before a motion reverses direction (affects slew planning).
  bool dir_change_start = true;
  /// @brief The chain after a motion reverses direction (affects slew planning).
  bool dir_change_end = true;

  /**
   * @brief Boomerang slip-speed coefficient.
   *
   * Used once, as `sqrt(chase_power * arc_radius_in * 9.8)` inside
   * boomerang(). The 9.8 is gravity in m/s^2 while the radius is inches and
   * the result is compared against volts: it is an empirical fudge factor,
   * not a physical quantity, and it stays `double` for that reason.
   */
  double chase_power = 10.0;

  /// @brief Whether straight moves and correctHeading() steer on the IMU at all.
  bool heading_correction = true;

  /// @brief Exit conditions for curveCircle's outer-wheel distance, in inches.
  ///        These preserve the original arc tolerances independently of the
  ///        straight/point distance_exit rule; tune them for your drivetrain.
  PIDExit arc_exit{0.3, 0.9, 50.0 * units::millisecond,
                   250.0 * units::millisecond, 2.25};
};

/**
 * @brief The MotionConfig every drive loop reads.
 *
 * `ChassisController::setConfig()` writes it. Read at the start of each
 * motion routine, so changing it between motions is safe and changing it
 * mid-motion does nothing to the one already running.
 */
const MotionConfig& motionConfig();

/// @brief Replace the active MotionConfig. See motionConfig().
void setMotionConfig(const MotionConfig& config);

}  // namespace control
}  // namespace mclib
