// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/control/motion.hpp"
#include "mclib/telemetry/logger.hpp"
#include "mclib/units/units.hpp"

#include <functional>
#include <vector>

/**
 * @file characterize.hpp
 * @brief Robot routines that log what `tools/tune_drive.py` needs to fit the
 *        drive feedforward, the effective track width and RAMSETE's gains.
 *
 * All three write the same CSV columns through a `telemetry::Logger`:
 *
 * | Column | Meaning |
 * | --- | --- |
 * | `phase` | 1 driving straight, 2 spinning, 3 tracing a motion, 0 between |
 * | `left_cmd_V`, `right_cmd_V` | voltage commanded to each side |
 * | `left_in`, `right_in` | each side's encoder travel since the routine began |
 * | `heading_deg` | IMU heading |
 * | `x_in`, `y_in` | odometry position |
 * | `target_x_in`, `target_y_in`, `target_heading_deg` | the motion's target, while tracing |
 *
 * Create the logger with a 10 ms period, and drain it with a
 * `telemetry::FlushTask` (or `flush()` afterwards). Give the robot room: the
 * drive routine goes forward and back at each voltage, so it ends near where
 * it started, but each run covers up to about 2 ft at 9 V on a fast base.
 *
 * ```cpp
 * static mclib::telemetry::RowBuffer<512> buffer;
 * mclib::telemetry::SdCardSink sink("tune");
 * mclib::telemetry::Logger log(sink, buffer, {.period = 10_ms});
 * mclib::control::TuningLog columns(log);
 * mclib::telemetry::FlushTask flusher(log);
 * mclib::control::characterizeDrive(columns);
 * mclib::control::characterizeSpin(columns);
 * mclib::control::traceMotion(columns, [&] { return followTrajectory(path, follow, 8_s); });
 * flusher.stop();
 * ```
 *
 * Then on a computer: `python3 tools/tune_drive.py tune000.csv`.
 */

namespace mclib {
namespace control {

/// @brief The CSV columns the tuning routines write. Register before sampling.
class TuningLog {
 public:
  /// @brief Registers the columns on @p logger. @p logger must outlive this.
  explicit TuningLog(telemetry::Logger& logger);

  telemetry::Logger& logger() { return m_logger; }

  /**
   * @brief Set every column for this tick and sample.
   * @param phase 1 straight, 2 spin, 3 motion, 0 between.
   * @param left_volts,right_volts Commanded side voltages.
   * @param target_x_in,target_y_in,target_heading_deg The motion's target,
   *        or NaN when there is none.
   */
  void record(int phase, double left_volts, double right_volts,
              double target_x_in = NAN, double target_y_in = NAN,
              double target_heading_deg = NAN);

  /// @brief Encoder travel is measured from here.
  void zeroEncoders();

 private:
  telemetry::Logger& m_logger;
  telemetry::Channel<double> m_phase, m_left_cmd, m_right_cmd, m_left, m_right, m_heading,
      m_x, m_y, m_target_x, m_target_y, m_target_heading;
  double m_left_zero = 0, m_right_zero = 0;
};

/**
 * @brief Drive straight at each voltage, forward then back, logging phase 1.
 *
 * Each direction runs for @p each, long enough to reach a steady speed; the
 * script uses the second half of each run for kS and kV and the start for
 * kA. 0.3 s at 0 V separates runs. Stops on cancel, disable or a fault, like
 * any motion.
 *
 * @param log Where to write.
 * @param volts Voltages to hold. Three spread over the range fit best.
 * @param each How long to hold each voltage in each direction.
 */
MotionResult characterizeDrive(TuningLog& log,
                               std::vector<QVoltage> volts = {3.0 * units::volt, 6.0 * units::volt,
                                                              9.0 * units::volt},
                               QTime each = 1.2 * units::second);

/**
 * @brief Spin in place at each voltage, clockwise then back, logging phase 2.
 *
 * Left at +V, right at -V, then the reverse. The script compares the turn
 * rate with what the fitted feedforward predicts for those wheel speeds to
 * get the effective track width.
 */
MotionResult characterizeSpin(TuningLog& log,
                              std::vector<QVoltage> volts = {4.0 * units::volt, 6.0 * units::volt,
                                                             8.0 * units::volt},
                              QTime each = 1.5 * units::second);

/**
 * @brief Run @p motion on its own task and log it, phase 3, until it ends.
 *
 * Target columns come from the motion's own telemetry, so for
 * `followTrajectory()` they are the trajectory sample RAMSETE is chasing.
 * The commanded voltages are the motion's drive and yaw requests mixed.
 */
MotionResult traceMotion(TuningLog& log, std::function<MotionResult()> motion);

}  // namespace control
}  // namespace mclib
