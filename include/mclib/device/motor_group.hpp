// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/types.hpp"

#include "mclib/units/units.hpp"

#include <atomic>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <vector>

namespace mclib {
namespace device {

/**
 * @brief A set of V5 motors driven as one.
 *
 * Mirrors Motor: each `double` getter has a units-typed sibling reading the
 * same hardware. getTemperatures() and setPercent() have none, for the reasons
 * given on Motor.
 */
class MotorGroup {
public:
  MotorGroup(std::initializer_list<std::int8_t> ports,
             Gearset gearset = Gearset::Blue);
  MotorGroup(std::vector<std::int8_t> ports, Gearset gearset = Gearset::Blue);

  /**
   * @brief 11 W and 5.5 W motors geared to turn one shaft at the same top
   *        speed, such as one side of a 55 W drive: two 11 W motors and one
   *        5.5 W motor.
   *
   * @p ports are 11 W motors with @p gearset, and @p ports_5_5w are 5.5 W
   * motors. Every reading is on the 11 W motors' shaft: a 5.5 W motor's
   * position and speed are multiplied by gearsetRpm(@p gearset) / 200. Every
   * command is the same fraction of each motor's full speed: a 5.5 W motor
   * gets kMotor5_5WFullVolts / 12 of the voltage, so below full power it does
   * not run ahead of the 11 W motors and fight them.
   *
   * Per-motor readings come in the order @p ports, then @p ports_5_5w.
   */
  MotorGroup(std::vector<std::int8_t> ports, Gearset gearset,
             std::vector<std::int8_t> ports_5_5w);

  void setVoltage(double volts);
  void setPercent(double percent);
  void stop();
  void brake();
  void setBrakeMode(BrakeMode mode);
  void tarePosition();

  std::vector<double> getPositionsDeg() const;
  std::vector<double> getActualVelocities() const;
  std::vector<double> getCurrentDraws() const;
  std::vector<double> getTemperatures() const;
  double getAveragePositionDeg() const;
  double getAverageActualVelocity() const;
  double getAverageCurrentDraw() const;

  /**
   * @brief Commanded voltage for every motor, typed. Sibling of
   *        setVoltage(double); same +/-12 V clamp, overload not replacement.
   */
  void setVoltage(units::QVoltage voltage);

  /**
   * @brief Per-motor positions, typed, one entry per motor in port order.
   *
   * Sibling of getPositionsDeg(). A motor that is not reporting is nullopt in
   * its slot rather than the infinity the untyped version leaves there.
   */
  std::vector<std::optional<units::QAngle>> positions() const;
  /// @brief Per-motor speeds, typed. Sibling of getActualVelocities() (RPM);
  ///        nullopt per motor that is not reporting.
  std::vector<std::optional<units::QAngularVelocity>> velocities() const;
  /// @brief Per-motor current draw, typed. Sibling of getCurrentDraws(), which
  ///        returns milliamps; nullopt per motor that is not reporting.
  std::vector<std::optional<units::QCurrent>> currents() const;

  /**
   * @brief Mean position, typed, or nullopt if the group is empty or any
   *        motor is not reporting.
   *
   * Sibling of getAveragePositionDeg(), which skips motors that are not
   * reporting and averages the rest (NaN when none are). This one refuses an
   * average over a partly-dead drive side.
   */
  std::optional<units::QAngle> averagePosition() const;
  /// @brief Mean speed, typed. Sibling of getAverageActualVelocity(); nullopt
  ///        if the group is empty or any motor is not reporting.
  std::optional<units::QAngularVelocity> averageVelocity() const;
  /// @brief Mean current draw, typed. Sibling of getAverageCurrentDraw(),
  ///        which returns milliamps; nullopt if the group is empty or any
  ///        motor is not reporting.
  std::optional<units::QCurrent> averageCurrent() const;

private:
  static std::int32_t voltsToMillivolts(double volts);
  static std::int32_t percentToPower(double percent);
  static double average(const std::vector<double>& values);

  std::vector<double> readEach(double (pros::Motor::*read)(std::uint8_t) const) const;
  void reportMotorsInTheWrongList() const;

  std::vector<pros::Motor> m_motors;
  std::vector<bool> m_listed_5_5w;
  double m_5_5w_reading_scale = 1.0;
  mutable std::atomic<bool> m_types_checked{false};
};

}  // namespace device
}  // namespace mclib
