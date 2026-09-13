// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/types.hpp"

#include "mclib/units/units.hpp"

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
   * its slot rather than the infinity the untyped version leaves there, so a
   * single dead motor does not disguise itself as a reading.
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
   * Sibling of getAveragePositionDeg(), which averages the error sentinels in
   * with the real readings and returns a number that looks fine. An average
   * over a partly-dead drive side is not a position, so this refuses to give
   * you one.
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

  pros::MotorGroup m_motors;
};

}  // namespace device
}  // namespace mclib
