// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/motor_group.hpp"

#include "pros/error.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {
namespace device {

namespace {
/**
 * @brief True when a raw motor reading is a real measurement.
 *
 * PROS reports a dead or misconfigured port as PROS_ERR_F (infinity) from the
 * double getters and PROS_ERR (INT32_MAX) from the integer ones. Both are
 * values you can keep doing arithmetic with, which is exactly the problem.
 */
bool isReading(double value) {
  return std::isfinite(value) && value != static_cast<double>(PROS_ERR);
}

/// @brief Multiply every raw reading by its unit, keeping order, nullopt for
///        the ones that are not readings.
template <typename Q>
std::vector<std::optional<Q>> scale(const std::vector<double>& values, Q unit) {
  std::vector<std::optional<Q>> typed;
  typed.reserve(values.size());
  for (const double value : values) {
    if (isReading(value)) {
      typed.push_back(value * unit);
    } else {
      typed.push_back(std::nullopt);
    }
  }
  return typed;
}

/// @brief Mean of @p values, or nullopt if empty or any entry is not a reading.
std::optional<double> strictAverage(const std::vector<double>& values) {
  if (values.empty()) {
    return std::nullopt;
  }
  double total = 0.0;
  for (const double value : values) {
    if (!isReading(value)) {
      return std::nullopt;
    }
    total += value;
  }
  return total / static_cast<double>(values.size());
}

/// @brief strictAverage() scaled into its unit.
template <typename Q>
std::optional<Q> averageIn(const std::vector<double>& values, Q unit) {
  const std::optional<double> mean = strictAverage(values);
  if (!mean.has_value()) {
    return std::nullopt;
  }
  return *mean * unit;
}
}  // namespace

MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports,
                       Gearset gearset)
    : MotorGroup(std::vector<std::int8_t>(ports), gearset) {}

MotorGroup::MotorGroup(std::vector<std::int8_t> ports, Gearset gearset)
    : m_motors(std::move(ports), toProsGearset(gearset), pros::MotorUnits::degrees) {}

void MotorGroup::setVoltage(double volts) {
  m_motors.move_voltage(voltsToMillivolts(volts));
}

void MotorGroup::setPercent(double percent) {
  m_motors.move(percentToPower(percent));
}

void MotorGroup::stop() {
  m_motors.brake();
}

void MotorGroup::brake() {
  m_motors.brake();
}

void MotorGroup::setBrakeMode(BrakeMode mode) {
  m_motors.set_brake_mode_all(toProsBrakeMode(mode));
}

void MotorGroup::tarePosition() {
  // tare_position() without _all zeroes a single motor (index 0 by default),
  // which would leave the rest of the group reading their pre-tare positions.
  m_motors.tare_position_all();
}

std::vector<double> MotorGroup::getPositionsDeg() const {
  return m_motors.get_position_all();
}

std::vector<double> MotorGroup::getActualVelocities() const {
  return m_motors.get_actual_velocity_all();
}

std::vector<double> MotorGroup::getCurrentDraws() const {
  const std::vector<std::int32_t> draws = m_motors.get_current_draw_all();
  return std::vector<double>(draws.begin(), draws.end());
}

std::vector<double> MotorGroup::getTemperatures() const {
  return m_motors.get_temperature_all();
}

double MotorGroup::getAveragePositionDeg() const {
  return average(getPositionsDeg());
}

double MotorGroup::getAverageActualVelocity() const {
  return average(getActualVelocities());
}

double MotorGroup::getAverageCurrentDraw() const {
  return average(getCurrentDraws());
}

void MotorGroup::setVoltage(units::QVoltage voltage) {
  setVoltage(voltage.volts());
}

std::vector<std::optional<units::QAngle>> MotorGroup::positions() const {
  return scale<units::QAngle>(getPositionsDeg(), units::degree);
}

std::vector<std::optional<units::QAngularVelocity>> MotorGroup::velocities() const {
  return scale<units::QAngularVelocity>(getActualVelocities(), units::rpm);
}

std::vector<std::optional<units::QCurrent>> MotorGroup::currents() const {
  return scale<units::QCurrent>(getCurrentDraws(), units::milliampere);
}

std::optional<units::QAngle> MotorGroup::averagePosition() const {
  return averageIn<units::QAngle>(getPositionsDeg(), units::degree);
}

std::optional<units::QAngularVelocity> MotorGroup::averageVelocity() const {
  return averageIn<units::QAngularVelocity>(getActualVelocities(), units::rpm);
}

std::optional<units::QCurrent> MotorGroup::averageCurrent() const {
  return averageIn<units::QCurrent>(getCurrentDraws(), units::milliampere);
}

std::int32_t MotorGroup::voltsToMillivolts(double volts) {
  if (!std::isfinite(volts)) return 0;
  return static_cast<std::int32_t>(std::clamp(volts, -12.0, 12.0) * 1000.0);
}

std::int32_t MotorGroup::percentToPower(double percent) {
  if (!std::isfinite(percent)) return 0;
  return static_cast<std::int32_t>(std::clamp(percent, -1.0, 1.0) * 127.0);
}

double MotorGroup::average(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const double value : values) {
    total += value;
  }
  return total / static_cast<double>(values.size());
}

}  // namespace device
}  // namespace mclib
