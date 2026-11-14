// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/motor_group.hpp"

#include "pros/error.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    : MotorGroup(std::move(ports), gearset, {}) {}

MotorGroup::MotorGroup(std::vector<std::int8_t> ports, Gearset gearset,
                       std::vector<std::int8_t> ports_5_5w)
    : m_5_5w_reading_scale(readingScaleFor5_5WMotor(gearset)) {
  m_motors.reserve(ports.size() + ports_5_5w.size());
  for (const std::int8_t port : ports) {
    m_motors.emplace_back(port, toProsGearset(gearset), pros::MotorUnits::degrees);
    m_listed_5_5w.push_back(false);
  }
  for (const std::int8_t port : ports_5_5w) {
    m_motors.emplace_back(port, pros::MotorGears::green, pros::MotorUnits::degrees);
    m_listed_5_5w.push_back(true);
  }
}

void MotorGroup::setVoltage(double volts) {
  reportMotorsInTheWrongList();
  const double clamped = std::clamp(volts, -12.0, 12.0);
  for (std::size_t i = 0; i < m_motors.size(); ++i) {
    m_motors[i].move_voltage(
        voltsToMillivolts(m_listed_5_5w[i] ? voltsFor5_5WMotor(clamped) : clamped));
  }
}

void MotorGroup::setPercent(double percent) {
  reportMotorsInTheWrongList();
  const std::int32_t power = percentToPower(percent);
  for (std::size_t i = 0; i < m_motors.size(); ++i) {
    if (m_listed_5_5w[i]) {
      m_motors[i].move_voltage(voltsToMillivolts(voltsFor5_5WMotor(power / 127.0 * 12.0)));
    } else {
      m_motors[i].move(power);
    }
  }
}

void MotorGroup::stop() {
  brake();
}

void MotorGroup::brake() {
  for (const pros::Motor& motor : m_motors) motor.brake();
}

void MotorGroup::setBrakeMode(BrakeMode mode) {
  for (const pros::Motor& motor : m_motors) motor.set_brake_mode(toProsBrakeMode(mode));
}

void MotorGroup::tarePosition() {
  for (const pros::Motor& motor : m_motors) motor.tare_position();
}

std::vector<double> MotorGroup::readEach(
    double (pros::Motor::*read)(std::uint8_t) const) const {
  std::vector<double> values;
  values.reserve(m_motors.size());
  for (std::size_t i = 0; i < m_motors.size(); ++i) {
    const double value = (m_motors[i].*read)(0);
    values.push_back(m_listed_5_5w[i] && isReading(value) ? value * m_5_5w_reading_scale
                                                           : value);
  }
  return values;
}

std::vector<double> MotorGroup::getPositionsDeg() const {
  return readEach(&pros::Motor::get_position);
}

std::vector<double> MotorGroup::getActualVelocities() const {
  return readEach(&pros::Motor::get_actual_velocity);
}

std::vector<double> MotorGroup::getCurrentDraws() const {
  std::vector<double> draws;
  draws.reserve(m_motors.size());
  for (const pros::Motor& motor : m_motors) draws.push_back(motor.get_current_draw());
  return draws;
}

std::vector<double> MotorGroup::getTemperatures() const {
  std::vector<double> temperatures;
  temperatures.reserve(m_motors.size());
  for (const pros::Motor& motor : m_motors) temperatures.push_back(motor.get_temperature());
  return temperatures;
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
  // Round, don't truncate: 0.999 is 127, not 126.
  return static_cast<std::int32_t>(std::lround(std::clamp(percent, -1.0, 1.0) * 127.0));
}

void MotorGroup::reportMotorsInTheWrongList() const {
  if (m_types_checked.load(std::memory_order_relaxed)) return;
  std::vector<pros::MotorType> types;
  for (const pros::Motor& motor : m_motors) {
    types.push_back(motor.get_type());
    if (types.back() == pros::MotorType::invalid) return;
  }
  if (m_types_checked.exchange(true)) return;
  const bool has_11w = std::find(types.begin(), types.end(), pros::MotorType::v5) != types.end();
  const bool lists_5_5w = std::find(m_listed_5_5w.begin(), m_listed_5_5w.end(), true) !=
                          m_listed_5_5w.end();
  for (std::size_t i = 0; i < m_motors.size(); ++i) {
    const int port = std::abs(m_motors[i].get_port());
    if (m_listed_5_5w[i] && types[i] == pros::MotorType::v5) {
      std::printf("mclib: the motor on port %d is an 11 W motor, listed with the 5.5 W "
                  "motors. Its voltage is cut to %.0f%%. List it with the 11 W motors.\n",
                  port, kMotor5_5WFullVolts / 12.0 * 100.0);
    } else if (!m_listed_5_5w[i] && types[i] == pros::MotorType::exp && (has_11w || lists_5_5w)) {
      std::printf("mclib: the motor on port %d is a 5.5 W motor, listed with the 11 W motors. "
                  "Its position and speed are on a different shaft from theirs. List it with "
                  "the 5.5 W motors.\n",
                  port);
    }
  }
}

double MotorGroup::average(const std::vector<double>& values) {
  // Skip motors that are not reporting: one unplugged motor used to make the
  // average infinite, or add ~2e9 / n mA to the current. NaN when none are.
  double total = 0.0;
  int counted = 0;
  for (const double value : values) {
    if (!isReading(value)) continue;
    total += value;
    ++counted;
  }
  return counted > 0 ? total / counted : NAN;
}

}  // namespace device
}  // namespace mclib
