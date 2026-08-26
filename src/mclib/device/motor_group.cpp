// mclib
#include "mclib/device/motor_group.hpp"

#include <algorithm>
#include <utility>

namespace mclib {
namespace device {

MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports,
                       Gearset gearset)
    : MotorGroup(std::vector<std::int8_t>(ports), gearset) {}

MotorGroup::MotorGroup(std::vector<std::int8_t> ports, Gearset gearset)
    : m_motors(std::move(ports), toProsGearset(gearset)) {}

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
  m_motors.tare_position();
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

std::int32_t MotorGroup::voltsToMillivolts(double volts) {
  return static_cast<std::int32_t>(std::clamp(volts, -12.0, 12.0) * 1000.0);
}

std::int32_t MotorGroup::percentToPower(double percent) {
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
