// mclib
#include "mclib/device/motor.hpp"

#include <algorithm>

namespace mclib {
namespace device {

Motor::Motor(std::int8_t port, Gearset gearset)
    : m_motor(port, toProsGearset(gearset)) {}

void Motor::setVoltage(double volts) {
  m_motor.move_voltage(voltsToMillivolts(volts));
}

void Motor::setPercent(double percent) {
  m_motor.move(percentToPower(percent));
}

void Motor::stop() {
  m_motor.brake();
}

void Motor::setBrakeMode(BrakeMode mode) {
  m_motor.set_brake_mode(toProsBrakeMode(mode));
}

void Motor::tarePosition() {
  m_motor.tare_position();
}

double Motor::getPositionDeg() const {
  return m_motor.get_position();
}

double Motor::getActualVelocity() const {
  return m_motor.get_actual_velocity();
}

double Motor::getCurrentDraw() const {
  return m_motor.get_current_draw();
}

double Motor::getTemperature() const {
  return m_motor.get_temperature();
}

std::int32_t Motor::voltsToMillivolts(double volts) {
  return static_cast<std::int32_t>(std::clamp(volts, -12.0, 12.0) * 1000.0);
}

std::int32_t Motor::percentToPower(double percent) {
  return static_cast<std::int32_t>(std::clamp(percent, -1.0, 1.0) * 127.0);
}

}  // namespace device
}  // namespace mclib
