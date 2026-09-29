// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host stand-in for the libpros motor classes, so the real
// src/mclib/device/motor.cpp and motor_group.cpp link on the host.
//
// device::Motor and device::MotorGroup hold a pros::Motor and a
// pros::MotorGroup by value, so a test cannot build one without constructing
// the PROS object. pros::Motor and pros::MotorGroup have virtual functions, and
// their vtables need every one of them defined. This file defines all of them.
// The calls mclib uses to drive a motor (move, move_voltage, brake) record the
// command in mclib::test::host_motor::ports. Every other call does nothing and
// returns a zero value.
//
// pros::Mutex is here too because pros::MotorGroup holds one.
//
// The definitions were generated from the member declarations in
// include/pros/motors.hpp and include/pros/motor_group.hpp. If a PROS update
// adds a virtual member, the link fails with an undefined vtable entry: add
// the missing definition here.
#include "host_motor.hpp"

#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <cstdlib>
#include <map>

namespace mclib {
namespace test {
namespace host_motor {
std::map<int, Port> ports;
void reset() { ports.clear(); }
}  // namespace host_motor
}  // namespace test
}  // namespace mclib

namespace {
mclib::test::host_motor::Port& record(std::int8_t port) {
  return mclib::test::host_motor::ports[std::abs(static_cast<int>(port))];
}
}  // namespace

// Only the recorded calls read their arguments.
#pragma GCC diagnostic ignored "-Wunused-parameter"

namespace pros {
inline namespace rtos {
mutex_t Mutex::lazy_init() { return nullptr; }
Mutex::~Mutex() {}
}  // namespace rtos

inline namespace v5 {

bool Device::is_installed() { return true; }

Motor::Motor(const std::int8_t port, const MotorGears gearset,
             const MotorUnits encoder_units)
    : Device(static_cast<std::uint8_t>(std::abs(port)), DeviceType::motor),
      _port(port) {}

MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports,
                       const MotorGears gearset, const MotorUnits encoder_units)
    : _ports(ports) {}

MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports,
                       const MotorGears gearset, const MotorUnits encoder_units)
    : _ports(ports) {}

MotorGroup::MotorGroup(AbstractMotor& motor_group)
    : _ports(motor_group.get_port_all()) {}

std::int32_t Motor::move(std::int32_t voltage) const {
  record(_port).power = voltage;
  ++record(_port).power_writes;
  return 1;
}
std::int32_t Motor::move_absolute(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_relative(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_velocity(const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_voltage(const std::int32_t voltage) const {
  record(_port).millivolts = voltage;
  ++record(_port).voltage_writes;
  return 1;
}
std::int32_t Motor::brake(void) const {
  ++record(_port).brakes;
  return 1;
}
std::int32_t Motor::modify_profiled_velocity(const std::int32_t velocity) const { return {}; }
double Motor::get_target_position(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_target_velocity(const std::uint8_t index) const { return {}; }
double Motor::get_actual_velocity(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_current_draw(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_direction(const std::uint8_t index) const { return {}; }
double Motor::get_efficiency(const std::uint8_t index) const { return {}; }
std::uint32_t Motor::get_faults(const std::uint8_t index) const { return {}; }
std::uint32_t Motor::get_flags(const std::uint8_t index) const { return {}; }
double Motor::get_position(const std::uint8_t index) const { return {}; }
double Motor::get_power(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_raw_position(std::uint32_t* const timestamp, const std::uint8_t index) const { return {}; }
double Motor::get_temperature(const std::uint8_t index) const { return {}; }
double Motor::get_torque(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_voltage(const std::uint8_t index) const { return {}; }
std::int32_t Motor::is_over_current(const std::uint8_t index) const { return {}; }
std::int32_t Motor::is_over_temp(const std::uint8_t index) const { return {}; }
MotorBrake Motor::get_brake_mode(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_current_limit(const std::uint8_t index) const { return {}; }
MotorUnits Motor::get_encoder_units(const std::uint8_t index) const { return {}; }
MotorGears Motor::get_gearing(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_voltage_limit(const std::uint8_t index) const { return {}; }
std::int32_t Motor::is_reversed(const std::uint8_t index) const { return {}; }
MotorType Motor::get_type(const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_current_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_encoder_units(const MotorUnits units, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_gearing(const MotorGears gearset, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_gearing(const pros::motor_gearset_e_t gearset, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_reversed(const bool reverse, const std::uint8_t index) { return {}; }
std::int32_t Motor::set_voltage_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_zero_position(const double position, const std::uint8_t index) const { return {}; }
std::int32_t Motor::tare_position(const std::uint8_t index) const { return {}; }
std::int8_t Motor::size(void) const {
  return 1;
}
std::vector<Motor> Motor::get_all_devices() { return {}; }
std::int8_t Motor::get_port(const std::uint8_t index) const {
  return _port;
}
std::vector<double> Motor::get_target_position_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_target_velocity_all(void) const { return {}; }
std::vector<double> Motor::get_actual_velocity_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_current_draw_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_direction_all(void) const { return {}; }
std::vector<double> Motor::get_efficiency_all(void) const { return {}; }
std::vector<std::uint32_t> Motor::get_faults_all(void) const { return {}; }
std::vector<std::uint32_t> Motor::get_flags_all(void) const { return {}; }
std::vector<double> Motor::get_position_all(void) const { return {}; }
std::vector<double> Motor::get_power_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_raw_position_all(std::uint32_t* const timestamp) const { return {}; }
std::vector<double> Motor::get_temperature_all(void) const { return {}; }
std::vector<double> Motor::get_torque_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_voltage_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_over_current_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_over_temp_all(void) const { return {}; }
std::vector<MotorBrake> Motor::get_brake_mode_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_current_limit_all(void) const { return {}; }
std::vector<MotorUnits> Motor::get_encoder_units_all(void) const { return {}; }
std::vector<MotorGears> Motor::get_gearing_all(void) const { return {}; }
std::vector<std::int8_t> Motor::get_port_all(void) const {
  return {_port};
}
std::vector<std::int32_t> Motor::get_voltage_limit_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_reversed_all(void) const { return {}; }
std::vector<MotorType> Motor::get_type_all(void) const { return {}; }
std::int32_t Motor::set_brake_mode_all(const MotorBrake mode) const { return {}; }
std::int32_t Motor::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const { return {}; }
std::int32_t Motor::set_current_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t Motor::set_encoder_units_all(const MotorUnits units) const { return {}; }
std::int32_t Motor::set_encoder_units_all(const pros::motor_encoder_units_e_t units) const { return {}; }
std::int32_t Motor::set_gearing_all(const MotorGears gearset) const { return {}; }
std::int32_t Motor::set_gearing_all(const pros::motor_gearset_e_t gearset) const { return {}; }
std::int32_t Motor::set_reversed_all(const bool reverse) { return {}; }
std::int32_t Motor::set_voltage_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t Motor::set_zero_position_all(const double position) const { return {}; }
std::int32_t Motor::tare_position_all(void) const { return {}; }
std::int32_t MotorGroup::move(std::int32_t voltage) const {
  for (const std::int8_t port : _ports) {
    record(port).power = voltage;
    ++record(port).power_writes;
  }
  return 1;
}
std::int32_t MotorGroup::move_absolute(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::move_relative(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::move_velocity(const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
  for (const std::int8_t port : _ports) {
    record(port).millivolts = voltage;
    ++record(port).voltage_writes;
  }
  return 1;
}
std::int32_t MotorGroup::brake(void) const {
  for (const std::int8_t port : _ports) {
    ++record(port).brakes;
  }
  return 1;
}
std::int32_t MotorGroup::modify_profiled_velocity(const std::int32_t velocity) const { return {}; }
double MotorGroup::get_target_position(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_target_position_all(void) const { return {}; }
std::int32_t MotorGroup::get_target_velocity(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_target_velocity_all(void) const { return {}; }
double MotorGroup::get_actual_velocity(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_actual_velocity_all(void) const { return {}; }
std::int32_t MotorGroup::get_current_draw(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_current_draw_all(void) const { return {}; }
std::int32_t MotorGroup::get_direction(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_direction_all(void) const { return {}; }
double MotorGroup::get_efficiency(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_efficiency_all(void) const { return {}; }
std::uint32_t MotorGroup::get_faults(const std::uint8_t index) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_faults_all(void) const { return {}; }
std::uint32_t MotorGroup::get_flags(const std::uint8_t index) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_flags_all(void) const { return {}; }
double MotorGroup::get_position(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_position_all(void) const { return {}; }
double MotorGroup::get_power(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_power_all(void) const { return {}; }
std::int32_t MotorGroup::get_raw_position(std::uint32_t* const timestamp, const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_raw_position_all(std::uint32_t* const timestamp) const { return {}; }
double MotorGroup::get_temperature(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_temperature_all(void) const { return {}; }
double MotorGroup::get_torque(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_torque_all(void) const { return {}; }
std::int32_t MotorGroup::get_voltage(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_all(void) const { return {}; }
std::int32_t MotorGroup::is_over_current(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_over_current_all(void) const { return {}; }
std::int32_t MotorGroup::is_over_temp(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_over_temp_all(void) const { return {}; }
MotorBrake MotorGroup::get_brake_mode(const std::uint8_t index) const { return {}; }
std::vector<MotorBrake> MotorGroup::get_brake_mode_all(void) const { return {}; }
std::int32_t MotorGroup::get_current_limit(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_current_limit_all(void) const { return {}; }
MotorUnits MotorGroup::get_encoder_units(const std::uint8_t index) const { return {}; }
std::vector<MotorUnits> MotorGroup::get_encoder_units_all(void) const { return {}; }
MotorGears MotorGroup::get_gearing(const std::uint8_t index) const { return {}; }
std::vector<MotorGears> MotorGroup::get_gearing_all(void) const { return {}; }
std::vector<std::int8_t> MotorGroup::get_port_all(void) const {
  return _ports;
}
std::int32_t MotorGroup::get_voltage_limit(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_limit_all(void) const { return {}; }
std::int32_t MotorGroup::is_reversed(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_reversed_all(void) const { return {}; }
MotorType MotorGroup::get_type(const std::uint8_t index) const { return {}; }
std::vector<MotorType> MotorGroup::get_type_all(void) const { return {}; }
std::int32_t MotorGroup::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake mode) const { return {}; }
std::int32_t MotorGroup::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const { return {}; }
std::int32_t MotorGroup::set_current_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_current_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t MotorGroup::set_encoder_units(const MotorUnits units, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits units) const { return {}; }
std::int32_t MotorGroup::set_encoder_units_all(const pros::motor_encoder_units_e_t units) const { return {}; }
std::int32_t MotorGroup::set_gearing(std::vector<pros::motor_gearset_e_t> gearsets) const { return {}; }
std::int32_t MotorGroup::set_gearing(const pros::motor_gearset_e_t gearset, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_gearing(std::vector<MotorGears> gearsets) const { return {}; }
std::int32_t MotorGroup::set_gearing(const MotorGears gearset, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const MotorGears gearset) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const pros::motor_gearset_e_t gearset) const { return {}; }
std::int32_t MotorGroup::set_reversed(const bool reverse, const std::uint8_t index) { return {}; }
std::int32_t MotorGroup::set_reversed_all(const bool reverse) { return {}; }
std::int32_t MotorGroup::set_voltage_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_voltage_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t MotorGroup::set_zero_position(const double position, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_zero_position_all(const double position) const { return {}; }
std::int32_t MotorGroup::tare_position(const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::tare_position_all(void) const { return {}; }
std::int8_t MotorGroup::size(void) const {
  return static_cast<std::int8_t>(_ports.size());
}
std::int8_t MotorGroup::get_port(const std::uint8_t index) const { return {}; }
void MotorGroup::append(AbstractMotor&) {}
void MotorGroup::erase_port(std::int8_t port) {}

}  // namespace v5
}  // namespace pros
