// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host stand-in for pros::MotorGroup, which on the robot comes out of libpros.
// With it the real src/mclib/device/motor_group.cpp links on the host, so a
// test can build any mechanism that holds a device::MotorGroup by value.
//
// device::MotorGroup holds a pros::MotorGroup member, and that class has a
// virtual base. Its inline destructor needs the vtable, and the vtable needs
// every virtual function defined, so all of them are here. Only the ones
// device::MotorGroup uses for voltage, braking, speed and current do anything;
// the rest return zero or an empty vector.
//
// State lives in mclib::test::host_motor_group::group, keyed by the group's
// first port, so a test can set the speed and current a group reports and read
// back the voltage it was sent. Listed in HOST_TEST_SRC, so every test binary
// links it.
#include "host_motor_group.hpp"

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <cstdint>
#include <map>
#include <vector>

namespace mclib {
namespace test {
namespace host_motor_group {
std::map<std::int8_t, Group> group;
void reset() { group.clear(); }
}  // namespace host_motor_group
}  // namespace test
}  // namespace mclib

namespace {
mclib::test::host_motor_group::Group& stateFor(const std::vector<std::int8_t>& ports) {
  const std::int8_t key = ports.empty() ? 0 : ports.front();
  auto& entry = mclib::test::host_motor_group::group[key];
  entry.ports = ports;
  return entry;
}
}  // namespace

namespace pros {
inline namespace rtos {
// pros::MotorGroup holds a pros::Mutex. On the host it is never locked.
mutex_t Mutex::lazy_init() { return nullptr; }
Mutex::~Mutex() {}
}  // namespace rtos

inline namespace v5 {

MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports,
                       const MotorGears, const MotorUnits)
    : _ports(ports) {
  stateFor(_ports);
}

MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports, const MotorGears,
                       const MotorUnits)
    : _ports(ports) {
  stateFor(_ports);
}

std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
  auto& state = stateFor(_ports);
  state.volts = static_cast<double>(voltage) / 1000.0;
  state.braked = false;
  ++state.voltage_writes;
  return 1;
}

std::int32_t MotorGroup::brake(void) const {
  auto& state = stateFor(_ports);
  state.volts = 0.0;
  state.braked = true;
  ++state.brakes;
  return 1;
}

std::vector<double> MotorGroup::get_actual_velocity_all(void) const {
  return std::vector<double>(_ports.size(), stateFor(_ports).rpm);
}

std::vector<std::int32_t> MotorGroup::get_current_draw_all(void) const {
  return std::vector<std::int32_t>(_ports.size(), stateFor(_ports).current_ma);
}

std::vector<std::int8_t> MotorGroup::get_port_all(void) const { return _ports; }

std::int8_t MotorGroup::size(void) const {
  return static_cast<std::int8_t>(_ports.size());
}

// Everything below exists only so the vtable links.
std::int32_t MotorGroup::move(std::int32_t) const { return {}; }
std::int32_t MotorGroup::move_absolute(const double, const std::int32_t) const { return {}; }
std::int32_t MotorGroup::move_relative(const double, const std::int32_t) const { return {}; }
std::int32_t MotorGroup::move_velocity(const std::int32_t) const { return {}; }
std::int32_t MotorGroup::modify_profiled_velocity(const std::int32_t) const { return {}; }
double MotorGroup::get_target_position(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_target_position_all(void) const { return {}; }
std::int32_t MotorGroup::get_target_velocity(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_target_velocity_all(void) const { return {}; }
double MotorGroup::get_actual_velocity(const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::get_current_draw(const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::get_direction(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_direction_all(void) const { return {}; }
double MotorGroup::get_efficiency(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_efficiency_all(void) const { return {}; }
std::uint32_t MotorGroup::get_faults(const std::uint8_t) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_faults_all(void) const { return {}; }
std::uint32_t MotorGroup::get_flags(const std::uint8_t) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_flags_all(void) const { return {}; }
double MotorGroup::get_position(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_position_all(void) const { return {}; }
double MotorGroup::get_power(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_power_all(void) const { return {}; }
std::int32_t MotorGroup::get_raw_position(std::uint32_t* const, const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_raw_position_all(std::uint32_t* const) const { return {}; }
double MotorGroup::get_temperature(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_temperature_all(void) const { return {}; }
double MotorGroup::get_torque(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_torque_all(void) const { return {}; }
std::int32_t MotorGroup::get_voltage(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_all(void) const { return {}; }
std::int32_t MotorGroup::is_over_current(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_over_current_all(void) const { return {}; }
std::int32_t MotorGroup::is_over_temp(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_over_temp_all(void) const { return {}; }
MotorBrake MotorGroup::get_brake_mode(const std::uint8_t) const { return {}; }
std::vector<MotorBrake> MotorGroup::get_brake_mode_all(void) const { return {}; }
std::int32_t MotorGroup::get_current_limit(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_current_limit_all(void) const { return {}; }
MotorUnits MotorGroup::get_encoder_units(const std::uint8_t) const { return {}; }
std::vector<MotorUnits> MotorGroup::get_encoder_units_all(void) const { return {}; }
MotorGears MotorGroup::get_gearing(const std::uint8_t) const { return {}; }
std::vector<MotorGears> MotorGroup::get_gearing_all(void) const { return {}; }
std::int32_t MotorGroup::get_voltage_limit(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_limit_all(void) const { return {}; }
std::int32_t MotorGroup::is_reversed(const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_reversed_all(void) const { return {}; }
MotorType MotorGroup::get_type(const std::uint8_t) const { return {}; }
std::vector<MotorType> MotorGroup::get_type_all(void) const { return {}; }
std::int32_t MotorGroup::set_brake_mode(const MotorBrake, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake) const { return {}; }
std::int32_t MotorGroup::set_brake_mode_all(const pros::motor_brake_mode_e_t) const { return {}; }
std::int32_t MotorGroup::set_current_limit(const std::int32_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_current_limit_all(const std::int32_t) const { return {}; }
std::int32_t MotorGroup::set_encoder_units(const MotorUnits, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_encoder_units(const pros::motor_encoder_units_e_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits) const { return {}; }
std::int32_t MotorGroup::set_encoder_units_all(const pros::motor_encoder_units_e_t) const { return {}; }
std::int32_t MotorGroup::set_gearing(const MotorGears, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_gearing(const pros::motor_gearset_e_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const MotorGears) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const pros::motor_gearset_e_t) const { return {}; }
std::int32_t MotorGroup::set_reversed(const bool, const std::uint8_t) { return {}; }
std::int32_t MotorGroup::set_reversed_all(const bool) { return {}; }
std::int32_t MotorGroup::set_voltage_limit(const std::int32_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_voltage_limit_all(const std::int32_t) const { return {}; }
std::int32_t MotorGroup::set_zero_position(const double, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_zero_position_all(const double) const { return {}; }
std::int32_t MotorGroup::tare_position(const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::tare_position_all(void) const { return {}; }
std::int8_t MotorGroup::get_port(const std::uint8_t) const { return {}; }

}  // namespace v5
}  // namespace pros
