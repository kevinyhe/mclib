// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host stand-ins for the libpros device classes mclib's wrappers call:
// pros::Motor, pros::MotorGroup, pros::Imu, pros::Controller and pros::Mutex.
// With these, the real src/mclib/device/*.cpp link on the host, and so does
// any library code built on them. This is the only place in tests/ that
// defines these classes; a test that needs a new call adds it here.
//
// The calls mclib makes write or read the state in host_devices.hpp. A motor
// and every motor in a group record into the same per-port entry, so a test
// reads a group port by port.
//
// pros::Motor, pros::MotorGroup and pros::Imu have virtual methods, and their
// vtables need every one of them defined. The ones mclib does not call return
// a value-initialised result; they are listed only so the vtables link. They
// follow the member declarations in include/pros/motors.hpp,
// motor_group.hpp and imu.hpp. If a PROS update adds a virtual member, the
// link fails with an undefined vtable entry: add the missing definition here.
//
// Listed in HOST_TEST_SRC, so every test binary links it. It must never
// define pros::millis() or pros::delay(): motion_safety_test defines its own,
// and auton_selector_test gets pros::millis() from tests/support/host_screen.cpp.
#include "support/host_devices.hpp"

#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace mclib {
namespace test {
namespace host_devices {
std::map<int, Motor> motors;
std::map<int, double> imu_rotation_deg;
std::map<int, int> analog;
std::map<int, bool> button_down;
std::vector<std::string> controller_text;
void reset() {
  motors.clear();
  imu_rotation_deg.clear();
  analog.clear();
  button_down.clear();
  controller_text.clear();
}
}  // namespace host_devices
}  // namespace test
}  // namespace mclib

namespace {
namespace hd = mclib::test::host_devices;

hd::Motor& motorAt(std::int8_t port) {
  return hd::motors[std::abs(static_cast<int>(port))];
}

// Called from each constructor: creates the port's entry and notes whether
// it was built reversed.
void attach(std::int8_t port) {
  motorAt(port).reversed = port < 0;
}

void recordMillivolts(std::int8_t port, double millivolts) {
  hd::Motor& m = motorAt(port);
  m.millivolts = millivolts;
  m.braked = false;
  ++m.voltage_writes;
}

void recordBrake(std::int8_t port) {
  hd::Motor& m = motorAt(port);
  m.millivolts = 0.0;
  m.braked = true;
  ++m.brakes;
}

double powerToMillivolts(std::int32_t power) {
  return static_cast<double>(power) * 12000.0 / 127.0;
}
}  // namespace

// Only the recorded calls read their arguments.
#pragma GCC diagnostic ignored "-Wunused-parameter"

namespace pros {

// No RTOS on the host. pros::MotorGroup holds a Mutex that the stand-ins
// never lock, so it only has to construct and destroy.
inline namespace rtos {
mutex_t Mutex::lazy_init() { return nullptr; }
Mutex::~Mutex() {}
}  // namespace rtos

inline namespace v5 {

Device::Device(const std::uint8_t port) : _port(port) {}
std::uint8_t Device::get_port(void) const { return _port; }
bool Device::is_installed() { return true; }

// --------------------------------------------------------------------- Motor

Motor::Motor(const std::int8_t port, const MotorGears gearset, const MotorUnits encoder_units)
    : Device(static_cast<std::uint8_t>(std::abs(port)), DeviceType::motor), _port(port) {
  attach(_port);
}

std::int32_t Motor::move(std::int32_t voltage) const {
  recordMillivolts(_port, powerToMillivolts(voltage));
  return 1;
}
std::int32_t Motor::move_voltage(const std::int32_t voltage) const {
  recordMillivolts(_port, static_cast<double>(voltage));
  return 1;
}
std::int32_t Motor::brake(void) const {
  recordBrake(_port);
  return 1;
}
std::int32_t Motor::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const {
  motorAt(_port).brake_mode = static_cast<int>(mode);
  return 1;
}
std::int32_t Motor::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const {
  return set_brake_mode(static_cast<MotorBrake>(mode), index);
}
std::int32_t Motor::set_brake_mode_all(const MotorBrake mode) const { return set_brake_mode(mode, 0); }
std::int32_t Motor::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const {
  return set_brake_mode(mode, 0);
}
std::int32_t Motor::tare_position(const std::uint8_t index) const {
  motorAt(_port).position_deg = 0.0;
  return 1;
}
std::int32_t Motor::tare_position_all(void) const { return tare_position(0); }
double Motor::get_position(const std::uint8_t index) const { return motorAt(_port).position_deg; }
std::vector<double> Motor::get_position_all(void) const { return {get_position(0)}; }
double Motor::get_actual_velocity(const std::uint8_t index) const { return motorAt(_port).rpm; }
std::vector<double> Motor::get_actual_velocity_all(void) const { return {get_actual_velocity(0)}; }
std::int32_t Motor::get_current_draw(const std::uint8_t index) const { return motorAt(_port).current_ma; }
std::vector<std::int32_t> Motor::get_current_draw_all(void) const { return {get_current_draw(0)}; }
double Motor::get_temperature(const std::uint8_t index) const { return 0.0; }
std::vector<double> Motor::get_temperature_all(void) const { return {0.0}; }
std::int8_t Motor::size(void) const { return 1; }
std::int8_t Motor::get_port(const std::uint8_t index) const { return _port; }
std::vector<std::int8_t> Motor::get_port_all(void) const { return {_port}; }

// ---------------------------------------------------------------- MotorGroup

MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports, const MotorGears gearset,
                       const MotorUnits encoder_units)
    : MotorGroup(std::vector<std::int8_t>(ports), gearset, encoder_units) {}

MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports, const MotorGears gearset,
                       const MotorUnits encoder_units)
    : _ports(ports) {
  for (const std::int8_t port : _ports) {
    attach(port);
  }
}

MotorGroup::MotorGroup(AbstractMotor& motor_group) : _ports(motor_group.get_port_all()) {
  for (const std::int8_t port : _ports) {
    attach(port);
  }
}

std::int32_t MotorGroup::move(std::int32_t voltage) const {
  for (const std::int8_t port : _ports) {
    recordMillivolts(port, powerToMillivolts(voltage));
  }
  return 1;
}
std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
  for (const std::int8_t port : _ports) {
    recordMillivolts(port, static_cast<double>(voltage));
  }
  return 1;
}
std::int32_t MotorGroup::brake(void) const {
  for (const std::int8_t port : _ports) {
    recordBrake(port);
  }
  return 1;
}
std::int32_t MotorGroup::set_brake_mode_all(const MotorBrake mode) const {
  for (const std::int8_t port : _ports) {
    motorAt(port).brake_mode = static_cast<int>(mode);
  }
  return 1;
}
std::int32_t MotorGroup::set_brake_mode_all(const pros::motor_brake_mode_e_t mode) const {
  return set_brake_mode_all(static_cast<MotorBrake>(mode));
}
std::int32_t MotorGroup::tare_position_all(void) const {
  for (const std::int8_t port : _ports) {
    motorAt(port).position_deg = 0.0;
  }
  return 1;
}
std::vector<double> MotorGroup::get_position_all(void) const {
  std::vector<double> out;
  for (const std::int8_t port : _ports) {
    out.push_back(motorAt(port).position_deg);
  }
  return out;
}
std::vector<double> MotorGroup::get_actual_velocity_all(void) const {
  std::vector<double> out;
  for (const std::int8_t port : _ports) {
    out.push_back(motorAt(port).rpm);
  }
  return out;
}
std::vector<std::int32_t> MotorGroup::get_current_draw_all(void) const {
  std::vector<std::int32_t> out;
  for (const std::int8_t port : _ports) {
    out.push_back(motorAt(port).current_ma);
  }
  return out;
}
std::vector<double> MotorGroup::get_temperature_all(void) const {
  return std::vector<double>(_ports.size(), 0.0);
}
std::vector<std::int8_t> MotorGroup::get_port_all(void) const { return _ports; }
std::int8_t MotorGroup::size(void) const { return static_cast<std::int8_t>(_ports.size()); }

// ----------------------------------------------------------------------- Imu

std::int32_t Imu::reset(bool blocking) const {
  hd::imu_rotation_deg[_port] = 0.0;
  return 1;
}
double Imu::get_rotation() const { return hd::imu_rotation_deg[_port]; }
double Imu::get_heading() const {
  const double wrapped = std::fmod(hd::imu_rotation_deg[_port], 360.0);
  return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}
std::int32_t Imu::set_rotation(const double target) const {
  hd::imu_rotation_deg[_port] = target;
  return 1;
}

// ---------------------------------------------------------------- Controller

Controller::Controller(controller_id_e_t id) : _id(id) {}

std::int32_t Controller::get_analog(controller_analog_e_t channel) {
  const auto found = hd::analog.find(static_cast<int>(channel));
  return found != hd::analog.end() ? found->second : 0;
}

std::int32_t Controller::get_digital(controller_digital_e_t button) {
  const auto found = hd::button_down.find(static_cast<int>(button));
  return found != hd::button_down.end() && found->second ? 1 : 0;
}

std::int32_t Controller::set_text(std::uint8_t line, std::uint8_t col, const char* str) {
  hd::controller_text.emplace_back(str);
  return 1;
}

std::int32_t Controller::set_text(std::uint8_t line, std::uint8_t col, const std::string& str) {
  return set_text(line, col, str.c_str());
}

// ---------------------------------------------- Unused, present for vtables

std::int32_t Motor::move_absolute(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_relative(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t Motor::move_velocity(const std::int32_t velocity) const { return {}; }
std::int32_t Motor::modify_profiled_velocity(const std::int32_t velocity) const { return {}; }
double Motor::get_target_position(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_target_velocity(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_direction(const std::uint8_t index) const { return {}; }
double Motor::get_efficiency(const std::uint8_t index) const { return {}; }
std::uint32_t Motor::get_faults(const std::uint8_t index) const { return {}; }
std::uint32_t Motor::get_flags(const std::uint8_t index) const { return {}; }
double Motor::get_power(const std::uint8_t index) const { return {}; }
std::int32_t Motor::get_raw_position(std::uint32_t* const timestamp, const std::uint8_t index) const { return {}; }
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
std::int32_t Motor::set_current_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_encoder_units(const MotorUnits units, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_encoder_units(const pros::motor_encoder_units_e_t units, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_gearing(const MotorGears gearset, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_gearing(const pros::motor_gearset_e_t gearset, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_reversed(const bool reverse, const std::uint8_t index) { return {}; }
std::int32_t Motor::set_voltage_limit(const std::int32_t limit, const std::uint8_t index) const { return {}; }
std::int32_t Motor::set_zero_position(const double position, const std::uint8_t index) const { return {}; }
std::vector<Motor> Motor::get_all_devices() { return {}; }
std::vector<double> Motor::get_target_position_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_target_velocity_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_direction_all(void) const { return {}; }
std::vector<double> Motor::get_efficiency_all(void) const { return {}; }
std::vector<std::uint32_t> Motor::get_faults_all(void) const { return {}; }
std::vector<std::uint32_t> Motor::get_flags_all(void) const { return {}; }
std::vector<double> Motor::get_power_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_raw_position_all(std::uint32_t* const timestamp) const { return {}; }
std::vector<double> Motor::get_torque_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_voltage_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_over_current_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_over_temp_all(void) const { return {}; }
std::vector<MotorBrake> Motor::get_brake_mode_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_current_limit_all(void) const { return {}; }
std::vector<MotorUnits> Motor::get_encoder_units_all(void) const { return {}; }
std::vector<MotorGears> Motor::get_gearing_all(void) const { return {}; }
std::vector<std::int32_t> Motor::get_voltage_limit_all(void) const { return {}; }
std::vector<std::int32_t> Motor::is_reversed_all(void) const { return {}; }
std::vector<MotorType> Motor::get_type_all(void) const { return {}; }
std::int32_t Motor::set_current_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t Motor::set_encoder_units_all(const MotorUnits units) const { return {}; }
std::int32_t Motor::set_encoder_units_all(const pros::motor_encoder_units_e_t units) const { return {}; }
std::int32_t Motor::set_gearing_all(const MotorGears gearset) const { return {}; }
std::int32_t Motor::set_gearing_all(const pros::motor_gearset_e_t gearset) const { return {}; }
std::int32_t Motor::set_reversed_all(const bool reverse) { return {}; }
std::int32_t Motor::set_voltage_limit_all(const std::int32_t limit) const { return {}; }
std::int32_t Motor::set_zero_position_all(const double position) const { return {}; }
std::int32_t MotorGroup::move_absolute(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::move_relative(const double position, const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::move_velocity(const std::int32_t velocity) const { return {}; }
std::int32_t MotorGroup::modify_profiled_velocity(const std::int32_t velocity) const { return {}; }
double MotorGroup::get_target_position(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_target_position_all(void) const { return {}; }
std::int32_t MotorGroup::get_target_velocity(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_target_velocity_all(void) const { return {}; }
double MotorGroup::get_actual_velocity(const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::get_current_draw(const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::get_direction(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_direction_all(void) const { return {}; }
double MotorGroup::get_efficiency(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_efficiency_all(void) const { return {}; }
std::uint32_t MotorGroup::get_faults(const std::uint8_t index) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_faults_all(void) const { return {}; }
std::uint32_t MotorGroup::get_flags(const std::uint8_t index) const { return {}; }
std::vector<std::uint32_t> MotorGroup::get_flags_all(void) const { return {}; }
double MotorGroup::get_position(const std::uint8_t index) const { return {}; }
double MotorGroup::get_power(const std::uint8_t index) const { return {}; }
std::vector<double> MotorGroup::get_power_all(void) const { return {}; }
std::int32_t MotorGroup::get_raw_position(std::uint32_t* const timestamp, const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_raw_position_all(std::uint32_t* const timestamp) const { return {}; }
double MotorGroup::get_temperature(const std::uint8_t index) const { return {}; }
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
std::int32_t MotorGroup::get_voltage_limit(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_voltage_limit_all(void) const { return {}; }
std::int32_t MotorGroup::is_reversed(const std::uint8_t index) const { return {}; }
std::vector<std::int32_t> MotorGroup::is_reversed_all(void) const { return {}; }
MotorType MotorGroup::get_type(const std::uint8_t index) const { return {}; }
std::vector<MotorType> MotorGroup::get_type_all(void) const { return {}; }
std::int32_t MotorGroup::set_brake_mode(const MotorBrake mode, const std::uint8_t index) const { return {}; }
std::int32_t MotorGroup::set_brake_mode(const pros::motor_brake_mode_e_t mode, const std::uint8_t index) const { return {}; }
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
std::int8_t MotorGroup::get_port(const std::uint8_t index) const { return {}; }
void MotorGroup::append(AbstractMotor&) {}
void MotorGroup::erase_port(std::int8_t port) {}
void MotorGroup::operator+=(AbstractMotor&) {}
std::int32_t Imu::set_data_rate(std::uint32_t) const { return {}; }
pros::quaternion_s_t Imu::get_quaternion() const { return {}; }
pros::euler_s_t Imu::get_euler() const { return {}; }
double Imu::get_pitch() const { return {}; }
double Imu::get_roll() const { return {}; }
double Imu::get_yaw() const { return {}; }
pros::imu_gyro_s_t Imu::get_gyro_rate() const { return {}; }
std::int32_t Imu::tare_rotation() const { return {}; }
std::int32_t Imu::tare_heading() const { return {}; }
std::int32_t Imu::tare_pitch() const { return {}; }
std::int32_t Imu::tare_yaw() const { return {}; }
std::int32_t Imu::tare_roll() const { return {}; }
std::int32_t Imu::tare() const { return {}; }
std::int32_t Imu::tare_euler() const { return {}; }
std::int32_t Imu::set_heading(const double) const { return {}; }
std::int32_t Imu::set_yaw(const double) const { return {}; }
std::int32_t Imu::set_pitch(const double) const { return {}; }
std::int32_t Imu::set_roll(const double) const { return {}; }
std::int32_t Imu::set_euler(const pros::euler_s_t) const { return {}; }
pros::imu_accel_s_t Imu::get_accel() const { return {}; }
pros::ImuStatus Imu::get_status() const { return {}; }
bool Imu::is_calibrating() const { return {}; }
imu_orientation_e_t Imu::get_physical_orientation() const { return {}; }

}  // namespace v5
}  // namespace pros
