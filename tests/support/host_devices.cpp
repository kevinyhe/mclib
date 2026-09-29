// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host stand-ins for the parts of libpros that mclib's device wrappers call:
// pros::MotorGroup, pros::Imu, pros::Controller and pros::Mutex. With these,
// the real src/mclib/device/motor_group.cpp, inertial.cpp and controller.cpp
// link on the host, and so does any library code built on them.
//
// pros::MotorGroup and pros::Imu have virtual methods, so their vtables need
// every one of them defined. The ones the wrappers call write or read the
// maps in host_devices.hpp. The rest return a value-initialised result; they
// are listed only so the vtables link.
//
// Not in HOST_TEST_SRC. A test that needs it gets its own link rule in the
// Makefile, the same way motion_safety_test gets its PROS clock.
#include "support/host_devices.hpp"

#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
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
void reset() {
  motors.clear();
  imu_rotation_deg.clear();
  analog.clear();
}
}  // namespace host_devices
}  // namespace test
}  // namespace mclib

namespace {
namespace hd = mclib::test::host_devices;

hd::Motor& motorAt(std::int8_t port) {
  return hd::motors[std::abs(static_cast<int>(port))];
}

double direction(std::int8_t port) {
  return port < 0 ? -1.0 : 1.0;
}
}  // namespace

namespace pros {

// No RTOS on the host. The MotorGroup member mutex is never locked by the
// stand-ins below, so it only has to construct and destroy.
mutex_t Mutex::lazy_init() { return nullptr; }
Mutex::~Mutex() {}

inline namespace v5 {

// ---------------------------------------------------------------- MotorGroup

MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports,
                       const MotorGears gearset,
                       const MotorUnits encoder_units)
    : MotorGroup(std::vector<std::int8_t>(ports), gearset, encoder_units) {}

MotorGroup::MotorGroup(const std::vector<std::int8_t>& ports,
                       const MotorGears /*gearset*/,
                       const MotorUnits /*encoder_units*/)
    : _ports(ports) {
  for (const std::int8_t port : _ports) {
    motorAt(port);
  }
}

MotorGroup::MotorGroup(AbstractMotor& motor_group) : _ports(motor_group.get_port_all()) {}

std::int32_t MotorGroup::move(std::int32_t voltage) const {
  for (const std::int8_t port : _ports) {
    motorAt(port).volts = direction(port) * static_cast<double>(voltage) * 12.0 / 127.0;
  }
  return 1;
}

std::int32_t MotorGroup::move_voltage(const std::int32_t voltage) const {
  for (const std::int8_t port : _ports) {
    motorAt(port).volts = direction(port) * static_cast<double>(voltage) / 1000.0;
  }
  return 1;
}

std::int32_t MotorGroup::brake(void) const {
  for (const std::int8_t port : _ports) {
    motorAt(port).volts = 0.0;
    ++motorAt(port).brake_calls;
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
    out.push_back(direction(port) * motorAt(port).position_deg);
  }
  return out;
}

std::vector<double> MotorGroup::get_actual_velocity_all(void) const {
  return std::vector<double>(_ports.size(), 0.0);
}

std::vector<std::int32_t> MotorGroup::get_current_draw_all(void) const {
  return std::vector<std::int32_t>(_ports.size(), 0);
}

std::vector<double> MotorGroup::get_temperature_all(void) const {
  return std::vector<double>(_ports.size(), 0.0);
}

std::vector<std::int8_t> MotorGroup::get_port_all(void) const {
  return _ports;
}

std::int8_t MotorGroup::size(void) const {
  return static_cast<std::int8_t>(_ports.size());
}

// ----------------------------------------------------------------------- Imu

Device::Device(const std::uint8_t port) : _port(port) {}
std::uint8_t Device::get_port(void) const { return _port; }
bool Device::is_installed() { return true; }

std::int32_t Imu::reset(bool /*blocking*/) const {
  hd::imu_rotation_deg[_port] = 0.0;
  return 1;
}

double Imu::get_rotation() const {
  return hd::imu_rotation_deg[_port];
}

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
  return hd::analog[static_cast<int>(channel)];
}

std::int32_t Controller::get_digital(controller_digital_e_t /*button*/) {
  return 0;
}

std::int32_t Controller::set_text(std::uint8_t /*line*/, std::uint8_t /*col*/,
                                  const char* /*str*/) {
  return 1;
}

std::int32_t Controller::set_text(std::uint8_t line, std::uint8_t col,
                                  const std::string& str) {
  return set_text(line, col, str.c_str());
}

// ---------------------------------------------- Unused, present for vtables

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
double MotorGroup::get_power(const std::uint8_t) const { return {}; }
std::vector<double> MotorGroup::get_power_all(void) const { return {}; }
std::int32_t MotorGroup::get_raw_position(std::uint32_t* const, const std::uint8_t) const { return {}; }
std::vector<std::int32_t> MotorGroup::get_raw_position_all(std::uint32_t* const) const { return {}; }
double MotorGroup::get_temperature(const std::uint8_t) const { return {}; }
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
std::int32_t MotorGroup::set_current_limit(const std::int32_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_current_limit_all(const std::int32_t) const { return {}; }
std::int32_t MotorGroup::set_encoder_units(const MotorUnits, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_encoder_units(const pros::motor_encoder_units_e_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_encoder_units_all(const MotorUnits) const { return {}; }
std::int32_t MotorGroup::set_encoder_units_all(const pros::motor_encoder_units_e_t) const { return {}; }
std::int32_t MotorGroup::set_gearing(std::vector<pros::motor_gearset_e_t>) const { return {}; }
std::int32_t MotorGroup::set_gearing(const pros::motor_gearset_e_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_gearing(std::vector<MotorGears>) const { return {}; }
std::int32_t MotorGroup::set_gearing(const MotorGears, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const MotorGears) const { return {}; }
std::int32_t MotorGroup::set_gearing_all(const pros::motor_gearset_e_t) const { return {}; }
std::int32_t MotorGroup::set_reversed(const bool, const std::uint8_t) { return {}; }
std::int32_t MotorGroup::set_reversed_all(const bool) { return {}; }
std::int32_t MotorGroup::set_voltage_limit(const std::int32_t, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_voltage_limit_all(const std::int32_t) const { return {}; }
std::int32_t MotorGroup::set_zero_position(const double, const std::uint8_t) const { return {}; }
std::int32_t MotorGroup::set_zero_position_all(const double) const { return {}; }
std::int32_t MotorGroup::tare_position(const std::uint8_t) const { return {}; }
std::int8_t MotorGroup::get_port(const std::uint8_t) const { return {}; }
void MotorGroup::operator+=(AbstractMotor&) {}
void MotorGroup::append(AbstractMotor&) {}
void MotorGroup::erase_port(std::int8_t) {}
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
