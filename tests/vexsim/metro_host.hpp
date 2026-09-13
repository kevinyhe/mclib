#pragma once

// Preincluded instead of the embedded PROS main header. This compatibility
// surface runs the separately supplied Metro sources; it is not an mclib port.
#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifndef PROS_ERR_F
#define PROS_ERR_F INFINITY
#endif

extern "C" {
// Logical, reversal-corrected hardware readings, not controller odometry.
// Heading is raw unwrapped CW degrees. Metro's MockIMU applies its own gain.
// Chassis encoder values are motor degrees; currents are per-motor mA.
struct MetroSample {
  double heading;
  double left_deg;
  double right_deg;
  double left_current_ma;
  double right_current_ma;
  double left_rpm;
  double right_rpm;
  double tracker_centideg;
  double optical_proximity;
  double optical_hue;
  double distance_mm;
};

// kind 0: drivetrain, channel 0 left / 1 right. mode 0 = voltage (V),
// 1 = coast, 2 = brake, 3 = hold (the latter three carry value 0).
// kind 1: intake, channel 0 bottom / 1 top; mode 0, logical raw -127..127.
// kind 2: pneumatic, channel = ASCII ADI port; mode 0, value = bool.
using MetroOutput = void (*)(int kind, int channel, int mode, double value);
// Events: 0 create, 1 sleep (delay,wake), 2 end (0 normal / -1 exception),
// 3 routine returned (time,0), 4 disabled, 5 error, 6 encoder tare (side,raw),
// 7 raw IMU reset/set (requested,offset). task = -1 outside a task.
using MetroEvent = void (*)(int kind, int task, double a, double b);

// One initialization per process. Call tick(0), then once per simulated ms.
// Callbacks may read metro_now/task_name/error, but must not reenter
// init/tick/disable or metro_state (original intake code can hold its mutex).
int metro_init(const MetroSample* sample, MetroOutput output, MetroEvent event);
int metro_tick(std::uint32_t now_ms, const MetroSample* sample);
// out[10]: x,y,scaled heading,correct_angle,is_turning,previous L/R output,
// intake enum,routine returned,return ms (-1 before return).
void metro_state(double* out);
void metro_disable();
const char* metro_error();
const char* metro_task_name(int task);
std::uint32_t metro_now();
}

namespace pros {
enum motor_brake_mode_e_t {
  E_MOTOR_BRAKE_COAST = 0,
  E_MOTOR_BRAKE_BRAKE = 1,
  E_MOTOR_BRAKE_HOLD = 2,
};
enum class MotorGears { red, green, blue };
enum controller_id_e_t { E_CONTROLLER_MASTER = 0 };
enum controller_digital_e_t {
  E_CONTROLLER_DIGITAL_R1,
  E_CONTROLLER_DIGITAL_R2,
  E_CONTROLLER_DIGITAL_L1,
  E_CONTROLLER_DIGITAL_L2,
  E_CONTROLLER_DIGITAL_X,
};

std::uint32_t millis();
void delay(std::uint32_t milliseconds);

class Task {
 public:
  explicit Task(std::function<void()> function, const char* name = "delayed");
  // PROS task handles do not join or cancel their task when destroyed.
  ~Task() = default;
 private:
  int id_;
};

using Mutex = std::mutex;

class Controller {
 public:
  explicit Controller(controller_id_e_t) {}
  bool get_digital(controller_digital_e_t) const { return false; }
};

class MotorGroup {
 public:
  MotorGroup(std::initializer_list<std::int8_t> ports, MotorGears gears);
  std::int32_t move_voltage(std::int32_t millivolts) const;
  std::int32_t set_brake_mode_all(motor_brake_mode_e_t mode);
  std::int32_t brake() const;
  std::int32_t tare_position();
  std::vector<double> get_position_all() const;
  std::vector<std::int32_t> get_current_draw_all() const;
  std::vector<double> get_actual_velocity_all() const;
 private:
  int side_;
  std::size_t count_;
  double zero_deg_ = 0;
  motor_brake_mode_e_t brake_mode_ = E_MOTOR_BRAKE_COAST;
};

class Motor {
 public:
  explicit Motor(std::int8_t port);
  std::int32_t move(std::int32_t raw_power) const;
 private:
  int channel_;
};

class Imu {
 public:
  explicit Imu(int port) : port_(port) {}
  virtual ~Imu() = default;
  virtual double get_rotation() const;
  virtual double get_heading() const;
  std::int32_t set_rotation(double heading);
  std::int32_t reset(bool blocking = false);
 private:
  int port_;
  double offset_ = 0;
};

class Rotation {
 public:
  explicit Rotation(int port) : port_(port) {}
  std::int32_t get_position() const;
 private:
  int port_;
};

class Optical {
 public:
  explicit Optical(int port) : port_(port) {}
  std::int32_t get_proximity() const;
  double get_hue() const;
  std::int32_t set_led_pwm(std::uint8_t value);
  std::int32_t set_integration_time(double milliseconds);
 private:
  int port_;
};

class Distance {
 public:
  explicit Distance(int port) : port_(port) {}
  std::int32_t get() const;
 private:
  int port_;
};

namespace adi {
class DigitalOut {
 public:
  explicit DigitalOut(char port, bool value = false) : port_(port), value_(value) {}
  std::int32_t set_value(bool value);
  bool get_value() const { return value_; }
 private:
  char port_;
  bool value_;
};
}

namespace competition {
bool is_autonomous();
bool is_disabled();
}
}
