#include "metro_host.hpp"

#include "autonomous.h"
#include "command/commandScheduler.h"
#include "config.h"
#include "control.h"
#include "mechanism/intake.hpp"

#include <array>
#include <cstdlib>
#include <exception>
#include <ucontext.h>

extern double prev_left_output;
extern double prev_right_output;

namespace {
constexpr std::size_t kStackBytes = 512 * 1024;
constexpr std::size_t kMaxTasks = 64;
constexpr std::size_t kMaxResumptionsPerTick = 1024;

struct HostTask {
  int id;
  std::string name;
  std::function<void()> function;
  std::unique_ptr<unsigned char[]> stack;
  ucontext_t context{};
  std::uint64_t wake_ms = 0;
  bool finished = false;
};

struct HostRuntime {
  MetroSample sample{};
  MetroOutput output = nullptr;
  MetroEvent event = nullptr;
  std::vector<std::unique_ptr<HostTask>> tasks;
  ucontext_t master{};
  HostTask* current = nullptr;
  std::uint32_t now = 0;
  std::int64_t last_tick = -1;
  bool initialized = false;
  bool intake_initialized = false;
  bool disabled = false;
  bool entered = false;
  bool returned = false;
  std::int64_t return_ms = -1;
  std::string error;
};

// Embedded globals do not have a shutdown phase on the robot. Keep host state
// alive through library teardown; callbacks are detached by metro_disable().
HostRuntime& runtime() {
  static HostRuntime* value = new HostRuntime;
  return *value;
}

int task_id() { return runtime().current ? runtime().current->id : -1; }

void event(int kind, double a = 0, double b = 0, int id = -2) {
  auto& host = runtime();
  if (host.event) host.event(kind, id == -2 ? task_id() : id, a, b);
}

void remember_error(const std::string& message) {
  auto& host = runtime();
  if (host.error.empty()) {
    host.error = message;
    event(5);
  }
}

void emit(int kind, int channel, int mode, double value) {
  if (!std::isfinite(value)) throw std::runtime_error("nonfinite hardware output");
  auto& host = runtime();
  if (host.output) host.output(kind, channel, mode, value);
}

void validate_sample(const MetroSample* sample) {
  if (!sample) throw std::invalid_argument("null MetroSample");
  const std::array values{
      sample->heading, sample->left_deg, sample->right_deg,
      sample->left_current_ma, sample->right_current_ma,
      sample->left_rpm, sample->right_rpm, sample->tracker_centideg,
      sample->optical_proximity, sample->optical_hue, sample->distance_mm};
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("nonfinite MetroSample");
  }
  if (sample->optical_proximity < 0 || sample->optical_proximity > 255)
    throw std::invalid_argument("optical proximity outside 0..255");
}

std::int32_t checked_int(double value) {
  if (!std::isfinite(value) || value < std::numeric_limits<std::int32_t>::min() ||
      value > std::numeric_limits<std::int32_t>::max())
    throw std::runtime_error("hardware reading outside int32 range");
  return static_cast<std::int32_t>(value);
}

double side_value(int side, double left, double right) {
  if (side == 0) return left;
  if (side == 1) return right;
  throw std::runtime_error("unsupported MotorGroup used by Metro routine");
}

void task_entry() {
  auto& host = runtime();
  auto& task = *host.current;
  bool failed = false;
  try {
    task.function();
  } catch (const std::exception& exception) {
    failed = true;
    remember_error("task " + task.name + ": " + exception.what());
  } catch (...) {
    failed = true;
    remember_error("task " + task.name + ": unknown C++ exception");
  }
  task.finished = true;
  event(2, failed ? -1 : 0);
  // The trampoline returns only into the saved C++ master context. Hardware
  // callbacks have returned before either this return or pros::delay yields.
}

int create_task(std::function<void()> function, const char* name) {
  auto& host = runtime();
  if (!host.initialized || host.disabled) throw std::runtime_error("task creation outside enabled runtime");
  if (host.tasks.size() >= kMaxTasks) throw std::runtime_error("task count safety cap exceeded");
  auto task = std::make_unique<HostTask>();
  task->id = static_cast<int>(host.tasks.size());
  task->name = name ? name : "unnamed";
  task->function = std::move(function);
  task->stack = std::make_unique<unsigned char[]>(kStackBytes);
  task->wake_ms = host.now;
  if (getcontext(&task->context) != 0) throw std::runtime_error("getcontext failed");
  task->context.uc_stack.ss_sp = task->stack.get();
  task->context.uc_stack.ss_size = kStackBytes;
  task->context.uc_link = &host.master;
  makecontext(&task->context, task_entry, 0);
  const int id = task->id;
  host.tasks.push_back(std::move(task));
  event(0, host.now, 0, id);
  return id;
}

void stop_runtime() {
  auto& host = runtime();
  if (host.disabled) return;
  host.disabled = true;
  // No task is executing here. Retain names/finished flags for inspection but
  // drop all suspended stacks and captures, without resuming their controller.
  for (auto& task : host.tasks) {
    task->finished = true;
    task->stack.reset();
    task->function = {};
  }
  left_chassis.brake();
  right_chassis.brake();
  if (host.intake_initialized) mechanism::Intake::get_instance().stop_task();
  event(4);
}
}

namespace pros {
std::uint32_t millis() { return runtime().now; }

void delay(std::uint32_t milliseconds) {
  auto& host = runtime();
  if (!host.current) throw std::runtime_error("pros::delay outside a host task");
  host.current->wake_ms = static_cast<std::uint64_t>(host.now) + milliseconds;
  event(1, milliseconds, host.current->wake_ms);
  if (swapcontext(&host.current->context, &host.master) != 0)
    throw std::runtime_error("task yield swapcontext failed");
}

Task::Task(std::function<void()> function, const char* name)
    : id_(create_task(std::move(function), name)) {}

MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports, MotorGears)
    : side_(ports.size() && std::abs(*ports.begin()) == 11 ? 0 :
            ports.size() && std::abs(*ports.begin()) == 16 ? 1 : -1),
      count_(ports.size()) {}

std::int32_t MotorGroup::move_voltage(std::int32_t millivolts) const {
  side_value(side_, 0, 0);
  emit(0, side_, 0, std::clamp(millivolts, -12000, 12000) / 1000.0);
  return 1;
}

std::int32_t MotorGroup::set_brake_mode_all(motor_brake_mode_e_t mode) {
  if (mode < E_MOTOR_BRAKE_COAST || mode > E_MOTOR_BRAKE_HOLD)
    throw std::runtime_error("unsupported brake mode");
  brake_mode_ = mode;
  return 1;
}

std::int32_t MotorGroup::brake() const {
  side_value(side_, 0, 0);
  emit(0, side_, static_cast<int>(brake_mode_) + 1, 0);
  return 1;
}

std::int32_t MotorGroup::tare_position() {
  const auto& sample = runtime().sample;
  zero_deg_ = side_value(side_, sample.left_deg, sample.right_deg);
  event(6, side_, zero_deg_);
  return 1;
}

std::vector<double> MotorGroup::get_position_all() const {
  const auto& sample = runtime().sample;
  return std::vector<double>(count_, side_value(side_, sample.left_deg, sample.right_deg) - zero_deg_);
}

std::vector<std::int32_t> MotorGroup::get_current_draw_all() const {
  const auto& sample = runtime().sample;
  return std::vector<std::int32_t>(count_, checked_int(side_value(side_, sample.left_current_ma, sample.right_current_ma)));
}

std::vector<double> MotorGroup::get_actual_velocity_all() const {
  const auto& sample = runtime().sample;
  return std::vector<double>(count_, side_value(side_, sample.left_rpm, sample.right_rpm));
}

Motor::Motor(std::int8_t port)
    : channel_(std::abs(port) == 21 ? 0 : std::abs(port) == 20 ? 1 : -1) {}

std::int32_t Motor::move(std::int32_t raw_power) const {
  if (channel_ < 0) throw std::runtime_error("unsupported intake Motor used");
  emit(1, channel_, 0, std::clamp(raw_power, -127, 127));
  return 1;
}

double Imu::get_rotation() const { return runtime().sample.heading + offset_; }
double Imu::get_heading() const {
  double heading = std::fmod(Imu::get_rotation(), 360.0);
  return heading < 0 ? heading + 360.0 : heading;
}
std::int32_t Imu::set_rotation(double heading) {
  if (!std::isfinite(heading)) throw std::runtime_error("nonfinite IMU reset heading");
  offset_ = heading - runtime().sample.heading;
  event(7, heading, offset_);
  return 1;
}
std::int32_t Imu::reset(bool) {
  // Simulation begins after pre-autonomous calibration, without advancing the
  // autonomous clock or moving the physical robot.
  return set_rotation(0);
}

std::int32_t Rotation::get_position() const { return checked_int(runtime().sample.tracker_centideg); }
std::int32_t Optical::get_proximity() const { return checked_int(runtime().sample.optical_proximity); }
double Optical::get_hue() const { return runtime().sample.optical_hue; }
std::int32_t Optical::set_led_pwm(std::uint8_t) { return 1; }
std::int32_t Optical::set_integration_time(double) { return 1; }
std::int32_t Distance::get() const { return checked_int(runtime().sample.distance_mm); }

std::int32_t adi::DigitalOut::set_value(bool value) {
  value_ = value;
  emit(2, static_cast<unsigned char>(port_), 0, value ? 1 : 0);
  return 1;
}

bool competition::is_autonomous() { return runtime().initialized && !runtime().disabled; }
bool competition::is_disabled() { return !runtime().initialized || runtime().disabled; }
}

extern "C" int metro_init(const MetroSample* sample, MetroOutput output, MetroEvent callback) {
  auto& host = runtime();
  if (host.entered) return -1;
  host.entered = true;
  try {
    if (host.initialized) throw std::runtime_error("Metro supports only one init per process");
    validate_sample(sample);
    if (!output) throw std::invalid_argument("MetroOutput is required");
    host.sample = *sample;
    host.output = output;
    host.event = callback;
    host.initialized = true;
    auto bottom = std::shared_ptr<pros::Motor>(&intake_bottom_motor, [](pros::Motor*) {});
    auto top = std::shared_ptr<pros::Motor>(&intake_top_motor, [](pros::Motor*) {});
    auto optical = std::make_shared<pros::Optical>(9);
    optical->set_led_pwm(100);
    optical->set_integration_time(10);
    auto distance = std::make_shared<pros::Distance>(7);
    mechanism::Intake::initialize(bottom, top, optical, distance, 60, 40.0, 200.0);
    host.intake_initialized = true;
    inertial_sensor.reset(true);
    correct_angle = inertial_sensor.get_heading();
    xpos = 0;
    ypos = 0;
    pros::Task([] { trackNoOdomWheel(); }, "odom_no");
    pros::Task([] {
      while (pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
        CommandScheduler::run();
        pros::delay(10);
      }
    }, "auton_scheduler");
    pros::Task([] {
      leftSideSevenMiddle();
      auto& host = runtime();
      host.returned = true;
      host.return_ms = host.now;
      event(3, host.now);
    }, "leftSideSevenMiddle");
  } catch (const std::exception& exception) {
    remember_error(exception.what());
  } catch (...) {
    remember_error("unknown exception during Metro initialization");
  }
  if (!host.error.empty() && host.initialized) {
    try { stop_runtime(); } catch (...) {}
  }
  host.entered = false;
  return host.error.empty() ? 0 : -1;
}

extern "C" int metro_tick(std::uint32_t now_ms, const MetroSample* sample) {
  auto& host = runtime();
  if (host.entered) return -1;
  host.entered = true;
  try {
    if (!host.initialized) throw std::runtime_error("Metro is not initialized");
    if (!host.error.empty()) throw std::runtime_error(host.error);
    if (static_cast<std::int64_t>(now_ms) != host.last_tick + 1)
      throw std::invalid_argument("Metro ticks must start at zero and advance exactly one ms");
    host.now = now_ms;
    host.last_tick = now_ms;
    validate_sample(sample);
    host.sample = *sample;
    if (!host.disabled) {
      std::size_t resumptions = 0;
      bool ran;
      do {
        ran = false;
        // Tasks are visited in creation-ID order on each pass; tasks created
        // during a pass can start at the same timestamp, after their creator.
        for (std::size_t index = 0; index < host.tasks.size(); ++index) {
          auto& task = *host.tasks[index];
          if (task.finished || task.wake_ms > now_ms) continue;
          if (++resumptions > kMaxResumptionsPerTick)
            throw std::runtime_error("ready-task resumption safety cap exceeded");
          host.current = &task;
          if (swapcontext(&host.master, &task.context) != 0)
            throw std::runtime_error("master swapcontext failed");
          host.current = nullptr;
          if (!host.error.empty()) throw std::runtime_error(host.error);
          ran = true;
        }
      } while (ran);
    }
  } catch (const std::exception& exception) {
    host.current = nullptr;
    remember_error(exception.what());
    try { stop_runtime(); } catch (...) {}
  } catch (...) {
    host.current = nullptr;
    remember_error("unknown exception during Metro tick");
    try { stop_runtime(); } catch (...) {}
  }
  host.entered = false;
  return host.error.empty() ? 0 : -1;
}

extern "C" void metro_state(double* out) {
  if (!out) return;
  auto& host = runtime();
  out[0] = xpos;
  out[1] = ypos;
  out[2] = inertial_sensor.get_rotation();
  out[3] = correct_angle;
  out[4] = is_turning ? 1 : 0;
  out[5] = prev_left_output;
  out[6] = prev_right_output;
  out[7] = host.intake_initialized ? static_cast<int>(mechanism::Intake::get_instance().get_state()) : -1;
  out[8] = host.returned ? 1 : 0;
  out[9] = host.return_ms;
}

extern "C" void metro_disable() {
  auto& host = runtime();
  if (host.entered) return;
  host.entered = true;
  try { if (host.initialized) stop_runtime(); }
  catch (const std::exception& exception) { remember_error(exception.what()); }
  catch (...) { remember_error("unknown exception during Metro disable"); }
  host.entered = false;
  // Python callback closures may be released after disable. No original
  // singleton destructor may subsequently call into that interpreter state.
  host.output = nullptr;
  host.event = nullptr;
}

extern "C" const char* metro_error() { return runtime().error.c_str(); }
extern "C" const char* metro_task_name(int task) {
  auto& tasks = runtime().tasks;
  return task >= 0 && static_cast<std::size_t>(task) < tasks.size() ? tasks[task]->name.c_str() : "";
}
extern "C" std::uint32_t metro_now() { return runtime().now; }
