// mclib
#include "mclib/control/odometry_task.hpp"

#include "mclib/config.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/utils.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>

namespace mclib {
namespace control {

namespace {

std::atomic_bool g_should_run{false};
std::unique_ptr<pros::Task> g_task;

}  // namespace

OdometryConfig odometryConfigFromGlobals() {
  OdometryConfig config;
  config.drive_inches_per_revolution = ::wheel_distance_in * units::inch;
  config.use_vertical_tracker = false;
  config.vertical_circumference =
      ::vertical_tracker_diameter * units::pi * units::inch;
  config.vertical_offset_right =
      ::vertical_tracker_dist_from_center * units::inch;
  config.use_horizontal_tracker = false;
  return config;
}

OdometrySample sampleOdometrySensors() {
  OdometrySample sample;
  // getInertialHeading() is in degrees and hands NAN straight through on a
  // sensor fault. Odometry::update() is the thing that checks; do not paper
  // over it here.
  sample.heading_rad = degToRad(getInertialHeading());
  sample.left_deg = getLeftRotationDegree();
  sample.right_deg = getRightRotationDegree();
  sample.vertical_deg = vertical_tracker.getPositionDeg();
  sample.horizontal_deg = 0.0;
  return sample;
}

bool startOdometry(const OdometryConfig& config, QTime period) {
  // Claim the flag and check it in one step, so two callers racing to start
  // cannot both create a task and leak the first one.
  bool expected = false;
  if (!g_should_run.compare_exchange_strong(expected, true)) {
    return false;
  }

  setOdometryConfig(config);
  // Drop any encoder baseline left over from a previous run. Without this, a
  // stop, some driving, and a restart would fold the whole gap into one tick.
  resetOdometry(robotState().pose());

  const std::uint32_t period_ms =
      static_cast<std::uint32_t>(period.ms() < 1.0 ? 1.0 : period.ms());

  g_task = std::make_unique<pros::Task>(
      [period_ms]() {
        std::uint32_t now = pros::millis();
        while (g_should_run.load()) {
          odometryTick(sampleOdometrySensors());
          pros::Task::delay_until(&now, period_ms);
        }
      },
      "mclib odometry");
  return true;
}

bool startOdometry() {
  return startOdometry(odometryConfigFromGlobals());
}

void stopOdometry() {
  g_should_run.store(false);
  if (g_task != nullptr) {
    g_task->join();
    g_task.reset();
  }
}

bool isOdometryRunning() {
  return g_should_run.load();
}

}  // namespace control
}  // namespace mclib
