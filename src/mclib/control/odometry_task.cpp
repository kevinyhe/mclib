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
  // Reads the one geometry in config.hpp. It used to read four mutable
  // `extern double` globals - `wheel_distance_in` (a circumference despite the
  // name), `vertical_tracker_diameter` (really a diameter) and
  // `vertical_tracker_dist_from_center` - and had to remember which convention
  // each one used. Wheel carries that now, and the conversion goes through
  // `encoderToDistance()`, so it cannot drop `gear_ratio` the way a bare
  // `wheel.circumference()` did. The math is in the header so a host test can
  // link it; this file cannot be built on a host.
  return odometryConfigFrom(::mclib::config::robot_drive_geometry,
                            ::mclib::config::vertical_tracking_wheel);
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
  // There is no horizontal tracking wheel in config.cpp, so there is nothing
  // to read. startOdometry() refuses a config that claims one rather than
  // letting Odometry integrate this hardcoded zero against a real offset.
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

  OdometryConfig effective = config;
  if (effective.use_horizontal_tracker) {
    // sampleOdometrySensors() has no horizontal sensor to read. Honouring the
    // flag would feed a constant zero reading into
    //   dx_body = 0 - dtheta * horizontal_offset_forward
    // which invents a sideways displacement on every turn - an in-place pivot
    // would walk the pose. Drop the flag instead of integrating a lie. Drive
    // odometryTick() yourself if you have wired a horizontal wheel up.
    effective.use_horizontal_tracker = false;
  }
  setOdometryConfig(effective);
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
