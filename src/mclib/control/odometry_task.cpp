// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/odometry_task.hpp"

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
// The task's own copy of the setup. Copied at start so the caller's struct
// can go out of scope; the readers inside still refer to the caller's
// sensors, which must not.
OdometrySetup g_setup;
}  // namespace

OdometrySetup odometrySetupFrom(DriveHardware& drive) {
  OdometrySetup setup;
  setup.heading_deg = [&drive]() { return drive.headingDeg(); };
  setup.left_deg = [&drive]() { return drive.leftPositionDeg(); };
  setup.right_deg = [&drive]() { return drive.rightPositionDeg(); };
  setup.drive = drive.driveGeometry();
  return setup;
}

bool odometrySetupIsUsable(const OdometrySetup& setup) {
  if (!setup.heading_deg) {
    return false;
  }
  const bool has_drive_encoders = setup.left_deg && setup.right_deg;
  const bool has_vertical =
      setup.vertical.has_value() && setup.vertical->position_deg;
  if (setup.horizontal.has_value() && !setup.horizontal->position_deg) {
    return false;
  }
  return has_drive_encoders || has_vertical;
}

OdometrySample sampleOdometrySensors(const OdometrySetup& setup) {
  OdometrySample sample;
  sample.heading_rad =
      setup.heading_deg ? degToRad(setup.heading_deg()) : NAN;
  sample.left_deg = setup.left_deg ? setup.left_deg() : 0.0;
  sample.right_deg = setup.right_deg ? setup.right_deg() : 0.0;
  sample.vertical_deg = setup.vertical.has_value() && setup.vertical->position_deg
                            ? setup.vertical->position_deg()
                            : 0.0;
  sample.horizontal_deg =
      setup.horizontal.has_value() && setup.horizontal->position_deg
          ? setup.horizontal->position_deg()
          : 0.0;
  return sample;
}

bool startOdometry(const OdometrySetup& setup, QTime period) {
  if (!odometrySetupIsUsable(setup)) {
    return false;
  }
  bool expected = false;
  if (!g_should_run.compare_exchange_strong(expected, true)) {
    return false;
  }
  g_setup = setup;
  setOdometryConfig(odometryConfigFrom(g_setup));
  resetOdometry(robotState().pose());
  const std::uint32_t period_ms =
      static_cast<std::uint32_t>(period.ms() < 1.0 ? 1.0 : period.ms());
  g_task = std::make_unique<pros::Task>(
      [period_ms]() {
        std::uint32_t now = pros::millis();
        while (g_should_run.load()) {
          odometryTick(sampleOdometrySensors(g_setup));
          pros::Task::delay_until(&now, period_ms);
        }
      },
      "mclib odometry");
  return true;
}

bool startOdometry(QTime period) {
  DriveHardware* drive = requireDrive("startOdometry");
  if (drive == nullptr) {
    return false;
  }
  return startOdometry(odometrySetupFrom(*drive), period);
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
