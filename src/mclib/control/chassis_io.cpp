// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/chassis_io.hpp"

#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"

#include <cmath>
#include <cstdio>

namespace mclib {
namespace control {

namespace {
DriveHardware* g_drive = nullptr;
bool g_warned_unbound = false;
}  // namespace

void bindDrive(DriveHardware* drive) {
  g_drive = drive;
  g_warned_unbound = false;
}

DriveHardware* boundDrive() {
  return g_drive;
}

bool isDriveBound() {
  return g_drive != nullptr;
}

DriveHardware* requireDrive(const char* caller) {
  if (g_drive == nullptr && !g_warned_unbound) {
    g_warned_unbound = true;
    std::printf("mclib: %s called with no drive bound. Build a "
                "ChassisController, or call mclib::control::bindDrive().\n",
                caller != nullptr ? caller : "a motion routine");
  }
  return g_drive;
}

}  // namespace control
}  // namespace mclib

namespace {
mclib::control::DriveHardware* drive() {
  return mclib::control::boundDrive();
}
}  // namespace

void driveChassis(QVoltage left_power, QVoltage right_power) {
  if (auto* hw = drive()) {
    if (!std::isfinite(left_power.volts()) || !std::isfinite(right_power.volts())) {
      hw->setDriveVoltage(0.0, 0.0);
      hw->brakeDrive(mclib::device::BrakeMode::Hold);
      return;
    }
    hw->setDriveVoltage(left_power.volts(), right_power.volts());
  }
}

void stopChassis(mclib::device::BrakeMode mode) {
  if (auto* hw = drive()) {
    hw->brakeDrive(mode);
  }
}

void resetChassis() {
  auto* hw = drive();
  if (hw == nullptr) {
    return;
  }
  // Snapshot the pose BEFORE taring. The odometry task samples these same
  // encoders every 10 ms on another task; if it ticks between the tare and the
  // reset it reads the tare as a delta the size of everything driven so far
  // and corrupts the pose. Reading the pose afterwards would then adopt the
  // corrupted value as the reset target and make it permanent.
  const mclib::Pose2D pose = mclib::control::robotState().pose();
  hw->tareDrive();
  // Re-seed the odometry baseline, which also repairs any tick that landed in
  // the window above.
  mclib::control::resetOdometry(pose);
}

double getLeftRotationDegree() {
  auto* hw = drive();
  return hw != nullptr ? hw->leftPositionDeg() : 0.0;
}

double getRightRotationDegree() {
  auto* hw = drive();
  return hw != nullptr ? hw->rightPositionDeg() : 0.0;
}

double getInertialHeading() {
  auto* hw = drive();
  return hw != nullptr ? hw->headingDeg() : NAN;
}

double normalizeTarget(double angle) {
  // Wrap the target so heading math always uses the minimal signed angular
  // difference. One read of the heading: reading it once per iteration made
  // the loop bound move while the loop ran.
  const double heading = getInertialHeading();
  if (!std::isfinite(heading) || !std::isfinite(angle)) {
    return NAN;
  }
  double delta = std::fmod(angle - heading, 360.0);
  if (delta > 180.0) delta -= 360.0;
  if (delta < -180.0) delta += 360.0;
  return heading + delta;
}
