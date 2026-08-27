// mclib
#include "mclib/control/chassis_io.hpp"

#include "mclib/config.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
void driveChassis(QVoltage left_power, QVoltage right_power)
{
  left_chassis.setVoltage(left_power.volts());
  right_chassis.setVoltage(right_power.volts());
}

void stopChassis(mclib::device::BrakeMode mode)
{
  left_chassis.setBrakeMode(mode);
  right_chassis.setBrakeMode(mode);
  left_chassis.brake();
  right_chassis.brake();
}

void resetChassis()
{
  // Snapshot the pose BEFORE taring. The odometry task samples these same
  // encoders every 10 ms on another task; if it ticks between the tare and the
  // reset it reads the tare as a delta the size of everything driven so far
  // and corrupts the pose. Reading the pose afterwards would then adopt the
  // corrupted value as the reset target and make it permanent.
  const mclib::Pose2D pose = mclib::control::robotState().pose();

  // Set both chassis motor encoders to zero
  left_chassis.tarePosition();
  right_chassis.tarePosition();

  // Re-seed the odometry baseline, which also repairs any tick that landed in
  // the window above.
  mclib::control::resetOdometry(pose);
}

double getLeftRotationDegree()
{
  auto positions = left_chassis.getPositionsDeg();
  if (positions.empty())
  {
    return 0;
  }

  double total = 0;
  for (double position : positions)
  {
    total += position;
  }
  return total / static_cast<double>(positions.size());
}

double getRightRotationDegree()
{
  auto positions = right_chassis.getPositionsDeg();
  if (positions.empty())
  {
    return 0;
  }

  double total = 0;
  for (double position : positions)
  {
    total += position;
  }
  return total / static_cast<double>(positions.size());
}

double getInertialHeading()
{
  // Get inertial sensor rotation in degrees
  return inertial_sensor.getRotationDeg();
}

double normalizeTarget(double angle)
{
  // wrap target so heading math always uses the minimal signed angular difference
  if (angle - getInertialHeading() > 180)
  {
    while (angle - getInertialHeading() > 180)
      angle -= 360;
  }
  else if (angle - getInertialHeading() < -180)
  {
    while (angle - getInertialHeading() < -180)
      angle += 360;
  }
  return angle;
}
