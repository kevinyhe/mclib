// mclib
#include "mclib/control/chassis_io.hpp"

#include "mclib/config.hpp"
void driveChassis(double left_power, double right_power)
{
  left_chassis.setVoltage(left_power);
  right_chassis.setVoltage(right_power);
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
  // Set both chassis motor encoders to zero
  left_chassis.tarePosition();
  right_chassis.tarePosition();
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
