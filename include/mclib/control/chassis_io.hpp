// mclib
#pragma once

#include "mclib/device/types.hpp"

void driveChassis(double left_power, double right_power);
void stopChassis(mclib::device::BrakeMode mode);
void resetChassis();

double getLeftRotationDegree();
double getRightRotationDegree();
double getInertialHeading();
double normalizeTarget(double angle);
