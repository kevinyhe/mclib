// mclib
#pragma once

#include "pros/imu.hpp"

class MockIMU : public pros::Imu
{
public:
    MockIMU(int port, double gain);

    double get_rotation() const override;
    double get_heading() const override;

private:
    double imu_gain;
};
