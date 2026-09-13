// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
