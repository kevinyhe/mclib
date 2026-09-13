// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/device/motor_group.hpp"
#include "mclib/mechanism/mechanism.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

class MotorSubsystem : public StateMechanism<double> {
public:
  MotorSubsystem(std::initializer_list<std::int8_t> ports,
                 device::Gearset gearset = device::Gearset::Blue);
  MotorSubsystem(std::vector<std::int8_t> ports,
                 device::Gearset gearset = device::Gearset::Blue);

  void setVoltage(double volts);
  void setPercent(double percent);
  void stop();
  void onDisabled() override { stop(); m_motors.stop(); }
  double getCommandedVoltage() const;
  device::MotorGroup& motors();

  std::unique_ptr<Command> makeVoltageCommand(double volts);
  std::unique_ptr<Command> makePercentCommand(double percent);
  std::unique_ptr<Command> makeStopCommand();

protected:
  void applyState(const double& volts) override;

private:
  device::MotorGroup m_motors;
};

}  // namespace mechanism
}  // namespace mclib
