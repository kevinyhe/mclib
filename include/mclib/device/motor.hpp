// mclib
#pragma once

#include "mclib/device/types.hpp"

#include <cstdint>

namespace mclib {
namespace device {

class Motor {
public:
  explicit Motor(std::int8_t port, Gearset gearset = Gearset::Blue);

  void setVoltage(double volts);
  void setPercent(double percent);
  void stop();
  void setBrakeMode(BrakeMode mode);
  void tarePosition();

  double getPositionDeg() const;
  double getActualVelocity() const;
  double getCurrentDraw() const;
  double getTemperature() const;

private:
  static std::int32_t voltsToMillivolts(double volts);
  static std::int32_t percentToPower(double percent);

  pros::Motor m_motor;
};

}  // namespace device
}  // namespace mclib
