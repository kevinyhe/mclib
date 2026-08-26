// mclib
#pragma once

#include "mclib/device/types.hpp"

#include <cstdint>
#include <initializer_list>
#include <vector>

namespace mclib {
namespace device {

class MotorGroup {
public:
  MotorGroup(std::initializer_list<std::int8_t> ports,
             Gearset gearset = Gearset::Blue);
  MotorGroup(std::vector<std::int8_t> ports, Gearset gearset = Gearset::Blue);

  void setVoltage(double volts);
  void setPercent(double percent);
  void stop();
  void brake();
  void setBrakeMode(BrakeMode mode);
  void tarePosition();

  std::vector<double> getPositionsDeg() const;
  std::vector<double> getActualVelocities() const;
  std::vector<double> getCurrentDraws() const;
  std::vector<double> getTemperatures() const;
  double getAveragePositionDeg() const;
  double getAverageActualVelocity() const;
  double getAverageCurrentDraw() const;

private:
  static std::int32_t voltsToMillivolts(double volts);
  static std::int32_t percentToPower(double percent);
  static double average(const std::vector<double>& values);

  pros::MotorGroup m_motors;
};

}  // namespace device
}  // namespace mclib
