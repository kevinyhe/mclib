// mclib
#pragma once

#include "mclib/device/motor_group.hpp"
#include "mclib/device/rotation.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/pid.hpp"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

struct ArmConfig {
  std::vector<std::int8_t> motor_ports;
  std::int8_t rotation_port = 0;
  bool rotation_reversed = false;
  device::Gearset gearset = device::Gearset::Blue;
  double kp = 0.08;
  double ki = 0.0;
  double kd = 0.0;
  double max_voltage = 12.0;
  double small_error_deg = 2.0;
  double big_error_deg = 5.0;
  double small_duration_ms = 50.0;
  double big_duration_ms = 250.0;
  double derivative_tolerance = 5.0;
};

class Arm : public StateMechanism<double> {
public:
  explicit Arm(const ArmConfig& config);
  Arm(std::initializer_list<std::int8_t> motor_ports,
      std::int8_t rotation_port,
      device::Gearset gearset = device::Gearset::Blue);

  void moveTo(double target_deg);
  void setManualVoltage(double volts);
  void stop();

  double positionDeg() const;
  double targetDeg() const;
  bool atTarget();

  std::unique_ptr<Command> makeMoveToCommand(double target_deg,
                                             double timeout_ms = 0.0);
  std::unique_ptr<Command> makeManualCommand(double volts);
  std::unique_ptr<Command> makeStopCommand();

protected:
  void applyState(const double& target_deg) override;
  void onStateChanged(const double& target_deg) override;

private:
  static double clampVoltage(double volts, double max_voltage);

  ArmConfig m_config;
  device::MotorGroup m_motors;
  device::Rotation m_rotation;
  PID m_pid;
  bool m_manual = false;
  double m_manual_voltage = 0.0;
};

}  // namespace mechanism
}  // namespace mclib
