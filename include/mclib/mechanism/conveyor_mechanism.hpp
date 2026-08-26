// mclib
#pragma once

#include "mclib/device/motor_group.hpp"
#include "mclib/mechanism/mechanism.hpp"

#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

/**
 * @brief What the conveyor should be doing.
 */
enum class ConveyorState {
  /// Motors held at zero volts.
  Stopped,
  /// Run toward the scoring end at ConveyorConfig::forward_voltage.
  Forward,
  /// Run back toward the intake at ConveyorConfig::reverse_voltage.
  Reverse,
  /// Run at ConveyorConfig::index_voltage whenever the sensor gate reads
  /// false, and hold at zero volts while it reads true. Removing the object
  /// re-arms it for the next one. Without a sensor gate this behaves like
  /// Stopped.
  IndexToSensor,
};

/**
 * @brief Ports, voltages, and jam thresholds for a ConveyorMechanism.
 */
struct ConveyorConfig {
  /// Motor ports. Must contain at least one port, and opposed motors must be
  /// sign-corrected here (a negative port reverses that motor). Jam detection
  /// reads the group average, so uncorrected ports cancel out and look stalled.
  /// An empty list yields a conveyor that silently does nothing.
  std::vector<std::int8_t> motor_ports;
  device::Gearset gearset = device::Gearset::Blue;
  double forward_voltage = 12.0;
  double reverse_voltage = -12.0;
  double index_voltage = 8.0;
  /// Average current draw, in amps, above which the conveyor may be jammed.
  double jam_current_amps = 2.0;
  /// Average absolute velocity, in RPM, below which the conveyor may be jammed.
  double jam_velocity_rpm = 10.0;
  /// How long both jam conditions must hold before an unjam starts.
  double jam_dwell_ms = 250.0;
  /// How long each unjam attempt runs.
  double unjam_ms = 250.0;
  double unjam_voltage = -8.0;
  /// How long the conveyor must run unstalled before the retry budget refills.
  double jam_clear_ms = 1000.0;
  /// Consecutive unjam attempts allowed before the conveyor gives up and
  /// latches jammed. The count resets after jam_clear_ms of clean running.
  int max_unjam_retries = 3;
  /// Absolute voltage ceiling applied to every value above.
  double max_voltage = 12.0;
};

/**
 * @brief A sensor-gated conveyor with current-based jam detection and recovery.
 *
 * Jam detection: average current draw above ConveyorConfig::jam_current_amps
 * while average absolute velocity is below ConveyorConfig::jam_velocity_rpm,
 * sustained for ConveyorConfig::jam_dwell_ms. On a jam the conveyor runs
 * ConveyorConfig::unjam_voltage for ConveyorConfig::unjam_ms and then resumes
 * the state it was already in. After ConveyorConfig::max_unjam_retries
 * consecutive attempts it stops the motors and latches isJammed(), so a
 * permanent jam cannot ping-pong forever. The retry count refills after
 * ConveyorConfig::jam_clear_ms of unstalled running; the isJammed() latch
 * clears on the next state change or on clearJam().
 *
 * Inherited from StateMechanism: setState(), getState(), makeStateCommand(),
 * makeStateOnceCommand(), makeStateUntilCommand(), makeStateForCommand().
 * Those are not shadowed here, so they behave the same through a base
 * reference. setConveyorState() / getConveyorState() are typed conveniences
 * over setState() / getState().
 */
class ConveyorMechanism : public StateMechanism<ConveyorState> {
public:
  /// Returns true when a game object sits at the index position.
  using SensorGate = std::function<bool()>;

  /**
   * @param config Ports, voltages, and jam thresholds.
   * @param sensor_gate Optional. When null, IndexToSensor holds at zero volts
   *        and hasObject() always returns false.
   */
  explicit ConveyorMechanism(const ConveyorConfig& config,
                             SensorGate sensor_gate = nullptr);

  ConveyorMechanism(std::initializer_list<std::int8_t> motor_ports,
                    device::Gearset gearset = device::Gearset::Blue);

  void setConveyorState(ConveyorState state);
  ConveyorState getConveyorState() const;
  void stop();

  /// Replaces the sensor gate. Pass nullptr to remove it.
  void setSensorGate(SensorGate sensor_gate);

  /// True once max_unjam_retries unjam attempts have failed.
  bool isJammed() const;

  /// The last sensor gate reading. Always false without a gate.
  bool hasObject() const;

  /// Clears a latched jam and resets the retry budget.
  void clearJam();

  std::unique_ptr<Command> makeForwardCommand();
  std::unique_ptr<Command> makeReverseCommand();
  std::unique_ptr<Command> makeStopCommand();

  /**
   * @brief Indexes until the sensor gate latches, then finishes.
   *
   * Also finishes on a latched jam, and, when timeout_ms is greater than zero,
   * after that many milliseconds. With no sensor gate it finishes immediately
   * instead of running forever. On a normal finish it returns the conveyor to
   * Stopped; on a latched jam it leaves the state alone so isJammed() still
   * reads true afterward (the motors are held at zero volts either way).
   */
  std::unique_ptr<Command> makeIndexCommand(double timeout_ms = 0.0);

protected:
  void applyState(const ConveyorState& state) override;
  void onStateChanged(const ConveyorState& state) override;

private:
  double voltageForState(ConveyorState state) const;
  double clampVoltage(double volts) const;
  void resetRecovery();
  void beginIndex();

  ConveyorConfig m_config;
  device::MotorGroup m_motors;
  SensorGate m_sensor_gate;

  bool m_object_present = false;
  bool m_index_latched = false;
  bool m_jam_timing = false;
  double m_jam_start_ms = 0.0;
  bool m_clear_timing = false;
  double m_clear_start_ms = 0.0;
  bool m_unjamming = false;
  double m_unjam_end_ms = 0.0;
  int m_unjam_retries = 0;
  bool m_jammed = false;
};

}  // namespace mechanism
}  // namespace mclib
