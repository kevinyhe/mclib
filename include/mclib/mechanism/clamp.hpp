// mclib
#pragma once

#include "mclib/device/distance.hpp"
#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"

#include <cstdint>
#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief Configuration for a Clamp.
 *
 * `default_state` and every state this class stores are LOGICAL: true means
 * "clamped onto a goal". The solenoid's raw pin value is derived from
 * `extended_state` only at the moment the state is written to hardware, so an
 * inverted solenoid never corrupts the cached logical state.
 */
struct ClampConfig {
  /** 3-wire port the clamp solenoid is plugged into. */
  char adi_port = 'A';
  /** Whether driving the solenoid true is the CLAMPED position. */
  bool extended_state = true;
  /** Logical state the clamp starts in (true = clamped). */
  bool default_state = false;
  /** Smart port of the goal-detecting distance sensor, 0 (or any
   * non-positive value) for no sensor. */
  std::int8_t distance_port = 0;
  /** Reading below which a goal is considered in range, in millimetres. */
  double auto_clamp_threshold_mm = 50.0;
  /** Whether auto-clamping is armed at construction. */
  bool auto_clamp_enabled = false;
  /**
   * Minimum distance-sensor confidence (0-63) that is trusted.
   *
   * PROS returns a fixed confidence of 10 for anything closer than 200mm, so a
   * clamp threshold under 200mm must not ask for more than 10 or the sensor's
   * readings are thrown away and auto-clamp never fires.
   */
  std::int32_t min_confidence = 10;
};

/**
 * @brief A pneumatic mobile-goal clamp with optional sensor-triggered
 * auto-clamping.
 *
 * The mechanism state is a bool: true = clamped, false = released.
 *
 * Auto-clamp behaviour: when armed and a distance sensor is configured, the
 * clamp closes on its own as soon as a trusted reading drops below
 * `auto_clamp_threshold_mm`. A manual release LATCHES auto-clamp off so the
 * driver can push a goal away without the clamp instantly grabbing it again.
 * The latch clears when the sensor reports nothing in range, when a trusted
 * reading rises back above the threshold (either way, the goal is gone), or
 * when the clamp is closed manually. An untrusted reading leaves the latch
 * alone, since it says nothing about where the goal is.
 */
class Clamp : public StateMechanism<bool> {
public:
  explicit Clamp(const ClampConfig& config);

  /** @brief Close the clamp and clear the manual-release latch. */
  void clamp();
  /** @brief Open the clamp and latch auto-clamp off until the goal is gone. */
  void release();
  /** @brief Clamp if released, release if clamped. */
  void toggle();
  /** @brief True when the clamp is logically closed. */
  bool isClamped() const;

  /**
   * @brief Arm or disarm auto-clamping.
   *
   * Either direction clears the release latch, so re-arming always starts from
   * a clean slate.
   */
  void setAutoClamp(bool enabled);
  bool isAutoClampEnabled() const;

  /**
   * @brief Latest distance reading in millimetres.
   *
   * @return -1.0 when there is no sensor, the port errored, or the reading
   * failed the confidence gate. 9999 means the sensor sees nothing in range.
   */
  double distanceMm() const;

  void periodic() override;

  /** @brief Closes the clamp, then finishes immediately. */
  std::unique_ptr<Command> makeClampCommand();
  /** @brief Opens the clamp, then finishes immediately. */
  std::unique_ptr<Command> makeReleaseCommand();
  /** @brief Toggles the clamp, then finishes immediately. */
  std::unique_ptr<Command> makeToggleCommand();
  /**
   * @brief Arms auto-clamp and finishes once clamped, or on timeout.
   *
   * @param timeout_ms Milliseconds before giving up, 0 for no timeout.
   */
  std::unique_ptr<Command> makeAutoClampCommand(double timeout_ms = 0.0);

protected:
  void applyState(const bool& clamped) override;

private:
  /** @brief Map a logical clamped state onto the solenoid's raw value. */
  bool solenoidValueFor(bool clamped) const;
  void updateAutoClamp();

  ClampConfig m_config;
  std::unique_ptr<device::Pneumatic> m_pneumatic;
  std::unique_ptr<device::Distance> m_distance;
  bool m_auto_clamp_enabled;
  bool m_release_latched = false;
};

}  // namespace mechanism
}  // namespace mclib
