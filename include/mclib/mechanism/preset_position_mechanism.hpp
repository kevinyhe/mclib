// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/command/functionalCommand.h"
#include "mclib/command/subsystem.h"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/mechanism/multi_position_mechanism.hpp"
#include "mclib/mechanism/position_mechanism.hpp"
#include "mclib/time.hpp"

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <utility>

namespace mclib {
namespace mechanism {

/**
 * @brief A closed-loop position mechanism whose targets are named presets.
 *
 * @details This is the composition of the two mechanisms that already exist:
 * MultiPositionMechanism<StateT> owns the ordered preset table and the
 * next/previous/cycle stepping, PositionMechanism owns the PID loop, the
 * tolerance and dwell exit, the voltage clamp, the hold-at-target behavior and
 * the manual-voltage override. Nothing here re-implements either of them; the
 * setpoint sink of the first is wired into the target of the second.
 *
 * Use it for any articulated joint, lift, tilter or indexer that moves between
 * a fixed list of stops. It is parameterised on the caller's own enum, so the
 * preset names come from the robot, not from this library.
 *
 * ### One scheduler requirement, not two
 *
 * MultiPositionMechanism and PositionMechanism are both Subsystems. Registering
 * both for one physical mechanism would let two commands drive it at once: a
 * command requiring the preset half and a command requiring the PID half would
 * be allowed to run together, and they would fight over the motors.
 *
 * So only one of them is a scheduler subsystem. This class *derives* from
 * MultiPositionMechanism<StateT> (that is the registered requirement) and
 * *owns* the PositionMechanism as a plain component that is never registered.
 * The inner mechanism is ticked from this class's applyState(), which the
 * scheduler reaches through the one registration. Do not register the object
 * returned by controller().
 *
 * ### Why the sink is guarded
 *
 * MultiPositionMechanism pushes the current preset's setpoint into the sink on
 * every tick, but PositionMechanism::moveTo() restarts the PID every call.
 * Feeding one straight into the other would reset the loop 50 times a second
 * and it would never settle. The sink therefore retargets only when the
 * setpoint actually differs from the live target, or when a retarget was
 * explicitly requested.
 *
 * That explicit request is what makes re-issuing the *same* preset after a
 * manual-voltage override take control back. StateMechanism::setState only
 * fires onStateChanged on a real value change, so the hook alone is not enough.
 *
 * ### Raw targets and manual voltage
 *
 * moveTo(double) and setManualVoltage(double) put the preset table on hold, so
 * a table push cannot yank the mechanism back off a raw target. After
 * moveTo(double) (and after stop()) the reported preset snaps to the table
 * entry with the closest setpoint, so getPreset() is never stale and a
 * following next() steps from somewhere sensible. The next setPreset(),
 * next(), previous() or cycle() hands control back to the table.
 *
 * Nothing drives until the caller asks: a freshly built mechanism sits at 0 V
 * with the loop disengaged, exactly like a bare PositionMechanism, and the
 * first setPreset() / next() / moveTo() engages it.
 *
 * ### Holding torque
 *
 * The whole PositionMechanismConfig, PositionMechanismConfig::hold_output
 * included, is forwarded to the inner PositionMechanism, so the preset layer
 * changes nothing about how the mechanism holds. With the default
 * hold_output true the inner PID keeps driving after arrival, which is what
 * stops a settled mechanism from sagging; it holds within
 * PositionMechanismConfig::small_error of the setpoint, not exactly on it.
 * With hold_output false the inner loop instead falls to 0 V on arrival and
 * only re-arms once the position has drifted outside that band. For a
 * mechanism that can back-drive, also put the motors in BrakeMode::Hold and
 * route the voltage sink through the motor group so stop() brakes rather than
 * coasts.
 *
 * ```cpp
 * enum class ArmPreset { Down, Load, Score };
 *
 * mclib::device::MotorGroup arm_motors({-11, 12});
 * mclib::device::Rotation arm_sensor(13);
 *
 * mclib::mechanism::PresetPositionMechanism<ArmPreset> arm(
 *     ArmPreset::Down,
 *     {{ArmPreset::Down, 0.0}, {ArmPreset::Load, 45.0}, {ArmPreset::Score, 130.0}},
 *     [&]() { return arm_sensor.getAngleDeg(); },
 *     [&](double volts) { arm_motors.setVoltage(volts); });
 *
 * arm.setName("arm");
 * arm.registerSelf();
 * ```
 */
template <typename StateT>
class PresetPositionMechanism : public MultiPositionMechanism<StateT> {
public:
  using Base = MultiPositionMechanism<StateT>;
  using Entry = typename Base::Entry;
  using Table = typename Base::Table;
  using PositionSource = PositionMechanism::PositionSource;
  using VoltageSink = PositionMechanism::VoltageSink;

  PresetPositionMechanism(StateT initial_preset,
                          std::initializer_list<Entry> presets,
                          PositionSource position_source,
                          VoltageSink voltage_sink,
                          const PositionMechanismConfig& config = {},
                          double default_setpoint = 0.0)
      : PresetPositionMechanism(std::move(initial_preset),
                                Table(presets),
                                std::move(position_source),
                                std::move(voltage_sink),
                                config,
                                default_setpoint) {}

  PresetPositionMechanism(StateT initial_preset,
                          Table presets,
                          PositionSource position_source,
                          VoltageSink voltage_sink,
                          const PositionMechanismConfig& config = {},
                          double default_setpoint = 0.0)
      : Base(std::move(initial_preset),
             std::move(presets),
             [this](double setpoint) { syncTarget(setpoint); },
             default_setpoint),
        m_position(std::move(position_source),
                   std::move(voltage_sink),
                   config) {}

  // The setpoint sink and the command factories capture `this`, and the base
  // does not suppress either operation, so copying or moving would leave the
  // captures pointing at the old object.
  PresetPositionMechanism(const PresetPositionMechanism&) = delete;
  PresetPositionMechanism& operator=(const PresetPositionMechanism&) = delete;
  PresetPositionMechanism(PresetPositionMechanism&&) = delete;
  PresetPositionMechanism& operator=(PresetPositionMechanism&&) = delete;

  /**
   * @brief Drive to @p preset.
   *
   * Always retargets, even when @p preset is already the current one, so this
   * takes control back after a manual-voltage override or a raw target.
   */
  void setPreset(StateT preset) {
    m_engaged = true;
    m_bypass_presets = false;
    m_retarget_pending = true;
    StateMechanism<StateT>::setState(std::move(preset));
    // Retarget now, not on the next tick. A command that finishes on
    // atTarget() polls isFinished() in the same scheduler pass that scheduled
    // it, before any periodic() push, and PositionMechanism latches its
    // arrived flag: a mechanism already settled on the previous preset would
    // report atTarget() and finish the move before it had started.
    syncTarget(this->currentSetpoint());
  }

  /**
   * @brief Drive to @p preset, but do nothing if it is already in charge.
   *
   * setPreset() restarts the loop on every call, which is what makes it take
   * control back from a manual override. That makes it wrong to call every
   * tick: the PID would be reset 50 times a second and never accumulate its
   * dwell. This is the version for commands that execute every frame.
   */
  void holdPreset(StateT preset) {
    if (m_engaged && !m_bypass_presets && this->getState() == preset) {
      return;
    }
    setPreset(std::move(preset));
  }

  /// The preset the mechanism is on, or the one nearest the last raw target.
  const StateT& getPreset() const {
    return this->getState();
  }

  /**
   * @brief Hides Base::setPosition so it goes through setPreset().
   */
  void setPosition(StateT preset) {
    setPreset(std::move(preset));
  }

  /**
   * @brief Hides StateMechanism::setState, which would skip the retarget.
   */
  void setState(StateT preset) {
    setPreset(std::move(preset));
  }

  /**
   * @brief Step forward one preset, clamping at the last entry.
   *
   * At the last entry this does nothing at all: no PID reset, no atTarget()
   * flip, no dwell restart. Same for previous() at the first entry.
   */
  void next() {
    const StateT before = this->getState();
    Base::next();
    afterStep(before);
  }

  /// Step back one preset, clamping at the first entry.
  void previous() {
    const StateT before = this->getState();
    Base::previous();
    afterStep(before);
  }

  /// Step forward one preset, wrapping from the last entry to the first.
  void cycle() {
    const StateT before = this->getState();
    Base::cycle();
    afterStep(before);
  }

  /**
   * @brief Drive to a raw target instead of a preset.
   *
   * The preset table stops pushing until the next setPreset() or step, and the
   * reported preset snaps to whichever table entry has the closest setpoint.
   * With an empty table the reported preset is left alone.
   */
  void moveTo(double target) {
    m_engaged = true;
    m_bypass_presets = true;
    m_retarget_pending = false;
    m_position.moveTo(target);
    snapPresetTo(target);
  }

  /// Current position, in whatever unit the position source reports.
  double positionValue() const {
    return m_position.position();
  }

  /// Target the inner loop is driving to, preset or raw.
  double targetValue() const {
    return m_position.target();
  }

  /// Setpoint of the current preset. Differs from targetValue() under a raw
  /// target or while the mechanism has not been engaged yet.
  double presetSetpoint() const {
    return this->currentSetpoint();
  }

  /**
   * @brief True once the loop has settled on the target.
   *
   * False while a manual voltage is overriding the loop, and false before the
   * mechanism has been given anything to do.
   */
  bool atTarget() const {
    return m_engaged && m_position.atTarget();
  }

  /// True while a manual voltage is overriding the loop.
  bool isManual() const {
    return m_position.isManual();
  }

  /// True while a raw target or a manual voltage has the preset table on hold.
  bool isBypassingPresets() const {
    return m_bypass_presets;
  }

  /**
   * @brief Override the loop and apply @p volts directly.
   *
   * Lasts until the next setPreset(), step, moveTo() or stop(). The preset
   * table is put on hold so it cannot retarget out from under the driver.
   */
  void setManualVoltage(double volts) {
    m_engaged = true;
    m_bypass_presets = true;
    m_retarget_pending = false;
    m_position.setManualVoltage(volts);
  }

  void onDisabled() override {
    setManualVoltage(0.0);
    m_position.onDisabled();
  }

  /**
   * @brief Actively hold the present position.
   *
   * A brake, not a coast: the present position becomes the target, so a raised
   * mechanism does not fall. The reported preset snaps to the nearest entry.
   * Use setManualVoltage(0.0) to go limp instead.
   */
  void stop() {
    m_engaged = true;
    m_bypass_presets = true;
    m_retarget_pending = false;
    m_position.stop();
    snapPresetTo(m_position.target());
  }

  const PositionMechanismConfig& getConfig() const {
    return m_position.getConfig();
  }

  /**
   * @brief The inner closed-loop controller.
   *
   * @warning It is a Subsystem, but it is *this* object's component. Never
   * register it with the scheduler; that would give one mechanism two
   * requirements and let two commands drive it at once.
   */
  PositionMechanism& controller() {
    return m_position;
  }
  const PositionMechanism& controller() const {
    return m_position;
  }

  /**
   * @brief Drive to @p preset and finish on arrival.
   * @param timeout_ms A value above 0.0 also finishes the command after that
   *        many milliseconds, whether or not the mechanism arrived.
   */
  std::unique_ptr<Command> makePresetCommand(StateT preset,
                                             double timeout_ms = 0.0) {
    auto start_time = std::make_shared<double>(0.0);
    return std::make_unique<FunctionalCommand>(
        [this, preset, start_time]() {
          setPreset(preset);
          *start_time = static_cast<double>(mclib::time::millis());
        },
        []() {},
        [this](bool interrupted) {
          if (interrupted) {
            stop();
          }
        },
        [this, start_time, timeout_ms]() {
          const bool timed_out =
              timeout_ms > 0.0 &&
              static_cast<double>(mclib::time::millis()) - *start_time >= timeout_ms;
          return atTarget() || timed_out;
        },
        std::initializer_list<Subsystem*>{this});
  }

  /**
   * @brief Drive to the raw target @p target and finish on arrival.
   * @param timeout_ms A value above 0.0 also finishes the command after that
   *        many milliseconds, whether or not the mechanism arrived.
   */
  std::unique_ptr<Command> makeMoveToCommand(double target,
                                             double timeout_ms = 0.0) {
    auto start_time = std::make_shared<double>(0.0);
    return std::make_unique<FunctionalCommand>(
        [this, target, start_time]() {
          moveTo(target);
          *start_time = static_cast<double>(mclib::time::millis());
        },
        []() {},
        [this](bool interrupted) {
          if (interrupted) {
            stop();
          }
        },
        [this, start_time, timeout_ms]() {
          const bool timed_out =
              timeout_ms > 0.0 &&
              static_cast<double>(mclib::time::millis()) - *start_time >= timeout_ms;
          return atTarget() || timed_out;
        },
        std::initializer_list<Subsystem*>{this});
  }

  /// One-shot command that steps forward, clamping at the last preset.
  std::unique_ptr<Command> makeNextCommand() {
    return this->runOnce([this]() { next(); });
  }

  /// One-shot command that steps back, clamping at the first preset.
  std::unique_ptr<Command> makePreviousCommand() {
    return this->runOnce([this]() { previous(); });
  }

  /// One-shot command that steps forward, wrapping past the last preset.
  std::unique_ptr<Command> makeCycleCommand() {
    return this->runOnce([this]() { cycle(); });
  }

  /// Apply @p volts for as long as the command is scheduled.
  std::unique_ptr<Command> makeManualCommand(double volts) {
    return this->run([this, volts]() { setManualVoltage(volts); });
  }

  /// Actively hold the present position for as long as the command runs.
  std::unique_ptr<Command> makeStopCommand() {
    return this->startEnd([this]() { stop(); }, []() {});
  }

  // The inherited state-command factories would call StateMechanism::setState
  // and so skip the retarget, exactly like the inherited setState. Hidden here
  // so every path into this class goes through setPreset().
  std::unique_ptr<Command> makePositionCommand(StateT preset) {
    return makeStateCommand(std::move(preset));
  }

  std::unique_ptr<Command> makeStateCommand(StateT preset) {
    return this->run([this, preset]() { holdPreset(preset); });
  }

  std::unique_ptr<Command> makeStateOnceCommand(StateT preset) {
    return this->runOnce([this, preset]() { setPreset(preset); });
  }

  std::unique_ptr<Command> makeStateUntilCommand(
      StateT preset, std::function<bool()> is_finished) {
    return this->runUntil([this, preset]() { holdPreset(preset); },
                          std::move(is_finished));
  }

  std::unique_ptr<Command> makeStateForCommand(StateT preset, QTime duration) {
    auto start_time = std::make_shared<QTime>(0.0);
    return std::make_unique<FunctionalCommand>(
        [this, preset, start_time]() {
          setPreset(preset);
          *start_time = mclib::time::now();
        },
        []() {},
        [](bool) {},
        [start_time, duration]() {
          return mclib::time::now() - *start_time >= duration;
        },
        std::initializer_list<Subsystem*>{this});
  }

protected:
  /**
   * @brief One tick: push the preset setpoint, then run the inner loop.
   *
   * The push is skipped while a raw target or a manual voltage is in charge.
   * runPeriodic() is the inner mechanism's non-virtual scheduler entry point;
   * calling it here is what replaces the second registration.
   */
  void applyState(const StateT& preset) override {
    if (!m_bypass_presets) {
      Base::applyState(preset);
    }
    m_position.runPeriodic();
  }

private:
  /// Sink handed to MultiPositionMechanism. Called once per tick.
  void syncTarget(double setpoint) {
    if (!m_engaged) {
      return;
    }
    if (m_retarget_pending || setpoint != m_position.target()) {
      m_position.moveTo(setpoint);
      m_retarget_pending = false;
    }
  }

  /// Shared tail of next()/previous()/cycle().
  void afterStep(const StateT& before) {
    if (before == this->getState()) {
      // Clamped at a rail, or a single-entry table wrapping onto itself.
      if (!m_bypass_presets) {
        // The table was already in charge, so there is nothing to do. In
        // particular do not retarget: the PID keeps its dwell and atTarget()
        // does not flip back to false.
        return;
      }
      // A manual voltage or a raw target was in charge. A step at a rail does
      // not move the mechanism, but it still has to hand control back to the
      // table, or the latched manual voltage would keep driving forever.
    }
    m_engaged = true;
    m_bypass_presets = false;
    m_retarget_pending = true;
    syncTarget(this->currentSetpoint());
  }

  /// Report the table entry whose setpoint is closest to @p value. Ties go to
  /// the earlier entry. Does nothing when the table is empty.
  void snapPresetTo(double value) {
    const std::size_t count = this->positionCount();
    if (count == 0) {
      return;
    }

    std::size_t best = 0;
    double best_error = std::fabs(this->setpointAt(0) - value);
    for (std::size_t i = 1; i < count; ++i) {
      const double error = std::fabs(this->setpointAt(i) - value);
      if (error < best_error) {
        best = i;
        best_error = error;
      }
    }

    // Base setState on purpose: this only relabels which preset is reported,
    // it must not retarget the loop away from the raw target.
    StateMechanism<StateT>::setState(*this->positionAt(best));
  }

  PositionMechanism m_position;
  /// False until the caller commands something. Keeps a fresh mechanism idle.
  bool m_engaged = false;
  /// True while a raw target or a manual voltage holds off the preset table.
  bool m_bypass_presets = false;
  /// Forces the next sink push to retarget even if the setpoint is unchanged.
  bool m_retarget_pending = false;
};

}  // namespace mechanism
}  // namespace mclib
