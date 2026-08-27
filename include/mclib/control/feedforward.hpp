// mclib
#pragma once

#include "mclib/control/profile.hpp"
#include "mclib/pid.hpp"
#include "mclib/telemetry/logger.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>

/**
 * @file feedforward.hpp
 * @brief A kS/kV/kA drivetrain model, a way to identify it, and a follower.
 *
 * Every motion in this library used to be pure PID against a position error,
 * which means the only knob for "go faster" is kp, and the only feedback that
 * a motion is too aggressive is oscillation. Feedforward fixes the asymmetry:
 * the model says what voltage a given velocity and acceleration *should* need,
 * the PID only has to make up the difference, and kp can stay small.
 *
 * The model is the standard permanent-magnet DC one:
 *
 *     V = kS * sgn(v) + kV * v + kA * a
 *
 * - `kS` is the voltage that gets the drivetrain moving at all - static
 *   friction and cogging. Volts.
 * - `kV` is the voltage per unit of steady-state velocity. Volts per (m/s),
 *   though you will read it in volts per (in/s).
 * - `kA` is the voltage per unit of acceleration. Volts per (m/s^2).
 *
 * All three are typed, so handing kV a bare number is a compile error rather
 * than a loop that is 39.37x wrong.
 *
 * **Identifying the gains.** See `fitVelocityGains()` and
 * `fitAccelerationGain()` below for the procedure; there is a worked version
 * in the README. Short form: drive at a series of fixed voltages, record the
 * steady-state speed at each, fit a line, and the intercept is kS while the
 * slope's reciprocal is kV.
 */

namespace mclib {
namespace control {

/// @brief Volts per unit of velocity - the type of kV.
using QVoltagePerVelocity = decltype(units::QVoltage{} / units::QVelocity{});

/// @brief Volts per unit of acceleration - the type of kA.
using QVoltagePerAcceleration = decltype(units::QVoltage{} / units::QAcceleration{});

/// @brief Volts per unit of position error - the type of a distance PID gain.
using QVoltagePerLength = decltype(units::QVoltage{} / units::QLength{});

/// @brief The position loop a ProfileFollower closes: inches in, volts out.
using PositionPID = PIDController<units::QLength, units::QVoltage>;

/**
 * @brief The three constants of the drivetrain voltage model.
 *
 * @details Defaults are all zero, which makes the feedforward contribute
 * nothing. That is deliberate: an unidentified model should be visibly inert
 * rather than quietly wrong.
 */
struct FeedforwardGains {
  /// @brief Volts needed to break static friction. Applied with sgn(v).
  units::QVoltage kS{};
  /// @brief Volts per unit of steady-state velocity.
  QVoltagePerVelocity kV{};
  /// @brief Volts per unit of acceleration.
  QVoltagePerAcceleration kA{};
};

/**
 * @brief Turns a velocity and acceleration into the voltage they should need.
 *
 * @details Stateless and constexpr-friendly; hold one by value anywhere.
 */
class SimpleMotorFeedforward {
 public:
  /// @brief Build with all-zero gains: calculate() returns 0 V.
  constexpr SimpleMotorFeedforward() = default;

  /// @brief Build with an identified model.
  explicit constexpr SimpleMotorFeedforward(const FeedforwardGains& gains) : m_gains(gains) {}

  /// @brief The gains in use.
  constexpr const FeedforwardGains& gains() const { return m_gains; }

  /// @brief Replace the gains.
  constexpr void setGains(const FeedforwardGains& gains) { m_gains = gains; }

  /**
   * @brief The voltage for a velocity and acceleration setpoint.
   *
   * @details `kS` is applied with the sign of @p velocity. When the velocity
   * setpoint is exactly zero the sign of @p acceleration is used instead, so
   * the very first tick of a motion still gets the static term rather than
   * waiting a cycle for the velocity to become non-zero. With both zero the
   * result is 0 V, not +kS - a stationary drivetrain that is meant to stay
   * stationary should not be pushed in an arbitrary direction.
   *
   * @param velocity Commanded velocity.
   * @param acceleration Commanded acceleration.
   * @return The modelled voltage. Not clamped - the caller owns the battery.
   */
  constexpr units::QVoltage calculate(units::QVelocity velocity,
                                      units::QAcceleration acceleration) const {
    double static_sign = units::sign(velocity);
    if (static_sign == 0.0) {
      static_sign = units::sign(acceleration);
    }
    return m_gains.kS * static_sign + m_gains.kV * velocity + m_gains.kA * acceleration;
  }

  /// @brief The steady-state voltage for a velocity, i.e. zero acceleration.
  constexpr units::QVoltage calculate(units::QVelocity velocity) const {
    return calculate(velocity, units::QAcceleration{});
  }

  /// @brief calculate() for a whole profile setpoint.
  constexpr units::QVoltage calculate(const ProfileState& setpoint) const {
    return calculate(setpoint.velocity, setpoint.acceleration);
  }

  /**
   * @brief The fastest the model says the drivetrain can go on @p supply while
   * accelerating at @p acceleration.
   *
   * @details Inverts the model for v. Useful for capping a
   * ProfileConstraints::max_velocity at something the robot can actually hold,
   * instead of a number someone guessed. A negative @p supply gives the
   * reverse limit, with kS signed to match - the model is odd-symmetric, and
   * subtracting kS unsigned there would over-report the reverse speed.
   *
   * @return Zero when kV has not been identified. An unidentified model cannot
   *   answer this, and zero is the answer that fails safe: it feeds straight
   *   into ProfileConstraints, where a returned infinity would become an
   *   unbounded velocity limit.
   */
  constexpr units::QVelocity maxAchievableVelocity(units::QVoltage supply,
                                                   units::QAcceleration acceleration) const {
    if (m_gains.kV.raw() == 0.0) {
      return units::QVelocity{};
    }
    return (supply - m_gains.kS * units::sign(supply) - m_gains.kA * acceleration) / m_gains.kV;
  }

  /**
   * @brief The hardest the model says the drivetrain can accelerate on
   * @p supply while already moving at @p velocity.
   *
   * @details The honest source for ProfileConstraints::max_acceleration, which
   * otherwise tends to be whatever number stopped the wheels slipping. When
   * @p velocity is zero the static term is signed by @p supply, matching
   * calculate().
   *
   * @return Zero when kA has not been identified - which is the common case,
   * since the docs above say a kA you do not trust is worse than none. Same
   * fail-safe reasoning as maxAchievableVelocity().
   */
  constexpr units::QAcceleration maxAchievableAcceleration(units::QVoltage supply,
                                                           units::QVelocity velocity) const {
    if (m_gains.kA.raw() == 0.0) {
      return units::QAcceleration{};
    }
    double static_sign = units::sign(velocity);
    if (static_sign == 0.0) {
      static_sign = units::sign(supply);
    }
    return (supply - m_gains.kS * static_sign - m_gains.kV * velocity) / m_gains.kA;
  }

 private:
  FeedforwardGains m_gains{};
};

// ---------------------------------------------------------------------------
// Characterisation
// ---------------------------------------------------------------------------

/**
 * @brief One steady-state point from a kS/kV identification run.
 *
 * @details "Steady state" means the drivetrain had stopped accelerating: hold
 * a fixed voltage until the speed stops changing, then record both.
 */
struct VelocitySample {
  /// @brief The voltage that was applied.
  units::QVoltage voltage{};
  /// @brief The speed it settled at.
  units::QVelocity velocity{};
};

/**
 * @brief What fitVelocityGains() recovered.
 */
struct VelocityFit {
  /// @brief False when there were not two usable samples at distinct speeds.
  bool valid = false;
  /// @brief The fitted intercept: volts to start moving.
  units::QVoltage kS{};
  /// @brief The fitted slope: volts per unit of velocity.
  QVoltagePerVelocity kV{};
  /**
   * @brief Coefficient of determination, 0 to 1.
   *
   * @details Below about 0.98 something is wrong with the data - samples taken
   * before the speed settled, a battery that sagged during the run, or a
   * drivetrain binding. Look at the run before trusting the gains.
   */
  double r_squared = 0.0;
  /// @brief How many samples cleared the minimum-speed filter.
  std::size_t used = 0;
};

/**
 * @brief Least-squares fit of `V = kS + kV * v` over steady-state samples.
 *
 * @details **The procedure.** With the robot on blocks or on a long clear
 * stretch of field:
 *
 * 1. Command a fixed voltage to both sides of the drivetrain - start around
 *    2 V, which is usually just above kS.
 * 2. Wait for the speed to stop changing. 500 ms is plenty on a VEX drive.
 * 3. Record the commanded voltage and the measured velocity as one
 *    VelocitySample. Take the velocity from odometry or from the drive
 *    encoders through `DriveGeometry::encoderToDistance()`, not from the
 *    motor's own `get_actual_velocity()`, which is already filtered.
 * 4. Repeat in roughly 1 V steps up to 12 V. Eight to twelve points is
 *    plenty.
 * 5. Hand the array here.
 *
 * Do the same run in reverse and append those samples too. Negative-velocity
 * samples are folded onto the positive branch internally - each sample's
 * voltage and velocity are both multiplied by the sign of its velocity - so a
 * bidirectional run fits one symmetric model rather than two half-models.
 *
 * @param samples Steady-state points. May mix both directions.
 * @param count How many.
 * @param min_speed Samples slower than this are dropped. The drivetrain that
 *   did not move at 1 V carries no information about the slope and would drag
 *   the intercept down; the default of 1 in/s is well below any real cruise
 *   speed and well above encoder noise.
 * @return The fit. `valid` is false when fewer than two samples survive or
 *   they all sit at the same speed.
 */
VelocityFit fitVelocityGains(const VelocitySample* samples, std::size_t count,
                             units::QVelocity min_speed = 1 * units::inps);

/**
 * @brief One point from a kA identification run.
 */
struct AccelerationSample {
  /// @brief The voltage that was applied.
  units::QVoltage voltage{};
  /// @brief The velocity at that instant.
  units::QVelocity velocity{};
  /// @brief The acceleration at that instant.
  units::QAcceleration acceleration{};
};

/**
 * @brief What fitAccelerationGain() recovered.
 */
struct AccelerationFit {
  /// @brief False when no sample had a usable acceleration.
  bool valid = false;
  /// @brief The fitted slope: volts per unit of acceleration.
  QVoltagePerAcceleration kA{};
  /// @brief How many samples cleared the minimum-acceleration filter.
  std::size_t used = 0;
};

/**
 * @brief Recover kA from a run where the drivetrain was accelerating.
 *
 * @details **The procedure**, once kS and kV are known:
 *
 * 1. Ramp the voltage linearly from 0 to 12 V over about two seconds while
 *    logging voltage, velocity and acceleration every tick.
 * 2. Acceleration comes from differencing the velocity. Filter it - a raw
 *    difference of encoder velocity is mostly noise. A simple three-point
 *    central difference over a 10 ms loop is usually enough.
 * 3. Hand the samples and the already-known kS/kV here.
 *
 * The fit is a least-squares line through the origin of the leftover voltage
 * `V - kS*sgn(v) - kV*v` against acceleration, because the model says that
 * residual is exactly `kA * a`. Fitting through the origin rather than fitting
 * an intercept is the point: an intercept here would silently absorb an error
 * in kS.
 *
 * kA is the least important of the three and the hardest to measure. A model
 * with kA at zero still removes most of the work from the PID; if the ramp
 * data is noisy, leaving kA at zero beats a fitted value you do not trust.
 *
 * @param samples The ramp.
 * @param count How many.
 * @param known kS and kV from fitVelocityGains(). Its kA field is ignored.
 * @param min_acceleration Samples below this are dropped; near-zero
 *   acceleration carries no information about the slope and amplifies noise.
 * @return The fit. `valid` is false when no sample survives.
 */
AccelerationFit fitAccelerationGain(const AccelerationSample* samples, std::size_t count,
                                    const FeedforwardGains& known,
                                    units::QAcceleration min_acceleration = 1 * inps2);

// ---------------------------------------------------------------------------
// Follower
// ---------------------------------------------------------------------------

/**
 * @brief Setup for a ProfileFollower.
 */
struct ProfileFollowerConfig {
  /// @brief The identified drivetrain model. Zeroes mean pure feedback.
  FeedforwardGains gains{};

  /**
   * @brief Proportional gain on the position error, volts per unit of length.
   *
   * @details With a decent feedforward this is small - it only has to correct
   * modelling error, not drive the motion. Start at a value that produces
   * about 1 V at 1 inch of error and go from there.
   */
  QVoltagePerLength kp{};
  /// @brief Integral gain. Usually zero; feedforward removes the steady offset.
  QVoltagePerLength ki{};
  /// @brief Derivative gain on the position error.
  QVoltagePerLength kd{};

  /**
   * @brief Clamp on the integral term's contribution.
   *
   * @details 2 V by default rather than PID's unbounded 0, because a follower
   * that stalls against a wall would otherwise wind up the whole battery.
   */
  units::QVoltage integral_max = 2 * units::volt;

  /// @brief Clamp on the total output. 12 V is the V5 supply.
  units::QVoltage max_voltage = 12 * units::volt;
};

/**
 * @brief Drives a MotionProfile: feedforward from the setpoint, PID on the
 * error between the setpoint and where the robot actually is.
 *
 * @details Holds the profile by value and knows when the motion started, so a
 * control loop is:
 *
 * @code
 * mclib::control::ProfileFollower follower({
 *     .gains = {.kS = 0.9_V, .kV = kv, .kA = ka},
 *     .kp = 1.5 * mclib::units::volt / mclib::units::inch,
 * });
 * follower.follow(MotionProfile::generate(48_in, limits));
 * const QLength start = travelledDistance();
 * while (!follower.isFinished()) {
 *   const QVoltage command = follower.update(travelledDistance() - start);
 *   driveVoltage(command, command);
 *   pros::delay(10);
 * }
 * @endcode
 *
 * The PID inside runs with `setUseDt(true)` - this is new code, so there are
 * no gains fitted against the historical raw-delta numerics to protect, and
 * real rates are what the gains should mean. It also runs with
 * `setArrive(false)`: a profile ends when the profile ends, not when the error
 * settles, and a latched arrival mid-motion would zero the output while the
 * robot was still moving.
 */
class ProfileFollower {
 public:
  /// @brief Build from a config. Defaults give a pure-feedforward follower.
  explicit ProfileFollower(const ProfileFollowerConfig& config = {});

  /// @brief The position loop, for tuning knobs the config does not expose.
  PositionPID& pid() { return m_pid; }
  /// @brief The position loop.
  const PositionPID& pid() const { return m_pid; }

  /// @brief The feedforward model in use.
  const FeedforwardGains& gains() const { return m_feedforward.gains(); }
  /// @brief Replace the feedforward model, e.g. after a fresh characterisation.
  void setGains(const FeedforwardGains& gains) { m_feedforward.setGains(gains); }
  /// @brief Replace the output clamp.
  void setMaxVoltage(units::QVoltage max_voltage) { m_max_voltage = max_voltage; }

  /**
   * @brief Start following @p profile, timed from @p start_time.
   *
   * @details Resets the PID, so a follower is reusable across motions.
   */
  void follow(const MotionProfile& profile, units::QTime start_time);

  /// @brief follow() timed from `mclib::time::now()`.
  void follow(const MotionProfile& profile);

  /// @brief Drop the profile and the PID state. update() then returns 0 V.
  void reset();

  /// @brief The profile being followed.
  const MotionProfile& profile() const { return m_profile; }

  /// @brief How far into the profile @p now is.
  units::QTime elapsed(units::QTime now) const;

  /// @brief Whether the profile has run out at @p now.
  bool isFinished(units::QTime now) const;
  /// @brief isFinished() against `mclib::time::now()`.
  bool isFinished() const;

  /**
   * @brief One control tick.
   *
   * @param measured_position Displacement travelled since the motion started,
   *   in the same frame and sign convention as the profile. Subtract the
   *   position at follow() time; the profile always starts from zero.
   * @param now The current time.
   * @return The commanded voltage, clamped to +/- max_voltage.
   */
  units::QVoltage update(units::QLength measured_position, units::QTime now);

  /// @brief update() against `mclib::time::now()`.
  units::QVoltage update(units::QLength measured_position);

  /**
   * @brief One control tick against a setpoint the caller sampled itself.
   *
   * @details The seam a path follower wants: it already knows the velocity it
   * needs at this point on the path, and does not want this class's stopwatch.
   *
   * @param setpoint Where the robot should be and how fast.
   * @param measured_position Where it actually is.
   * @param dt Time since the previous tick, for the PID's rate terms.
   * @return The commanded voltage, clamped to +/- max_voltage.
   */
  units::QVoltage calculate(const ProfileState& setpoint, units::QLength measured_position,
                            units::QTime dt);

  /// @brief The setpoint the last tick used.
  const ProfileState& setpoint() const { return m_setpoint; }
  /// @brief The feedforward half of the last tick's output, before clamping.
  units::QVoltage lastFeedforward() const { return m_last_feedforward; }
  /// @brief The feedback half of the last tick's output, before clamping.
  units::QVoltage lastFeedback() const { return m_last_feedback; }
  /// @brief The position error the last tick saw.
  units::QLength lastError() const { return m_last_error; }
  /// @brief The voltage the last tick returned.
  units::QVoltage lastOutput() const { return m_last_output; }

  /**
   * @brief Register telemetry columns and write them on every update().
   *
   * @details Six columns: setpoint position and velocity, measured position,
   * error, and the feedforward / feedback split of the command. That split is
   * the whole tuning story - a feedback term that is large compared with the
   * feedforward means the model is wrong, not that kp needs raising.
   *
   * The follower never calls sample(); the control loop owns the cadence.
   * Pass a null logger to detach.
   *
   * @param logger The logger, or nullptr to stop logging. Must outlive this.
   * @param prefix Prepended to each column name with an underscore.
   */
  void attachTelemetry(telemetry::Logger* logger, const char* prefix = "profile");

 private:
  SimpleMotorFeedforward m_feedforward{};
  PositionPID m_pid;
  units::QVoltage m_max_voltage;
  MotionProfile m_profile{};
  units::QTime m_start_time{};
  units::QTime m_last_tick{};
  bool m_running = false;
  bool m_first_tick = true;

  ProfileState m_setpoint{};
  units::QVoltage m_last_feedforward{};
  units::QVoltage m_last_feedback{};
  units::QVoltage m_last_output{};
  units::QLength m_last_error{};

  telemetry::Logger* m_logger = nullptr;
  telemetry::Channel<units::QLength> m_ch_setpoint_position{};
  telemetry::Channel<units::QVelocity> m_ch_setpoint_velocity{};
  telemetry::Channel<units::QLength> m_ch_measured_position{};
  telemetry::Channel<units::QLength> m_ch_error{};
  telemetry::Channel<units::QVoltage> m_ch_feedforward{};
  telemetry::Channel<units::QVoltage> m_ch_feedback{};
};

}  // namespace control
}  // namespace mclib
