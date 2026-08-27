// mclib
#pragma once

#include "mclib/chassis/chassis_controller.hpp"
#include "mclib/command/command.h"
#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace mclib {
namespace auton {

/// @brief A field position. Compass frame, origin and axes per `mclib/math.hpp`.
struct Point {
  QLength x{};
  QLength y{};

  constexpr Point() = default;
  constexpr Point(QLength x_in, QLength y_in) : x(x_in), y(y_in) {}
  /// @brief From an odometry vector, whose components are inches.
  explicit Point(const Vec2& point)
      : x(point.x() * units::inch), y(point.y() * units::inch) {}
};

/**
 * @brief A sequence of motions and commands, built by chaining calls.
 *
 * Every motion returns a MotionStep, whose `withXxx()` setters rebuild that
 * step in place, so an autonomous reads as one expression:
 *
 * @code
 * routine.driveTo(24_in, 1_s).withMaxVoltage(8_V)
 *        .turnToAngle(90_deg, 800_ms)
 *        .moveToPoint(Point{24_in, 36_in}, 1, 1500_ms).withoutExit();
 * @endcode
 */
class Routine : public Command {
public:
  class MotionStep {
  public:
    /// @brief Voltage cap for this step.
    MotionStep& withMaxVoltage(QVoltage max_voltage);
    /// @brief Voltage floor for this step; negative selects the `min_output` default.
    MotionStep& withMinVoltage(QVoltage min_voltage);
    MotionStep& withExit(bool exit = true);
    MotionStep& withoutExit();
    MotionStep& withStop(bool stop = true);
    MotionStep& withoutStop();
    MotionStep& withDirection(int direction);
    MotionStep& reversed(bool reverse = true);
    MotionStep& withOverturn(bool overturn = true);
    MotionStep& withoutOverturn();
    /// @brief Voltage `wallReset()` pushes into the wall with. Negative reverses.
    MotionStep& withDrivePower(QVoltage drive_power);
    /// @brief Motor current above which `wallReset()` calls it a stall.
    MotionStep& withCurrentThreshold(QCurrent current_threshold);
    /// @brief Motor speed below which `wallReset()` calls it a stall.
    MotionStep& withVelocityThreshold(QAngularVelocity velocity_threshold);

    Routine& done();
    Routine& add(std::unique_ptr<Command> command);
    Routine& then(std::unique_ptr<Command> command);
    Routine& trigger(std::unique_ptr<Command> command);
    Routine& runOnce(std::function<void()> action);
    Routine& wait(QTime duration);
    Routine& waitUntil(std::function<bool()> condition);

    MotionStep driveDistance(QLength distance, QTime timeout = 0.0 * units::second);
    MotionStep turnToHeading(QAngle heading, QTime timeout = 0.0 * units::second);
    MotionStep turnToAngle(QAngle turn_angle, QTime timeout);
    MotionStep driveTo(QLength distance, QTime timeout);
    MotionStep curveCircle(QAngle result_angle, QLength center_radius, QTime timeout);
    MotionStep curveCircleReverse(QAngle result_angle,
                                  QLength center_radius,
                                  QTime timeout);
    MotionStep swing(QAngle swing_angle, double drive_direction, QTime timeout);
    MotionStep wallReset(QLength reset_x,
                         QLength reset_y,
                         QAngle reset_heading,
                         QTime timeout);
    MotionStep turnToPoint(Point point, QTime timeout = 1000.0 * units::millisecond);
    MotionStep turnToPoint(QLength x,
                           QLength y,
                           QTime timeout = 1000.0 * units::millisecond);
    MotionStep moveToPoint(Point point, QTime timeout);
    MotionStep moveToPoint(Point point, int dir, QTime timeout);
    MotionStep moveToPoint(QLength x, QLength y, int dir, QTime timeout);
    MotionStep boomerang(Point point,
                         int dir,
                         QAngle final_heading,
                         double lead,
                         QTime timeout);
    MotionStep boomerang(QLength x,
                         QLength y,
                         int dir,
                         QAngle final_heading,
                         double lead,
                         QTime timeout);

  private:
    friend class Routine;

    MotionStep(Routine& routine, std::size_t index);
    MotionStep& rebuild();

    Routine* m_routine = nullptr;
    std::size_t m_index = 0;
  };

  Routine() = default;
  explicit Routine(ChassisController& chassis);

  Routine& setChassis(ChassisController& chassis);

  Routine& add(std::unique_ptr<Command> command);
  Routine& then(std::unique_ptr<Command> command);
  Routine& trigger(std::unique_ptr<Command> command);
  Routine& runOnce(std::function<void()> action);
  Routine& wait(QTime duration);
  Routine& waitUntil(std::function<bool()> condition);

  MotionStep driveDistance(ChassisController& chassis,
                           QLength distance,
                           QTime timeout = 0.0 * units::second);
  MotionStep driveDistance(QLength distance, QTime timeout = 0.0 * units::second);
  MotionStep turnToHeading(ChassisController& chassis,
                           QAngle heading,
                           QTime timeout = 0.0 * units::second);
  MotionStep turnToHeading(QAngle heading, QTime timeout = 0.0 * units::second);
  MotionStep turnToAngle(ChassisController& chassis, QAngle turn_angle, QTime timeout);
  MotionStep turnToAngle(QAngle turn_angle, QTime timeout);

  /**
   * @brief Drive a relative distance in a straight line.
   *
   * This used to be overloaded three ways: `driveTo(24, 1000)` drove 24 inches
   * forward while `driveTo(24, 36, 1000)` drove to the field point (24, 36) -
   * two different motions, told apart only by argument count. There is now one
   * `driveTo`, and it always means a distance. For a field point, say so:
   * `moveToPoint(Point{24_in, 36_in}, 1500_ms)`, which is what the coordinate
   * overloads dispatched to anyway.
   */
  MotionStep driveTo(ChassisController& chassis, QLength distance, QTime timeout);
  MotionStep driveTo(QLength distance, QTime timeout);

  MotionStep curveCircle(ChassisController& chassis,
                         QAngle result_angle,
                         QLength center_radius,
                         QTime timeout);
  MotionStep curveCircle(QAngle result_angle, QLength center_radius, QTime timeout);
  MotionStep curveCircleReverse(ChassisController& chassis,
                                QAngle result_angle,
                                QLength center_radius,
                                QTime timeout);
  MotionStep curveCircleReverse(QAngle result_angle,
                                QLength center_radius,
                                QTime timeout);
  MotionStep swing(ChassisController& chassis,
                   QAngle swing_angle,
                   double drive_direction,
                   QTime timeout);
  MotionStep swing(QAngle swing_angle, double drive_direction, QTime timeout);
  MotionStep wallReset(ChassisController& chassis,
                       QLength reset_x,
                       QLength reset_y,
                       QAngle reset_heading,
                       QTime timeout);
  MotionStep wallReset(QLength reset_x,
                       QLength reset_y,
                       QAngle reset_heading,
                       QTime timeout);
  MotionStep turnToPoint(ChassisController& chassis,
                         Point point,
                         QTime timeout = 1000.0 * units::millisecond);
  MotionStep turnToPoint(Point point, QTime timeout = 1000.0 * units::millisecond);
  MotionStep turnToPoint(ChassisController& chassis,
                         QLength x,
                         QLength y,
                         QTime timeout = 1000.0 * units::millisecond);
  MotionStep turnToPoint(QLength x,
                         QLength y,
                         QTime timeout = 1000.0 * units::millisecond);

  /// @brief Drive to a field point, direction taken from the step's options.
  MotionStep moveToPoint(ChassisController& chassis, Point point, QTime timeout);
  MotionStep moveToPoint(Point point, QTime timeout);
  MotionStep moveToPoint(ChassisController& chassis,
                         Point point,
                         int dir,
                         QTime timeout);
  MotionStep moveToPoint(Point point, int dir, QTime timeout);
  MotionStep moveToPoint(ChassisController& chassis,
                         QLength x,
                         QLength y,
                         int dir,
                         QTime timeout);
  MotionStep moveToPoint(QLength x, QLength y, int dir, QTime timeout);
  MotionStep boomerang(ChassisController& chassis,
                       Point point,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);
  MotionStep boomerang(Point point,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);
  MotionStep boomerang(ChassisController& chassis,
                       QLength x,
                       QLength y,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);
  MotionStep boomerang(QLength x,
                       QLength y,
                       int dir,
                       QAngle final_heading,
                       double lead,
                       QTime timeout);

  void clear();
  bool empty() const;
  std::size_t size() const;
  void runBlocking(std::uint32_t period_ms = 10);

  void initialize() override;
  void execute() override;
  bool isFinished() override;
  void end(bool interrupted) override;
  std::vector<Subsystem*> getRequirements() override;

private:
  /**
   * @brief The per-step knobs the `withXxx()` setters write.
   *
   * `max_voltage` was called `max_speed`, and `withMaxSpeed()` was literally
   * `return withMaxVoltage(...)`. It has always been volts - it lands in
   * `MotorGroup::setVoltage()` - so it is now named and typed for what it is.
   * The same goes for `min_voltage`, formerly `min_speed`.
   */
  struct MotionOptions {
    QVoltage max_voltage = 12.0 * units::volt;
    /// @brief Negative selects the `min_output` default. See motion.hpp.
    QVoltage min_voltage = -1.0 * units::volt;
    bool exit = true;
    bool stop_at_end = true;
    int direction = 1;
    bool reverse = false;
    bool overturn = true;
    /// @brief Negative drives backward into the wall, which is the usual case.
    QVoltage drive_power = -4.0 * units::volt;
    QCurrent current_threshold = 2500.0 * units::milliampere;
    QAngularVelocity velocity_threshold = 5.0 * units::rpm;
  };

  using MotionFactory =
      std::function<std::unique_ptr<Command>(const MotionOptions&)>;

  struct Step {
    std::unique_ptr<Command> command;
    bool reserve_requirements = true;
    MotionFactory factory;
    MotionOptions options;
  };

  ChassisController& requireChassis() const;
  MotionStep addMotion(MotionFactory factory);
  MotionStep addMotion(MotionFactory factory, MotionOptions options);
  void rebuildMotion(std::size_t index);
  void initializeCurrent();
  static void mergeRequirements(std::vector<Subsystem*>& requirements,
                                const std::vector<Subsystem*>& next);

  ChassisController* m_chassis = nullptr;
  std::vector<Step> m_steps;
  std::size_t m_index = 0;
  bool m_current_initialized = false;
};

using AutonomousRoutine = Routine;

}  // namespace auton
}  // namespace mclib
