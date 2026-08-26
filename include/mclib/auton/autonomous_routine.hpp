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

struct Point {
  double x = 0.0;
  double y = 0.0;

  constexpr Point() = default;
  constexpr Point(double x_in, double y_in) : x(x_in), y(y_in) {}
  explicit Point(const Vec2& point) : x(point.x()), y(point.y()) {}
};

class Routine : public Command {
public:
  class MotionStep {
  public:
    MotionStep& withMaxSpeed(double max_speed);
    MotionStep& withMaxVoltage(double max_voltage);
    MotionStep& withMinSpeed(double min_speed);
    MotionStep& withExit(bool exit = true);
    MotionStep& withoutExit();
    MotionStep& withStop(bool stop = true);
    MotionStep& withoutStop();
    MotionStep& withDirection(int direction);
    MotionStep& reversed(bool reverse = true);
    MotionStep& withOverturn(bool overturn = true);
    MotionStep& withoutOverturn();
    MotionStep& withDrivePower(double drive_power);
    MotionStep& withCurrentThreshold(double current_threshold);
    MotionStep& withVelocityThreshold(double velocity_threshold);

    Routine& done();
    Routine& add(std::unique_ptr<Command> command);
    Routine& then(std::unique_ptr<Command> command);
    Routine& trigger(std::unique_ptr<Command> command);
    Routine& runOnce(std::function<void()> action);
    Routine& wait(QTime duration);
    Routine& waitUntil(std::function<bool()> condition);

    MotionStep driveDistance(double distance_in, double timeout_ms = 0.0);
    MotionStep turnToHeading(double heading_deg, double timeout_ms = 0.0);
    MotionStep turnToAngle(double turn_angle, double timeout_ms);
    MotionStep driveTo(double distance_in, double timeout_ms);
    MotionStep driveTo(Point point, double timeout_ms);
    MotionStep driveTo(double x, double y, double timeout_ms);
    MotionStep curveCircle(double result_angle_deg,
                           double center_radius,
                           double timeout_ms);
    MotionStep curveCircleReverse(double result_angle_deg,
                                  double center_radius,
                                  double timeout_ms);
    MotionStep swing(double swing_angle,
                     double drive_direction,
                     double timeout_ms);
    MotionStep wallReset(double reset_x,
                         double reset_y,
                         double reset_heading,
                         double timeout_ms);
    MotionStep turnToPoint(Point point, double timeout_ms = 1000.0);
    MotionStep turnToPoint(double x, double y, double timeout_ms = 1000.0);
    MotionStep moveToPoint(Point point, int dir, double timeout_ms);
    MotionStep moveToPoint(double x, double y, int dir, double timeout_ms);
    MotionStep boomerang(Point point,
                         int dir,
                         double final_heading_deg,
                         double lead,
                         double timeout_ms);
    MotionStep boomerang(double x,
                         double y,
                         int dir,
                         double final_heading_deg,
                         double lead,
                         double timeout_ms);

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
                           double distance_in,
                           double timeout_ms = 0.0);
  MotionStep driveDistance(double distance_in, double timeout_ms = 0.0);
  MotionStep turnToHeading(ChassisController& chassis,
                           double heading_deg,
                           double timeout_ms = 0.0);
  MotionStep turnToHeading(double heading_deg, double timeout_ms = 0.0);
  MotionStep turnToAngle(ChassisController& chassis,
                         double turn_angle,
                         double timeout_ms);
  MotionStep turnToAngle(double turn_angle, double timeout_ms);
  MotionStep driveTo(ChassisController& chassis,
                     double distance_in,
                     double timeout_ms);
  MotionStep driveTo(double distance_in, double timeout_ms);
  MotionStep driveTo(ChassisController& chassis,
                     Point point,
                     double timeout_ms);
  MotionStep driveTo(Point point, double timeout_ms);
  MotionStep driveTo(ChassisController& chassis,
                     double x,
                     double y,
                     double timeout_ms);
  MotionStep driveTo(double x, double y, double timeout_ms);
  MotionStep curveCircle(ChassisController& chassis,
                         double result_angle_deg,
                         double center_radius,
                         double timeout_ms);
  MotionStep curveCircle(double result_angle_deg,
                         double center_radius,
                         double timeout_ms);
  MotionStep curveCircleReverse(ChassisController& chassis,
                                double result_angle_deg,
                                double center_radius,
                                double timeout_ms);
  MotionStep curveCircleReverse(double result_angle_deg,
                                double center_radius,
                                double timeout_ms);
  MotionStep swing(ChassisController& chassis,
                   double swing_angle,
                   double drive_direction,
                   double timeout_ms);
  MotionStep swing(double swing_angle,
                   double drive_direction,
                   double timeout_ms);
  MotionStep wallReset(ChassisController& chassis,
                       double reset_x,
                       double reset_y,
                       double reset_heading,
                       double timeout_ms);
  MotionStep wallReset(double reset_x,
                       double reset_y,
                       double reset_heading,
                       double timeout_ms);
  MotionStep turnToPoint(ChassisController& chassis,
                         Point point,
                         double timeout_ms = 1000.0);
  MotionStep turnToPoint(Point point, double timeout_ms = 1000.0);
  MotionStep turnToPoint(ChassisController& chassis,
                         double x,
                         double y,
                         double timeout_ms = 1000.0);
  MotionStep turnToPoint(double x, double y, double timeout_ms = 1000.0);
  MotionStep moveToPoint(ChassisController& chassis,
                         Point point,
                         int dir,
                         double timeout_ms);
  MotionStep moveToPoint(Point point, int dir, double timeout_ms);
  MotionStep moveToPoint(ChassisController& chassis,
                         double x,
                         double y,
                         int dir,
                         double timeout_ms);
  MotionStep moveToPoint(double x, double y, int dir, double timeout_ms);
  MotionStep boomerang(ChassisController& chassis,
                       Point point,
                       int dir,
                       double final_heading_deg,
                       double lead,
                       double timeout_ms);
  MotionStep boomerang(Point point,
                       int dir,
                       double final_heading_deg,
                       double lead,
                       double timeout_ms);
  MotionStep boomerang(ChassisController& chassis,
                       double x,
                       double y,
                       int dir,
                       double final_heading_deg,
                       double lead,
                       double timeout_ms);
  MotionStep boomerang(double x,
                       double y,
                       int dir,
                       double final_heading_deg,
                       double lead,
                       double timeout_ms);

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
  struct MotionOptions {
    double max_speed = 12.0;
    double min_speed = -1.0;
    bool exit = true;
    bool stop_at_end = true;
    int direction = 1;
    bool reverse = false;
    bool overturn = true;
    double drive_power = -4.0;
    double current_threshold = 2500.0;
    double velocity_threshold = 5.0;
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
