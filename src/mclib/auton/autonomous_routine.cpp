// mclib
#include "mclib/auton/autonomous_routine.hpp"

#include "mclib/command/commandScheduler.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/waitCommand.h"
#include "mclib/command/waitUntilCommand.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cassert>
#include <utility>

namespace mclib {
namespace auton {

namespace {
class TriggerCommand : public Command {
public:
  explicit TriggerCommand(std::unique_ptr<Command> command)
      : m_command(std::move(command)) {}

  void initialize() override {
    if (m_command != nullptr) {
      m_command->schedule();
    }
  }

  bool isFinished() override {
    return true;
  }

  ~TriggerCommand() override {
    if (m_command != nullptr && m_command->scheduled()) {
      m_command->cancel();
    }
  }

private:
  std::unique_ptr<Command> m_command;
};
}  // namespace

Routine::MotionStep::MotionStep(Routine& routine, std::size_t index)
    : m_routine(&routine), m_index(index) {}

Routine::MotionStep& Routine::MotionStep::withMaxSpeed(double max_speed) {
  return withMaxVoltage(max_speed);
}

Routine::MotionStep& Routine::MotionStep::withMaxVoltage(double max_voltage) {
  m_routine->m_steps[m_index].options.max_speed = max_voltage;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withMinSpeed(double min_speed) {
  m_routine->m_steps[m_index].options.min_speed = min_speed;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withExit(bool exit) {
  m_routine->m_steps[m_index].options.exit = exit;
  if (!exit) {
    m_routine->m_steps[m_index].options.stop_at_end = false;
  }
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withoutExit() {
  return withExit(false);
}

Routine::MotionStep& Routine::MotionStep::withStop(bool stop) {
  m_routine->m_steps[m_index].options.stop_at_end = stop;
  if (!stop) {
    m_routine->m_steps[m_index].options.exit = false;
  }
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withoutStop() {
  return withStop(false);
}

Routine::MotionStep& Routine::MotionStep::withDirection(int direction) {
  m_routine->m_steps[m_index].options.direction = direction >= 0 ? 1 : -1;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::reversed(bool reverse) {
  m_routine->m_steps[m_index].options.reverse = reverse;
  if (reverse) {
    m_routine->m_steps[m_index].options.direction = -1;
  }
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withOverturn(bool overturn) {
  m_routine->m_steps[m_index].options.overturn = overturn;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withoutOverturn() {
  return withOverturn(false);
}

Routine::MotionStep& Routine::MotionStep::withDrivePower(double drive_power) {
  m_routine->m_steps[m_index].options.drive_power = drive_power;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withCurrentThreshold(
    double current_threshold) {
  m_routine->m_steps[m_index].options.current_threshold = current_threshold;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withVelocityThreshold(
    double velocity_threshold) {
  m_routine->m_steps[m_index].options.velocity_threshold = velocity_threshold;
  return rebuild();
}

Routine& Routine::MotionStep::done() {
  return *m_routine;
}

Routine& Routine::MotionStep::add(std::unique_ptr<Command> command) {
  return m_routine->add(std::move(command));
}

Routine& Routine::MotionStep::then(std::unique_ptr<Command> command) {
  return m_routine->then(std::move(command));
}

Routine& Routine::MotionStep::trigger(std::unique_ptr<Command> command) {
  return m_routine->trigger(std::move(command));
}

Routine& Routine::MotionStep::runOnce(std::function<void()> action) {
  return m_routine->runOnce(std::move(action));
}

Routine& Routine::MotionStep::wait(QTime duration) {
  return m_routine->wait(duration);
}

Routine& Routine::MotionStep::waitUntil(std::function<bool()> condition) {
  return m_routine->waitUntil(std::move(condition));
}

Routine::MotionStep Routine::MotionStep::driveDistance(double distance_in,
                                                       double timeout_ms) {
  return m_routine->driveDistance(distance_in, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::turnToHeading(double heading_deg,
                                                       double timeout_ms) {
  return m_routine->turnToHeading(heading_deg, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::turnToAngle(double turn_angle,
                                                     double timeout_ms) {
  return m_routine->turnToAngle(turn_angle, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::driveTo(double distance_in,
                                                 double timeout_ms) {
  return m_routine->driveTo(distance_in, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::driveTo(Point point,
                                                 double timeout_ms) {
  return m_routine->driveTo(point, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::driveTo(double x,
                                                 double y,
                                                 double timeout_ms) {
  return m_routine->driveTo(x, y, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::curveCircle(
    double result_angle_deg,
    double center_radius,
    double timeout_ms) {
  return m_routine->curveCircle(result_angle_deg, center_radius, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::curveCircleReverse(
    double result_angle_deg,
    double center_radius,
    double timeout_ms) {
  return m_routine->curveCircleReverse(result_angle_deg,
                                       center_radius,
                                       timeout_ms);
}

Routine::MotionStep Routine::MotionStep::swing(double swing_angle,
                                               double drive_direction,
                                               double timeout_ms) {
  return m_routine->swing(swing_angle, drive_direction, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::wallReset(double reset_x,
                                                   double reset_y,
                                                   double reset_heading,
                                                   double timeout_ms) {
  return m_routine->wallReset(reset_x, reset_y, reset_heading, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::turnToPoint(Point point,
                                                     double timeout_ms) {
  return m_routine->turnToPoint(point, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::turnToPoint(double x,
                                                     double y,
                                                     double timeout_ms) {
  return m_routine->turnToPoint(x, y, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::moveToPoint(Point point,
                                                     int dir,
                                                     double timeout_ms) {
  return m_routine->moveToPoint(point, dir, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::moveToPoint(double x,
                                                     double y,
                                                     int dir,
                                                     double timeout_ms) {
  return m_routine->moveToPoint(x, y, dir, timeout_ms);
}

Routine::MotionStep Routine::MotionStep::boomerang(
    Point point,
    int dir,
    double final_heading_deg,
    double lead,
    double timeout_ms) {
  return m_routine->boomerang(point,
                              dir,
                              final_heading_deg,
                              lead,
                              timeout_ms);
}

Routine::MotionStep Routine::MotionStep::boomerang(
    double x,
    double y,
    int dir,
    double final_heading_deg,
    double lead,
    double timeout_ms) {
  return m_routine->boomerang(x,
                              y,
                              dir,
                              final_heading_deg,
                              lead,
                              timeout_ms);
}

Routine::MotionStep& Routine::MotionStep::rebuild() {
  m_routine->rebuildMotion(m_index);
  return *this;
}

Routine::Routine(ChassisController& chassis) : m_chassis(&chassis) {}

Routine& Routine::setChassis(ChassisController& chassis) {
  m_chassis = &chassis;
  return *this;
}

Routine& Routine::add(std::unique_ptr<Command> command) {
  if (command != nullptr) {
    m_steps.push_back({std::move(command), true, {}, {}});
  }
  return *this;
}

Routine& Routine::then(std::unique_ptr<Command> command) {
  return add(std::move(command));
}

Routine& Routine::trigger(std::unique_ptr<Command> command) {
  if (command != nullptr) {
    m_steps.push_back(
        {std::make_unique<TriggerCommand>(std::move(command)), false, {}, {}});
  }
  return *this;
}

Routine& Routine::runOnce(std::function<void()> action) {
  return add(std::make_unique<InstantCommand>(
      std::move(action),
      std::initializer_list<Subsystem*>{}));
}

Routine& Routine::wait(QTime duration) {
  return add(std::make_unique<WaitCommand>(duration));
}

Routine& Routine::waitUntil(std::function<bool()> condition) {
  return add(std::make_unique<WaitUntilCommand>(std::move(condition)));
}

Routine::MotionStep Routine::driveDistance(ChassisController& chassis,
                                           double distance_in,
                                           double timeout_ms) {
  return addMotion(
      [&chassis, distance_in, timeout_ms](const MotionOptions& options) {
        return chassis.makeDriveDistanceCommand(distance_in,
                                                timeout_ms,
                                                options.stop_at_end,
                                                options.max_speed);
      });
}

Routine::MotionStep Routine::driveDistance(double distance_in,
                                           double timeout_ms) {
  return driveDistance(requireChassis(), distance_in, timeout_ms);
}

Routine::MotionStep Routine::turnToHeading(ChassisController& chassis,
                                           double heading_deg,
                                           double timeout_ms) {
  return addMotion(
      [&chassis, heading_deg, timeout_ms](const MotionOptions& options) {
        return chassis.makeTurnToHeadingCommand(heading_deg,
                                                timeout_ms,
                                                options.stop_at_end,
                                                options.max_speed);
      });
}

Routine::MotionStep Routine::turnToHeading(double heading_deg,
                                           double timeout_ms) {
  return turnToHeading(requireChassis(), heading_deg, timeout_ms);
}

Routine::MotionStep Routine::turnToAngle(ChassisController& chassis,
                                         double turn_angle,
                                         double timeout_ms) {
  return addMotion(
      [&chassis, turn_angle, timeout_ms](const MotionOptions& options) {
        return chassis.makeTurnToAngleCommand(turn_angle,
                                              timeout_ms,
                                              options.exit,
                                              options.max_speed,
                                              options.min_speed);
      });
}

Routine::MotionStep Routine::turnToAngle(double turn_angle,
                                         double timeout_ms) {
  return turnToAngle(requireChassis(), turn_angle, timeout_ms);
}

Routine::MotionStep Routine::driveTo(ChassisController& chassis,
                                     double distance_in,
                                     double timeout_ms) {
  return addMotion(
      [&chassis, distance_in, timeout_ms](const MotionOptions& options) {
        return chassis.makeDriveToCommand(distance_in,
                                          timeout_ms,
                                          options.exit,
                                          options.max_speed,
                                          options.min_speed);
      });
}

Routine::MotionStep Routine::driveTo(double distance_in,
                                     double timeout_ms) {
  return driveTo(requireChassis(), distance_in, timeout_ms);
}

Routine::MotionStep Routine::driveTo(ChassisController& chassis,
                                     Point point,
                                     double timeout_ms) {
  return addMotion(
      [&chassis, point, timeout_ms](const MotionOptions& options) {
        return chassis.makeMoveToPointCommand(point.x,
                                              point.y,
                                              options.direction,
                                              timeout_ms,
                                              options.exit,
                                              options.max_speed,
                                              options.overturn,
                                              options.min_speed);
      });
}

Routine::MotionStep Routine::driveTo(Point point, double timeout_ms) {
  return driveTo(requireChassis(), point, timeout_ms);
}

Routine::MotionStep Routine::driveTo(ChassisController& chassis,
                                     double x,
                                     double y,
                                     double timeout_ms) {
  return driveTo(chassis, Point{x, y}, timeout_ms);
}

Routine::MotionStep Routine::driveTo(double x,
                                     double y,
                                     double timeout_ms) {
  return driveTo(requireChassis(), Point{x, y}, timeout_ms);
}

Routine::MotionStep Routine::curveCircle(ChassisController& chassis,
                                         double result_angle_deg,
                                         double center_radius,
                                         double timeout_ms) {
  return addMotion(
      [&chassis, result_angle_deg, center_radius, timeout_ms](
          const MotionOptions& options) {
        return chassis.makeCurveCircleCommand(result_angle_deg,
                                              center_radius,
                                              timeout_ms,
                                              options.exit,
                                              options.max_speed,
                                              options.min_speed,
                                              options.reverse);
      });
}

Routine::MotionStep Routine::curveCircle(double result_angle_deg,
                                         double center_radius,
                                         double timeout_ms) {
  return curveCircle(requireChassis(),
                     result_angle_deg,
                     center_radius,
                     timeout_ms);
}

Routine::MotionStep Routine::curveCircleReverse(ChassisController& chassis,
                                                double result_angle_deg,
                                                double center_radius,
                                                double timeout_ms) {
  MotionOptions options{};
  options.reverse = true;
  return addMotion(
      [&chassis, result_angle_deg, center_radius, timeout_ms](
          const MotionOptions& step_options) {
        return chassis.makeCurveCircleReverseCommand(result_angle_deg,
                                                     center_radius,
                                                     timeout_ms,
                                                     step_options.exit,
                                                     step_options.max_speed,
                                                     step_options.min_speed);
      },
      options);
}

Routine::MotionStep Routine::curveCircleReverse(double result_angle_deg,
                                                double center_radius,
                                                double timeout_ms) {
  return curveCircleReverse(requireChassis(),
                            result_angle_deg,
                            center_radius,
                            timeout_ms);
}

Routine::MotionStep Routine::swing(ChassisController& chassis,
                                   double swing_angle,
                                   double drive_direction,
                                   double timeout_ms) {
  return addMotion(
      [&chassis, swing_angle, drive_direction, timeout_ms](
          const MotionOptions& options) {
        return chassis.makeSwingCommand(swing_angle,
                                        drive_direction,
                                        timeout_ms,
                                        options.exit,
                                        options.max_speed,
                                        options.min_speed);
      });
}

Routine::MotionStep Routine::swing(double swing_angle,
                                   double drive_direction,
                                   double timeout_ms) {
  return swing(requireChassis(), swing_angle, drive_direction, timeout_ms);
}

Routine::MotionStep Routine::wallReset(ChassisController& chassis,
                                       double reset_x,
                                       double reset_y,
                                       double reset_heading,
                                       double timeout_ms) {
  return addMotion(
      [&chassis, reset_x, reset_y, reset_heading, timeout_ms](
          const MotionOptions& options) {
        return chassis.makeWallResetCommand(reset_x,
                                            reset_y,
                                            reset_heading,
                                            options.drive_power,
                                            timeout_ms,
                                            options.current_threshold,
                                            options.velocity_threshold);
      });
}

Routine::MotionStep Routine::wallReset(double reset_x,
                                       double reset_y,
                                       double reset_heading,
                                       double timeout_ms) {
  return wallReset(requireChassis(),
                   reset_x,
                   reset_y,
                   reset_heading,
                   timeout_ms);
}

Routine::MotionStep Routine::turnToPoint(ChassisController& chassis,
                                         Point point,
                                         double timeout_ms) {
  return addMotion(
      [&chassis, point, timeout_ms](const MotionOptions& options) {
        return chassis.makeTurnToPointCommand(point.x,
                                              point.y,
                                              options.direction,
                                              timeout_ms,
                                              options.min_speed);
      });
}

Routine::MotionStep Routine::turnToPoint(Point point, double timeout_ms) {
  return turnToPoint(requireChassis(), point, timeout_ms);
}

Routine::MotionStep Routine::turnToPoint(ChassisController& chassis,
                                         double x,
                                         double y,
                                         double timeout_ms) {
  return turnToPoint(chassis, Point{x, y}, timeout_ms);
}

Routine::MotionStep Routine::turnToPoint(double x,
                                         double y,
                                         double timeout_ms) {
  return turnToPoint(requireChassis(), Point{x, y}, timeout_ms);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         Point point,
                                         int dir,
                                         double timeout_ms) {
  MotionOptions options{};
  options.direction = dir >= 0 ? 1 : -1;
  return addMotion(
      [&chassis, point, timeout_ms](const MotionOptions& step_options) {
        return chassis.makeMoveToPointCommand(point.x,
                                              point.y,
                                              step_options.direction,
                                              timeout_ms,
                                              step_options.exit,
                                              step_options.max_speed,
                                              step_options.overturn,
                                              step_options.min_speed);
      },
      options);
}

Routine::MotionStep Routine::moveToPoint(Point point,
                                         int dir,
                                         double timeout_ms) {
  return moveToPoint(requireChassis(), point, dir, timeout_ms);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         double x,
                                         double y,
                                         int dir,
                                         double timeout_ms) {
  return moveToPoint(chassis, Point{x, y}, dir, timeout_ms);
}

Routine::MotionStep Routine::moveToPoint(double x,
                                         double y,
                                         int dir,
                                         double timeout_ms) {
  return moveToPoint(requireChassis(), Point{x, y}, dir, timeout_ms);
}

Routine::MotionStep Routine::boomerang(ChassisController& chassis,
                                       Point point,
                                       int dir,
                                       double final_heading_deg,
                                       double lead,
                                       double timeout_ms) {
  MotionOptions options{};
  options.direction = dir >= 0 ? 1 : -1;
  return addMotion(
      [&chassis, point, final_heading_deg, lead, timeout_ms](
          const MotionOptions& step_options) {
        return chassis.makeBoomerangCommand(point.x,
                                            point.y,
                                            step_options.direction,
                                            final_heading_deg,
                                            lead,
                                            timeout_ms,
                                            step_options.exit,
                                            step_options.max_speed,
                                            step_options.overturn,
                                            step_options.min_speed);
      },
      options);
}

Routine::MotionStep Routine::boomerang(Point point,
                                       int dir,
                                       double final_heading_deg,
                                       double lead,
                                       double timeout_ms) {
  return boomerang(requireChassis(),
                   point,
                   dir,
                   final_heading_deg,
                   lead,
                   timeout_ms);
}

Routine::MotionStep Routine::boomerang(ChassisController& chassis,
                                       double x,
                                       double y,
                                       int dir,
                                       double final_heading_deg,
                                       double lead,
                                       double timeout_ms) {
  return boomerang(chassis,
                   Point{x, y},
                   dir,
                   final_heading_deg,
                   lead,
                   timeout_ms);
}

Routine::MotionStep Routine::boomerang(double x,
                                       double y,
                                       int dir,
                                       double final_heading_deg,
                                       double lead,
                                       double timeout_ms) {
  return boomerang(requireChassis(),
                   Point{x, y},
                   dir,
                   final_heading_deg,
                   lead,
                   timeout_ms);
}

void Routine::clear() {
  m_steps.clear();
  m_index = 0;
  m_current_initialized = false;
}

bool Routine::empty() const {
  return m_steps.empty();
}

std::size_t Routine::size() const {
  return m_steps.size();
}

void Routine::runBlocking(std::uint32_t period_ms) {
  schedule();
  while (scheduled()) {
    CommandScheduler::run();
    pros::delay(period_ms);
  }
}

void Routine::initialize() {
  m_index = 0;
  m_current_initialized = false;
  initializeCurrent();
}

void Routine::execute() {
  if (isFinished()) {
    return;
  }

  initializeCurrent();
  Command* command = m_steps[m_index].command.get();
  command->execute();

  if (command->isFinished()) {
    command->end(false);
    ++m_index;
    m_current_initialized = false;
    initializeCurrent();
  }
}

bool Routine::isFinished() {
  return m_index >= m_steps.size();
}

void Routine::end(bool interrupted) {
  if (interrupted && m_index < m_steps.size() && m_current_initialized) {
    m_steps[m_index].command->end(true);
  }
  m_current_initialized = false;
}

std::vector<Subsystem*> Routine::getRequirements() {
  std::vector<Subsystem*> requirements;

  for (auto& step : m_steps) {
    if (step.reserve_requirements && step.command != nullptr) {
      mergeRequirements(requirements, step.command->getRequirements());
    }
  }

  return requirements;
}

ChassisController& Routine::requireChassis() const {
  assert(m_chassis != nullptr);
  return *m_chassis;
}

Routine::MotionStep Routine::addMotion(MotionFactory factory) {
  return addMotion(std::move(factory), MotionOptions{});
}

Routine::MotionStep Routine::addMotion(MotionFactory factory,
                                       MotionOptions options) {
  Step step{};
  step.reserve_requirements = true;
  step.factory = std::move(factory);
  step.options = options;
  step.command = step.factory(step.options);

  m_steps.push_back(std::move(step));
  return MotionStep(*this, m_steps.size() - 1);
}

void Routine::rebuildMotion(std::size_t index) {
  if (index >= m_steps.size() || !m_steps[index].factory) {
    return;
  }
  m_steps[index].command = m_steps[index].factory(m_steps[index].options);
}

void Routine::initializeCurrent() {
  if (m_current_initialized || m_index >= m_steps.size()) {
    return;
  }

  m_steps[m_index].command->initialize();
  m_current_initialized = true;
}

void Routine::mergeRequirements(std::vector<Subsystem*>& requirements,
                                const std::vector<Subsystem*>& next) {
  for (auto* subsystem : next) {
    if (std::find(requirements.begin(), requirements.end(), subsystem) ==
        requirements.end()) {
      requirements.push_back(subsystem);
    }
  }
}

}  // namespace auton
}  // namespace mclib
