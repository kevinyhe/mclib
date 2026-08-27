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

Routine::MotionStep& Routine::MotionStep::withMaxVoltage(QVoltage max_voltage) {
  m_routine->m_steps[m_index].options.max_voltage = max_voltage;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withMinVoltage(QVoltage min_voltage) {
  m_routine->m_steps[m_index].options.min_voltage = min_voltage;
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

Routine::MotionStep& Routine::MotionStep::withDrivePower(QVoltage drive_power) {
  m_routine->m_steps[m_index].options.drive_power = drive_power;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withCurrentThreshold(
    QCurrent current_threshold) {
  m_routine->m_steps[m_index].options.current_threshold = current_threshold;
  return rebuild();
}

Routine::MotionStep& Routine::MotionStep::withVelocityThreshold(
    QAngularVelocity velocity_threshold) {
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

Routine::MotionStep Routine::MotionStep::driveDistance(QLength distance,
                                                       QTime timeout) {
  return m_routine->driveDistance(distance, timeout);
}

Routine::MotionStep Routine::MotionStep::turnToHeading(QAngle heading,
                                                       QTime timeout) {
  return m_routine->turnToHeading(heading, timeout);
}

Routine::MotionStep Routine::MotionStep::turnToAngle(QAngle turn_angle,
                                                     QTime timeout) {
  return m_routine->turnToAngle(turn_angle, timeout);
}

Routine::MotionStep Routine::MotionStep::driveTo(QLength distance,
                                                 QTime timeout) {
  return m_routine->driveTo(distance, timeout);
}

Routine::MotionStep Routine::MotionStep::curveCircle(QAngle result_angle,
                                                     QLength center_radius,
                                                     QTime timeout) {
  return m_routine->curveCircle(result_angle, center_radius, timeout);
}

Routine::MotionStep Routine::MotionStep::curveCircleReverse(
    QAngle result_angle,
    QLength center_radius,
    QTime timeout) {
  return m_routine->curveCircleReverse(result_angle, center_radius, timeout);
}

Routine::MotionStep Routine::MotionStep::swing(QAngle swing_angle,
                                               double drive_direction,
                                               QTime timeout) {
  return m_routine->swing(swing_angle, drive_direction, timeout);
}

Routine::MotionStep Routine::MotionStep::wallReset(QLength reset_x,
                                                   QLength reset_y,
                                                   QAngle reset_heading,
                                                   QTime timeout) {
  return m_routine->wallReset(reset_x, reset_y, reset_heading, timeout);
}

Routine::MotionStep Routine::MotionStep::turnToPoint(Point point,
                                                     QTime timeout) {
  return m_routine->turnToPoint(point, timeout);
}

Routine::MotionStep Routine::MotionStep::turnToPoint(QLength x,
                                                     QLength y,
                                                     QTime timeout) {
  return m_routine->turnToPoint(x, y, timeout);
}

Routine::MotionStep Routine::MotionStep::moveToPoint(Point point,
                                                     QTime timeout) {
  return m_routine->moveToPoint(point, timeout);
}

Routine::MotionStep Routine::MotionStep::moveToPoint(Point point,
                                                     int dir,
                                                     QTime timeout) {
  return m_routine->moveToPoint(point, dir, timeout);
}

Routine::MotionStep Routine::MotionStep::moveToPoint(QLength x,
                                                     QLength y,
                                                     int dir,
                                                     QTime timeout) {
  return m_routine->moveToPoint(x, y, dir, timeout);
}

Routine::MotionStep Routine::MotionStep::boomerang(Point point,
                                                   int dir,
                                                   QAngle final_heading,
                                                   double lead,
                                                   QTime timeout) {
  return m_routine->boomerang(point, dir, final_heading, lead, timeout);
}

Routine::MotionStep Routine::MotionStep::boomerang(QLength x,
                                                   QLength y,
                                                   int dir,
                                                   QAngle final_heading,
                                                   double lead,
                                                   QTime timeout) {
  return m_routine->boomerang(x, y, dir, final_heading, lead, timeout);
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
                                           QLength distance,
                                           QTime timeout) {
  return addMotion(
      [&chassis, distance, timeout](const MotionOptions& options) {
        return chassis.makeDriveDistanceCommand(distance,
                                                timeout,
                                                options.stop_at_end,
                                                options.max_voltage);
      });
}

Routine::MotionStep Routine::driveDistance(QLength distance, QTime timeout) {
  return driveDistance(requireChassis(), distance, timeout);
}

Routine::MotionStep Routine::turnToHeading(ChassisController& chassis,
                                           QAngle heading,
                                           QTime timeout) {
  return addMotion(
      [&chassis, heading, timeout](const MotionOptions& options) {
        return chassis.makeTurnToHeadingCommand(heading,
                                                timeout,
                                                options.stop_at_end,
                                                options.max_voltage);
      });
}

Routine::MotionStep Routine::turnToHeading(QAngle heading, QTime timeout) {
  return turnToHeading(requireChassis(), heading, timeout);
}

Routine::MotionStep Routine::turnToAngle(ChassisController& chassis,
                                         QAngle turn_angle,
                                         QTime timeout) {
  return addMotion(
      [&chassis, turn_angle, timeout](const MotionOptions& options) {
        return chassis.makeTurnToAngleCommand(turn_angle,
                                              timeout,
                                              options.exit,
                                              options.max_voltage,
                                              options.min_voltage);
      });
}

Routine::MotionStep Routine::turnToAngle(QAngle turn_angle, QTime timeout) {
  return turnToAngle(requireChassis(), turn_angle, timeout);
}

Routine::MotionStep Routine::driveTo(ChassisController& chassis,
                                     QLength distance,
                                     QTime timeout) {
  return addMotion(
      [&chassis, distance, timeout](const MotionOptions& options) {
        return chassis.makeDriveToCommand(distance,
                                          timeout,
                                          options.exit,
                                          options.max_voltage,
                                          options.min_voltage);
      });
}

Routine::MotionStep Routine::driveTo(QLength distance, QTime timeout) {
  return driveTo(requireChassis(), distance, timeout);
}

Routine::MotionStep Routine::curveCircle(ChassisController& chassis,
                                         QAngle result_angle,
                                         QLength center_radius,
                                         QTime timeout) {
  return addMotion(
      [&chassis, result_angle, center_radius, timeout](
          const MotionOptions& options) {
        return chassis.makeCurveCircleCommand(result_angle,
                                              center_radius,
                                              timeout,
                                              options.exit,
                                              options.max_voltage,
                                              options.min_voltage,
                                              options.reverse);
      });
}

Routine::MotionStep Routine::curveCircle(QAngle result_angle,
                                         QLength center_radius,
                                         QTime timeout) {
  return curveCircle(requireChassis(), result_angle, center_radius, timeout);
}

Routine::MotionStep Routine::curveCircleReverse(ChassisController& chassis,
                                                QAngle result_angle,
                                                QLength center_radius,
                                                QTime timeout) {
  MotionOptions options{};
  options.reverse = true;
  return addMotion(
      [&chassis, result_angle, center_radius, timeout](
          const MotionOptions& step_options) {
        return chassis.makeCurveCircleReverseCommand(result_angle,
                                                     center_radius,
                                                     timeout,
                                                     step_options.exit,
                                                     step_options.max_voltage,
                                                     step_options.min_voltage);
      },
      options);
}

Routine::MotionStep Routine::curveCircleReverse(QAngle result_angle,
                                                QLength center_radius,
                                                QTime timeout) {
  return curveCircleReverse(requireChassis(),
                            result_angle,
                            center_radius,
                            timeout);
}

Routine::MotionStep Routine::swing(ChassisController& chassis,
                                   QAngle swing_angle,
                                   double drive_direction,
                                   QTime timeout) {
  return addMotion(
      [&chassis, swing_angle, drive_direction, timeout](
          const MotionOptions& options) {
        return chassis.makeSwingCommand(swing_angle,
                                        drive_direction,
                                        timeout,
                                        options.exit,
                                        options.max_voltage,
                                        options.min_voltage);
      });
}

Routine::MotionStep Routine::swing(QAngle swing_angle,
                                   double drive_direction,
                                   QTime timeout) {
  return swing(requireChassis(), swing_angle, drive_direction, timeout);
}

Routine::MotionStep Routine::wallReset(ChassisController& chassis,
                                       QLength reset_x,
                                       QLength reset_y,
                                       QAngle reset_heading,
                                       QTime timeout) {
  return addMotion(
      [&chassis, reset_x, reset_y, reset_heading, timeout](
          const MotionOptions& options) {
        return chassis.makeWallResetCommand(reset_x,
                                            reset_y,
                                            reset_heading,
                                            options.drive_power,
                                            timeout,
                                            options.current_threshold,
                                            options.velocity_threshold);
      });
}

Routine::MotionStep Routine::wallReset(QLength reset_x,
                                       QLength reset_y,
                                       QAngle reset_heading,
                                       QTime timeout) {
  return wallReset(requireChassis(), reset_x, reset_y, reset_heading, timeout);
}

Routine::MotionStep Routine::turnToPoint(ChassisController& chassis,
                                         Point point,
                                         QTime timeout) {
  return addMotion(
      [&chassis, point, timeout](const MotionOptions& options) {
        return chassis.makeTurnToPointCommand(point.x,
                                              point.y,
                                              options.direction,
                                              timeout,
                                              options.min_voltage);
      });
}

Routine::MotionStep Routine::turnToPoint(Point point, QTime timeout) {
  return turnToPoint(requireChassis(), point, timeout);
}

Routine::MotionStep Routine::turnToPoint(ChassisController& chassis,
                                         QLength x,
                                         QLength y,
                                         QTime timeout) {
  return turnToPoint(chassis, Point{x, y}, timeout);
}

Routine::MotionStep Routine::turnToPoint(QLength x, QLength y, QTime timeout) {
  return turnToPoint(requireChassis(), Point{x, y}, timeout);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         Point point,
                                         QTime timeout) {
  return addMotion(
      [&chassis, point, timeout](const MotionOptions& options) {
        return chassis.makeMoveToPointCommand(point.x,
                                              point.y,
                                              options.direction,
                                              timeout,
                                              options.exit,
                                              options.max_voltage,
                                              options.overturn,
                                              options.min_voltage);
      });
}

Routine::MotionStep Routine::moveToPoint(Point point, QTime timeout) {
  return moveToPoint(requireChassis(), point, timeout);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         Point point,
                                         int dir,
                                         QTime timeout) {
  MotionOptions options{};
  options.direction = dir >= 0 ? 1 : -1;
  return addMotion(
      [&chassis, point, timeout](const MotionOptions& step_options) {
        return chassis.makeMoveToPointCommand(point.x,
                                              point.y,
                                              step_options.direction,
                                              timeout,
                                              step_options.exit,
                                              step_options.max_voltage,
                                              step_options.overturn,
                                              step_options.min_voltage);
      },
      options);
}

Routine::MotionStep Routine::moveToPoint(Point point, int dir, QTime timeout) {
  return moveToPoint(requireChassis(), point, dir, timeout);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         QLength x,
                                         QLength y,
                                         int dir,
                                         QTime timeout) {
  return moveToPoint(chassis, Point{x, y}, dir, timeout);
}

Routine::MotionStep Routine::moveToPoint(QLength x,
                                         QLength y,
                                         int dir,
                                         QTime timeout) {
  return moveToPoint(requireChassis(), Point{x, y}, dir, timeout);
}

Routine::MotionStep Routine::boomerang(ChassisController& chassis,
                                       Point point,
                                       int dir,
                                       QAngle final_heading,
                                       double lead,
                                       QTime timeout) {
  MotionOptions options{};
  options.direction = dir >= 0 ? 1 : -1;
  return addMotion(
      [&chassis, point, final_heading, lead, timeout](
          const MotionOptions& step_options) {
        return chassis.makeBoomerangCommand(point.x,
                                            point.y,
                                            step_options.direction,
                                            final_heading,
                                            lead,
                                            timeout,
                                            step_options.exit,
                                            step_options.max_voltage,
                                            step_options.overturn,
                                            step_options.min_voltage);
      },
      options);
}

Routine::MotionStep Routine::boomerang(Point point,
                                       int dir,
                                       QAngle final_heading,
                                       double lead,
                                       QTime timeout) {
  return boomerang(requireChassis(), point, dir, final_heading, lead, timeout);
}

Routine::MotionStep Routine::boomerang(ChassisController& chassis,
                                       QLength x,
                                       QLength y,
                                       int dir,
                                       QAngle final_heading,
                                       double lead,
                                       QTime timeout) {
  return boomerang(chassis, Point{x, y}, dir, final_heading, lead, timeout);
}

Routine::MotionStep Routine::boomerang(QLength x,
                                       QLength y,
                                       int dir,
                                       QAngle final_heading,
                                       double lead,
                                       QTime timeout) {
  return boomerang(requireChassis(),
                   Point{x, y},
                   dir,
                   final_heading,
                   lead,
                   timeout);
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
