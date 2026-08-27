// mclib
#include "mclib/auton/autonomous_routine.hpp"

#include "mclib/auton/wait_until_command.hpp"
#include "mclib/command/commandScheduler.h"
#include "mclib/command/instantCommand.h"
#include "mclib/command/waitCommand.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>
#include <vector>

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

  /**
   * @brief Stop the inner command now, without waiting for the destructor.
   *
   * A triggered command is fire-and-forget: the step that started it finished
   * on the same tick, so when the routine's budget expires the only handle
   * left to it is this one.
   */
  void cancelInner() {
    if (m_command != nullptr && m_command->scheduled()) {
      m_command->cancel();
    }
  }

  ~TriggerCommand() override {
    cancelInner();
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

Routine::MotionStep& Routine::MotionStep::mustRun(bool must_run) {
  m_routine->m_steps[m_index].must_run = must_run;
  return *this;
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

Routine& Routine::MotionStep::waitUntil(std::function<bool()> condition,
                                       QTime timeout) {
  return m_routine->waitUntil(std::move(condition), timeout);
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
    Step step{};
    step.command = std::move(command);
    m_steps.push_back(std::move(step));
  }
  return *this;
}

Routine& Routine::then(std::unique_ptr<Command> command) {
  return add(std::move(command));
}

Routine& Routine::trigger(std::unique_ptr<Command> command) {
  if (command != nullptr) {
    Step step{};
    step.command = std::make_unique<TriggerCommand>(std::move(command));
    step.reserve_requirements = false;
    step.is_trigger = true;
    m_steps.push_back(std::move(step));
  }
  return *this;
}

Routine& Routine::runOnce(std::function<void()> action) {
  return add(std::make_unique<InstantCommand>(
      std::move(action),
      std::initializer_list<Subsystem*>{}));
}

Routine& Routine::wait(QTime duration) {
  add(std::make_unique<WaitCommand>(duration));
  if (!m_steps.empty()) {
    m_steps.back().timeout = duration;
  }
  return *this;
}

Routine& Routine::waitUntil(std::function<bool()> condition, QTime timeout) {
  add(std::make_unique<WaitUntilTimeoutCommand>(std::move(condition), timeout));
  if (!m_steps.empty()) {
    m_steps.back().timeout = timeout;
    m_steps.back().is_wait_until = true;
  }
  return *this;
}

Routine& Routine::withTimeBudget(QTime total) {
  TimeBudgetConfig config = m_budget.config();
  config.total = total;
  return withTimeBudget(config);
}

Routine& Routine::withTimeBudget(const TimeBudgetConfig& config) {
  // Deliberately does not touch the run in progress. initialize() stamps the
  // start and clears the counters; changing the policy from inside a routine
  // must not hand it a fresh allowance or un-apply a deadline that already
  // fired.
  m_budget.configure(config);
  return *this;
}

Routine& Routine::withDeadlinePolicy(DeadlinePolicy policy) {
  TimeBudgetConfig config = m_budget.config();
  config.policy = policy;
  return withTimeBudget(config);
}

Routine& Routine::withGrace(QTime grace) {
  TimeBudgetConfig config = m_budget.config();
  config.grace = grace;
  return withTimeBudget(config);
}

Routine& Routine::withoutTimeBudget() {
  return withTimeBudget(TimeBudgetConfig{});
}

Routine& Routine::mustRun(bool must_run) {
  if (!m_steps.empty()) {
    m_steps.back().must_run = must_run;
  }
  return *this;
}

bool Routine::hasTimeBudget() const {
  return m_budget.active();
}

const TimeBudget& Routine::timeBudget() const {
  return m_budget;
}

QTime Routine::elapsed() const {
  return m_budget.elapsed();
}

QTime Routine::remaining() const {
  return m_budget.remaining();
}

bool Routine::budgetExpired() const {
  return m_deadline_applied;
}

std::size_t Routine::skippedSteps() const {
  return m_skipped_steps;
}

std::size_t Routine::timedOutWaits() const {
  return m_timed_out_waits;
}

QTime Routine::stepTimeout(std::size_t index) const {
  if (index >= m_steps.size()) {
    return QTime{};
  }
  return m_steps[index].options.timeout;
}

bool Routine::hasConfigurationError() const {
  return m_configuration_error;
}

Routine::MotionStep Routine::driveDistance(ChassisController& chassis,
                                           QLength distance,
                                           QTime timeout) {
  return addMotion(
      [&chassis, distance](const MotionOptions& options) {
        return chassis.makeDriveDistanceCommand(distance,
                                                options.timeout,
                                                options.stop_at_end,
                                                options.max_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::driveDistance(QLength distance, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return driveDistance(*m_chassis, distance, timeout);
}

Routine::MotionStep Routine::turnToHeading(ChassisController& chassis,
                                           QAngle heading,
                                           QTime timeout) {
  return addMotion(
      [&chassis, heading](const MotionOptions& options) {
        return chassis.makeTurnToHeadingCommand(heading,
                                                options.timeout,
                                                options.stop_at_end,
                                                options.max_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::turnToHeading(QAngle heading, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return turnToHeading(*m_chassis, heading, timeout);
}

Routine::MotionStep Routine::turnToAngle(ChassisController& chassis,
                                         QAngle turn_angle,
                                         QTime timeout) {
  return addMotion(
      [&chassis, turn_angle](const MotionOptions& options) {
        return chassis.makeTurnToAngleCommand(turn_angle,
                                              options.timeout,
                                              options.exit,
                                              options.max_voltage,
                                              options.min_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::turnToAngle(QAngle turn_angle, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return turnToAngle(*m_chassis, turn_angle, timeout);
}

Routine::MotionStep Routine::driveTo(ChassisController& chassis,
                                     QLength distance,
                                     QTime timeout) {
  return addMotion(
      [&chassis, distance](const MotionOptions& options) {
        return chassis.makeDriveToCommand(distance,
                                          options.timeout,
                                          options.exit,
                                          options.max_voltage,
                                          options.min_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::driveTo(QLength distance, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return driveTo(*m_chassis, distance, timeout);
}

Routine::MotionStep Routine::curveCircle(ChassisController& chassis,
                                         QAngle result_angle,
                                         QLength center_radius,
                                         QTime timeout) {
  return addMotion(
      [&chassis, result_angle, center_radius](
          const MotionOptions& options) {
        return chassis.makeCurveCircleCommand(result_angle,
                                              center_radius,
                                              options.timeout,
                                              options.exit,
                                              options.max_voltage,
                                              options.min_voltage,
                                              options.reverse);
      },
      timeout);
}

Routine::MotionStep Routine::curveCircle(QAngle result_angle,
                                         QLength center_radius,
                                         QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return curveCircle(*m_chassis, result_angle, center_radius, timeout);
}

Routine::MotionStep Routine::curveCircleReverse(ChassisController& chassis,
                                                QAngle result_angle,
                                                QLength center_radius,
                                                QTime timeout) {
  MotionOptions options{};
  options.reverse = true;
  return addMotion(
      [&chassis, result_angle, center_radius](
          const MotionOptions& step_options) {
        return chassis.makeCurveCircleReverseCommand(result_angle,
                                                     center_radius,
                                                     step_options.timeout,
                                                     step_options.exit,
                                                     step_options.max_voltage,
                                                     step_options.min_voltage);
      },
      options,
      timeout);
}

Routine::MotionStep Routine::curveCircleReverse(QAngle result_angle,
                                                QLength center_radius,
                                                QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return curveCircleReverse(*m_chassis,
                            result_angle,
                            center_radius,
                            timeout);
}

Routine::MotionStep Routine::swing(ChassisController& chassis,
                                   QAngle swing_angle,
                                   double drive_direction,
                                   QTime timeout) {
  return addMotion(
      [&chassis, swing_angle, drive_direction](
          const MotionOptions& options) {
        return chassis.makeSwingCommand(swing_angle,
                                        drive_direction,
                                        options.timeout,
                                        options.exit,
                                        options.max_voltage,
                                        options.min_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::swing(QAngle swing_angle,
                                   double drive_direction,
                                   QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return swing(*m_chassis, swing_angle, drive_direction, timeout);
}

Routine::MotionStep Routine::wallReset(ChassisController& chassis,
                                       QLength reset_x,
                                       QLength reset_y,
                                       QAngle reset_heading,
                                       QTime timeout) {
  return addMotion(
      [&chassis, reset_x, reset_y, reset_heading](
          const MotionOptions& options) {
        return chassis.makeWallResetCommand(reset_x,
                                            reset_y,
                                            reset_heading,
                                            options.drive_power,
                                            options.timeout,
                                            options.current_threshold,
                                            options.velocity_threshold);
      },
      timeout);
}

Routine::MotionStep Routine::wallReset(QLength reset_x,
                                       QLength reset_y,
                                       QAngle reset_heading,
                                       QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return wallReset(*m_chassis, reset_x, reset_y, reset_heading, timeout);
}

Routine::MotionStep Routine::turnToPoint(ChassisController& chassis,
                                         Point point,
                                         QTime timeout) {
  return addMotion(
      [&chassis, point](const MotionOptions& options) {
        return chassis.makeTurnToPointCommand(point.x,
                                              point.y,
                                              options.direction,
                                              options.timeout,
                                              options.min_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::turnToPoint(Point point, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return turnToPoint(*m_chassis, point, timeout);
}

Routine::MotionStep Routine::turnToPoint(ChassisController& chassis,
                                         QLength x,
                                         QLength y,
                                         QTime timeout) {
  return turnToPoint(chassis, Point{x, y}, timeout);
}

Routine::MotionStep Routine::turnToPoint(QLength x, QLength y, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return turnToPoint(*m_chassis, Point{x, y}, timeout);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         Point point,
                                         QTime timeout) {
  return addMotion(
      [&chassis, point](const MotionOptions& options) {
        return chassis.makeMoveToPointCommand(point.x,
                                              point.y,
                                              options.direction,
                                              options.timeout,
                                              options.exit,
                                              options.max_voltage,
                                              options.overturn,
                                              options.min_voltage);
      },
      timeout);
}

Routine::MotionStep Routine::moveToPoint(Point point, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return moveToPoint(*m_chassis, point, timeout);
}

Routine::MotionStep Routine::moveToPoint(ChassisController& chassis,
                                         Point point,
                                         int dir,
                                         QTime timeout) {
  MotionOptions options{};
  options.direction = dir >= 0 ? 1 : -1;
  return addMotion(
      [&chassis, point](const MotionOptions& step_options) {
        return chassis.makeMoveToPointCommand(point.x,
                                              point.y,
                                              step_options.direction,
                                              step_options.timeout,
                                              step_options.exit,
                                              step_options.max_voltage,
                                              step_options.overturn,
                                              step_options.min_voltage);
      },
      options,
      timeout);
}

Routine::MotionStep Routine::moveToPoint(Point point, int dir, QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return moveToPoint(*m_chassis, point, dir, timeout);
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
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return moveToPoint(*m_chassis, Point{x, y}, dir, timeout);
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
      [&chassis, point, final_heading, lead](
          const MotionOptions& step_options) {
        return chassis.makeBoomerangCommand(point.x,
                                            point.y,
                                            step_options.direction,
                                            final_heading,
                                            lead,
                                            step_options.timeout,
                                            step_options.exit,
                                            step_options.max_voltage,
                                            step_options.overturn,
                                            step_options.min_voltage);
      },
      options,
      timeout);
}

Routine::MotionStep Routine::boomerang(Point point,
                                       int dir,
                                       QAngle final_heading,
                                       double lead,
                                       QTime timeout) {
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return boomerang(*m_chassis, point, dir, final_heading, lead, timeout);
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
  if (m_chassis == nullptr) {
    return addMissingChassisStep();
  }
  return boomerang(*m_chassis,
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
  m_budget.reset();
  m_deadline_applied = false;
  m_skipped_steps = 0;
  m_timed_out_waits = 0;
  m_configuration_error = false;
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
  m_deadline_applied = false;
  m_skipped_steps = 0;
  m_timed_out_waits = 0;
  m_budget.start();
  initializeCurrent();
}

void Routine::execute() {
  if (isFinished()) {
    return;
  }

  // The budget is checked before the step is run, not after, so a routine
  // that is out of time never gets one more tick of drive out of it.
  if (m_budget.active() && !m_deadline_applied && m_budget.expired()) {
    applyDeadlinePolicy();
    if (isFinished()) {
      return;
    }
  }

  // The must-run steps do not get to run forever either.
  if (m_deadline_applied && m_budget.graceExpired()) {
    countSkipped(m_steps.size());
    stopCurrentStep();
    brakeDrive();
    m_index = m_steps.size();
    return;
  }

  initializeCurrent();

  Command* command = m_steps[m_index].command.get();
  command->execute();

  if (command->isFinished()) {
    noteWaitTimeout(*command);
    command->end(false);
    m_current_initialized = false;
    advanceStep();
  }
}

bool Routine::isFinished() {
  return m_index >= m_steps.size();
}

void Routine::end(bool interrupted) {
  if (interrupted && m_index < m_steps.size() && m_current_initialized) {
    m_steps[m_index].command->end(true);
  }

  // Both paths, not just the interrupted one. A routine whose last motion was
  // written .withoutStop() told the chassis not to brake because another
  // motion was going to carry on from it; when the routine ends there is no
  // other motion, and the drive would otherwise hold its last PID voltage.
  // The triggered commands have nothing left to run alongside either.
  cancelTriggeredCommands();
  brakeDrive();

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

Routine::MotionStep Routine::addMotion(MotionFactory factory, QTime timeout) {
  return addMotion(std::move(factory), MotionOptions{}, timeout);
}

Routine::MotionStep Routine::addMotion(MotionFactory factory,
                                       MotionOptions options,
                                       QTime timeout) {
  Step step{};
  step.reserve_requirements = true;
  step.factory = std::move(factory);
  step.timeout = timeout;
  step.options = options;
  // The factories read options.timeout, so the requested timeout has to land
  // there and not only in step.timeout. Leaving it at the zero default built
  // every motion with no time limit at all: motion.cpp's loops are
  // `while (elapsed <= limit)`, so they ran one 10 ms tick and the robot never
  // moved, while ChassisController reads zero as "no timeout" and hung.
  // stepTimeoutFor() is the one place that answers this, here and at start.
  step.options.timeout = stepTimeoutFor(step.timeout, m_budget);
  step.command = step.factory(step.options);

  m_steps.push_back(std::move(step));
  return MotionStep(*this, m_steps.size() - 1);
}

Routine::MotionStep Routine::addMissingChassisStep() {
  // A motion with no chassis used to be assert(m_chassis != nullptr), which
  // compiles out in release: on the robot it was a null dereference in the
  // middle of an autonomous. Taking the program down is also not what a
  // routine wants - the mechanism steps around the motion, an intake or a
  // claw, work perfectly well without a drivetrain and are worth running. The
  // motion becomes a step that does nothing, and says so on the terminal and
  // through hasConfigurationError().
  if (!m_configuration_error) {
    std::fprintf(stderr,
                 "mclib: Routine has no chassis; motion steps will do "
                 "nothing. Call setChassis() before adding motions.\n");
  }
  m_configuration_error = true;

  Step step{};
  step.reserve_requirements = false;
  step.command = std::make_unique<InstantCommand>(
      []() {},
      std::initializer_list<Subsystem*>{});

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

  Step& step = m_steps[m_index];

  // Clamp the step to what is left of the budget. Only motion steps can be
  // clamped, because only they have a factory to rebuild; a wait already ends
  // on its own timeout, and the deadline catches whatever is left.
  if (step.factory) {
    const QTime effective = stepTimeoutFor(step.timeout, m_budget);

    if (effective != step.options.timeout) {
      step.options.timeout = effective;
      rebuildMotion(m_index);
    }
  }

  step.command->initialize();
  m_current_initialized = true;
}

void Routine::stopCurrentStep() {
  if (m_current_initialized && m_index < m_steps.size() &&
      m_steps[m_index].command != nullptr) {
    // end(true) is the cancellation path Phase 2 built: a blocking motion's
    // command sets the CancelToken from end(true), waits for the routine to
    // unwind at its next 10 ms boundary, then brakes the drive and clears the
    // slew state. Interrupting the step is what stops the robot; walking past
    // it would leave the motors driving.
    m_steps[m_index].command->end(true);
  }
  m_current_initialized = false;
}

void Routine::brakeDrive() {
  if (m_chassis == nullptr) {
    return;
  }

  // A step built with withoutStop() / withoutExit() is telling the chassis not
  // to brake when it ends, because the next motion was going to carry straight
  // on from it. When the budget cuts that step off there is no next motion, so
  // honouring it would leave the last PID voltage on the motors for the rest
  // of the match. ChassisController has no plain "stop" in its public API, so
  // this claims a zero-length goal that does stop at the end and cancels it,
  // which runs finishGoal() down the braking branch. A one-line
  // ChassisController::stop() would be the better home for this; it is outside
  // this unit's files.
  m_chassis->driveDistance(0.0 * units::inch,
                           0.0 * units::second,
                           /*stop_at_end=*/true);
  m_chassis->cancel();
}

void Routine::countSkipped(std::size_t next) {
  if (next <= m_index) {
    return;
  }

  // The step at m_index was running and is about to be interrupted. It ran, so
  // it is not one of the steps that never got a chance.
  const std::size_t ran = m_current_initialized ? 1 : 0;
  m_skipped_steps += next - m_index - ran;
}

void Routine::noteWaitTimeout(Command& command) {
  if (!m_steps[m_index].is_wait_until) {
    return;
  }

  auto& wait = static_cast<WaitUntilTimeoutCommand&>(command);
  if (!wait.timedOut()) {
    return;
  }

  // A waitUntil that gives up looks exactly like one whose condition came
  // true: the routine moves on either way. Saying so is the difference between
  // "the lift was up" and "we drove off with the lift down".
  ++m_timed_out_waits;
  std::fprintf(stderr,
               "mclib: Routine step %u waitUntil timed out after %.0f ms; "
               "the condition never became true\n",
               static_cast<unsigned>(m_index),
               wait.timeout().ms());
}

void Routine::cancelTriggeredCommands(bool keep_must_run) {
  for (std::size_t index = 0; index < m_steps.size() && index <= m_index;
       ++index) {
    const Step& step = m_steps[index];

    if (!step.is_trigger || step.command == nullptr) {
      continue;
    }
    // A trigger is what runs alongside the steps - the intake that the scoring
    // step needs still spinning. `.trigger(intake.spin()).mustRun()` says keep
    // it through the deadline; without that it stops with everything else.
    if (keep_must_run && step.must_run) {
      continue;
    }

    static_cast<TriggerCommand*>(step.command.get())->cancelInner();
  }
}

std::size_t Routine::nextRunnableStep(std::size_t from) const {
  std::vector<bool> must_run;
  must_run.reserve(m_steps.size());
  for (const auto& step : m_steps) {
    must_run.push_back(step.must_run);
  }

  return nextStepAfterDeadline(must_run, from, m_budget.config().policy);
}

void Routine::advanceStep() {
  ++m_index;
  m_current_initialized = false;

  if (m_deadline_applied) {
    const std::size_t next = nextRunnableStep(m_index);
    m_skipped_steps += next - m_index;
    m_index = next;
  } else if (m_budget.active() && m_budget.expired()) {
    // The step that just ended used up the last of the allowance. Apply the
    // policy here rather than starting the next step for one tick and killing
    // it: starting a motion command spawns a task and commands the drive.
    applyDeadlinePolicy();
    return;
  }

  initializeCurrent();
}

void Routine::applyDeadlinePolicy() {
  m_deadline_applied = true;
  m_budget.enterGrace();

  // A must-run step that is already running is left alone. Interrupting and
  // re-initialising it would run its action twice - an InstantCommand does its
  // work in initialize(), and a relative drive would re-stamp its start
  // position and travel the distance again.
  const bool keep_current = m_current_initialized && m_index < m_steps.size() &&
                            m_steps[m_index].must_run &&
                            m_budget.config().policy ==
                                DeadlinePolicy::FinishMustRun;

  cancelTriggeredCommands(/*keep_must_run=*/true);

  if (keep_current) {
    return;
  }

  const std::size_t next = nextRunnableStep(m_index);
  countSkipped(next);

  stopCurrentStep();
  brakeDrive();

  m_index = next;
  initializeCurrent();
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
