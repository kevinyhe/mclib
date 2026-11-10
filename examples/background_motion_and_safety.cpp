// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: running a motion in the background, and stopping safely. Acting
// partway through a motion, ending a motion early, the cancel requests, what
// disabled() must do, sensor faults and the autonomous time budget. Pick a
// routine on the brain screen or the controller, then start autonomous. Every
// routine starts from (0, 0) facing 0 deg. Change every port and every number
// to match your robot.
#include "main.h"

#include <cstdio>
#include <optional>

using namespace mclib::units::literals;
using mclib::auton::Point;
using mclib::control::AsyncMotion;
using mclib::control::CancelToken;
using mclib::control::MotionResult;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

auto imu = std::make_shared<mclib::device::Inertial>(15);

// The drive base. Say which measurement you took of the wheel: calipers
// across it (fromDiameter) or a tape around the tread (fromCircumference).
mclib::Chassis chassis(
    {-11, 13, 14},
    {-16, 17, -18},
    mclib::device::Gearset::Blue,
    mclib::ChassisDimensions{mclib::units::Wheel::fromDiameter(2.75_in),
                             11.375_in, 1.0},
    imu);

// One tracking wheel, forward-rolling, on a rotation sensor. Leave it out and
// the odometry measures forward travel off the drive encoders instead.
mclib::device::Rotation vertical_tracker(-6);
const mclib::units::TrackingWheel vertical_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), 0.0_in, 1.0};

// Building the controller binds `chassis` as the drivetrain every motion
// routine runs on and installs this tuning as the one every drive loop reads.
mclib::ChassisController drive(chassis, mclib::ChassisControllerConfig{
    .distance_pid = {0.4, 0.0, 3.0},
    .turn_pid = {0.3, 0.0, 1.5},
    .heading_pid = {0.3, 0.0, 1.5},
});

// Faces forward, to measure the gap to whatever is ahead.
mclib::device::Distance front_distance(9);

// The claw's piston, switched straight from its valve. On this example's
// robot the jaws close when the piston extends.
mclib::device::Pneumatic claw_piston('A');

void closeClaw() { claw_piston.extend(); }
void openClaw() { claw_piston.retract(); }

// The cascade lift, as a mechanism. It reads its height from its three
// motors, in degrees turned since initialize() zeroed them, so start the
// program with the lift fully down. The gains are placeholders. Tune them on
// your lift.
//
// A mechanism only acts while the command scheduler runs, and code that calls
// the blocking motions does not run it. So only the two Routine objects below
// move the lift: a Routine runs the scheduler.
mclib::device::MotorGroup lift_motors({-3, 4, 5},
                                      mclib::device::Gearset::Green);
mclib::mechanism::PositionMechanism lift(
    [] { return lift_motors.getAveragePositionDeg(); },
    [](double volts) { lift_motors.setVoltage(volts); },
    mclib::mechanism::PositionMechanismConfig{.kp = 0.15,
                                              .max_voltage = 12.0,
                                              .small_error = 5.0,
                                              .big_error = 15.0});

// Lift heights, in motor degrees up from fully down. The numbers are this
// example's: measure yours. "Over" carries a held stack clear of the Goal,
// and "on" sets it down on the Goal.
const double lift_down = 0.0;
const double lift_off_the_floor = 20.0;
const double lift_on_neutral_goal = 200.0;
const double lift_over_neutral_goal = 320.0;

// Globals, built once in initialize(). The scheduler keeps a pointer to a
// routine while it runs, and the field can end autonomous() in the middle of
// one. A routine local to autonomous() would be gone by then.
mclib::auton::Routine partway_routine(drive);
mclib::auton::Routine budgeted_routine(drive);

mclib::auton::AutonSelector selector;

// Line 2 of the controller is free: the selector writes line 0 and the
// odometry warning line 1.
bool reached(const char* step_name, MotionResult result) {
  if (result == MotionResult::Reached) {
    return true;
  }
  master.print(2, 0, "%s %s", step_name, mclib::control::toString(result));
  return false;
}

// ---------------------------------------------------------------------------
// A motion in the background. AsyncMotion::start() runs one blocking motion
// on its own task and returns at once.
//
// Keep the handle it returns: when the handle is destroyed, the motion is
// cancelled. And run one motion at a time: do not call another motion while
// one runs in the background.
// ---------------------------------------------------------------------------

void closeClawPartway() {
  openClaw();
  AsyncMotion approach = AsyncMotion::start([] { return driveTo(36_in, 3_s); });
  // False means the motion ended before the robot was 12 in from where it
  // started. That is a straight-line distance, not the length of a curve.
  if (approach.waitUntilTravelled(12_in)) {
    closeClaw();
  }
  reached("drive", approach.wait());
}

void openClawAfterATime() {
  closeClaw();
  AsyncMotion retreat =
      AsyncMotion::start([] { return moveToPoint(0_in, -24_in, -1, 3_s); });
  if (retreat.waitUntilElapsed(400_ms)) {
    openClaw();
  }
  reached("back", retreat.wait());
}

// Ends a motion on a sensor: the robot stops 8 in short of whatever is ahead.
void stopShortOfAnObstacle() {
  AsyncMotion approach =
      AsyncMotion::start([] { return driveTo(48_in, 4_s, true, 6_V); });

  bool sensor_lost = false;
  const bool stop_now = approach.waitUntil([&] {
    const std::optional<QLength> gap = front_distance.distance();
    // No reading means the sensor is unplugged. With nothing in range it
    // still reads, as 9999 mm. A robot that cannot see ahead stops.
    sensor_lost = !gap.has_value();
    return sensor_lost || *gap < 8_in;
  });
  if (!stop_now) {
    // The drive ended first: nothing was in the way.
    reached("drive", approach.wait());
    return;
  }
  // Returns once the drive has braked.
  approach.cancel();
  if (sensor_lost) {
    master.setText(2, 0, "NO DISTANCE");
  }
}

// The calling task is free while the motion runs. This one shows the distance
// the motion has left to go.
void watchAMotion() {
  AsyncMotion move =
      AsyncMotion::start([] { return moveToPoint(24_in, 48_in, 1, 3_s); });
  while (move.isRunning()) {
    master.print(2, 0, "%5.1f in left",
                 mclib::control::robotState().motionTelemetry().remaining_in);
    pros::delay(50);
  }
  // result() is empty only while the motion runs.
  reached("move", *move.result());
}

void changeTargetMidMotion() {
  AsyncMotion first =
      AsyncMotion::start([] { return moveToPoint(0_in, 48_in, 1, 3_s); });
  first.waitUntilTravelled(18_in);
  // Starting a motion cancels the one that is running and waits for it to
  // brake. `first` ends as Cancelled.
  AsyncMotion second =
      AsyncMotion::start([] { return moveToPoint(24_in, 24_in, 1, 3_s); });
  reached("second", second.wait());
}

// ---------------------------------------------------------------------------
// Ending a motion before its target
// ---------------------------------------------------------------------------

// `false` is `exit`: the next motion takes over with the drive still
// powered. The last argument makes that happen before the target. 12 V and
// -1 V are the defaults for the two arguments in between.
void handOverBeforeTheTarget() {
  // Hands over 10 deg before 90 deg, still turning.
  if (turnToAngle(90_deg, 1_s, false, 12_V, -1_V, 10_deg) !=
      MotionResult::Reached) {
    return;
  }
  // Hands over 6 in before the end, still driving.
  if (driveTo(36_in, 2_s, false, 12_V, -1_V, 6_in) != MotionResult::Reached) {
    return;
  }
  curveCircle(180_deg, 18_in, 3_s);
}

// The same things in a Routine: commands that start partway through a step,
// steps that hand over early, and a step that a sensor ends.
void buildPartwayRoutine() {
  partway_routine
      // Carries a stack to a Goal. The lift moves three times on the way.
      .moveToPoint(Point{0_in, 36_in}, 1, 3_s)
          // 200 ms after the step starts. 1500 is the lift's own time limit
          // in ms.
          .atTime(200_ms, lift.makeMoveToCommand(lift_off_the_floor, 1500.0))
          // After 12 in of travel, measured along the path driven.
          .atDistance(
              12_in, lift.makeMoveToCommand(lift_over_neutral_goal, 1500.0))
          // 6 in before the end.
          .atDistanceFromEnd(
              6_in, lift.makeMoveToCommand(lift_on_neutral_goal, 1500.0))
      .runOnce(openClaw)
      .wait(200_ms)
      .driveTo(-8_in, 2_s)
          .atTime(300_ms, lift.makeMoveToCommand(lift_down, 1500.0))
      .turnToAngle(90_deg, 1_s)
          .withEarlyExit(10_deg)
      .moveToPoint(Point{36_in, 28_in}, 1, 3_s)
          .withEarlyExit(6_in)
      .moveToPoint(Point{36_in, 0_in}, 1, 3_s)
          // Checked every tick. When it turns true the motion is cancelled,
          // the drive brakes and the next step starts.
          .until([] {
            const std::optional<QLength> gap = front_distance.distance();
            return !gap.has_value() || *gap < 8_in;
          })
      .runOnce([] {
        if (!front_distance.distance().has_value()) {
          master.setText(2, 0, "NO DISTANCE");
        }
      });
}

void runPartwayRoutine() {
  partway_routine.runBlocking();
  // A step that ends before one of its commands comes due never starts that
  // command.
  if (partway_routine.missedEvents() > 0) {
    master.print(2, 0, "%u NOT STARTED",
                 static_cast<unsigned>(partway_routine.missedEvents()));
  }
  // The lift's own check: none of its motors gave a position, and the lift
  // put 0 V on them.
  if (lift.hasSensorFault()) {
    master.setText(2, 0, "NO LIFT MOTORS");
  }
}

// ---------------------------------------------------------------------------
// The time budget. Autonomous is 15 s, and the field cuts the robot off
// whatever it is doing. A budget cuts the driving off first and keeps the end
// of the period for the steps that must not be skipped.
// ---------------------------------------------------------------------------

void buildBudgetedRoutine() {
  budgeted_routine
      // 14.5 s leaves half a second of margin. With no argument it is 15 s.
      // For a longer run, such as a skills run, pass its length.
      .withTimeBudget(14500_ms)
      // The last 1.5 s of the 14.5 s is kept for the mustRun() steps, so the
      // driving is cut off at 13 s.
      .withGrace(1500_ms)
      // FinishMustRun skips to the mustRun() steps when the driving time
      // runs out. Abandon stops there and runs nothing else.
      .withDeadlinePolicy(mclib::auton::DeadlinePolicy::FinishMustRun)
      // The robot starts with a stack in the claw and carries it high.
      .runOnce(closeClaw)
      .then(lift.makeMoveToCommand(lift_over_neutral_goal, 1500.0))
      .moveToPoint(Point{0_in, 48_in}, 1, 4_s)
      .moveToPoint(Point{48_in, 48_in}, 1, 4_s)
      .runOnce([] {
        std::printf("%.1f s of driving left\n",
                    budgeted_routine.remaining().s());
      })
      .moveToPoint(Point{48_in, 0_in}, 1, 4_s)
      // A step that asks for more time than is left gets what is left.
      .moveToPoint(Point{0_in, 0_in}, 1, 4_s)
      .turnToAngle(180_deg, 2_s)
      // These two run even when the driving was cut off: the lift comes down
      // and the claw lets go of the stack. 1200 is the lift's own time limit
      // in ms, inside the 1.5 s kept for it.
      .then(lift.makeMoveToCommand(lift_down, 1200.0)).mustRun()
      .runOnce(openClaw).mustRun();
}

void runBudgetedRoutine() {
  budgeted_routine.runBlocking();
  if (budgeted_routine.budgetExpired()) {
    master.print(2, 0, "%u STEPS CUT",
                 static_cast<unsigned>(budgeted_routine.skippedSteps()));
  }
}

// The same budget around blocking motions, without a Routine.
void budgetBlockingMotions() {
  mclib::auton::TimeBudget budget;
  budget.configure({
      .total = 14500_ms,
      .grace = 1_s,
      .policy = mclib::auton::DeadlinePolicy::FinishMustRun,
  });
  budget.start();

  // The time a motion asks for, or what is left of the 13.5 s of driving if
  // that is less. With nothing left it is 20 ms, and the motion times out.
  const auto time_for = [&budget](QTime wanted) {
    return mclib::auton::stepTimeoutFor(wanted, budget);
  };

  closeClaw();
  const bool drove_it_all =
      reached("first", moveToPoint(0_in, 48_in, 1, time_for(4_s))) &&
      reached("second", moveToPoint(48_in, 48_in, 1, time_for(4_s))) &&
      reached("third", moveToPoint(48_in, 0_in, 1, time_for(4_s))) &&
      reached("fourth", moveToPoint(0_in, 0_in, 1, time_for(4_s)));
  if (!drove_it_all && !budget.expired()) {
    // A motion failed with driving time to spare. Do not let go of the
    // stack in an unknown place.
    return;
  }

  // From here remaining() counts down to the end of the 14.5 s.
  budget.enterGrace();
  openClaw();
  // Backs off the stack once the jaws have opened. Each step gets the time it
  // asks for, or what is left of the 14.5 s if that is less.
  const QTime opening_time = mclib::auton::stepTimeoutFor(200_ms, budget);
  pros::delay(static_cast<std::uint32_t>(opening_time.ms()));
  reached("back", driveTo(-6_in, mclib::auton::stepTimeoutFor(1_s, budget)));
  std::printf("done at %.1f s\n", budget.elapsed().s());
}

// ---------------------------------------------------------------------------
// Cancel requests
// ---------------------------------------------------------------------------

// Holding B on the controller stops the routine. Use it while testing a
// routine, to save the robot from the perimeter wall.
void stopOnAButton() {
  // Any task may ask. The watcher leaves its loop once anyone has asked:
  // itself, the end of this routine, or disabled().
  pros::Task button_watch([] {
    while (!mclib::control::cancelRequested(CancelToken::Motion)) {
      if (master.getDigital(mclib::device::DigitalButton::B)) {
        mclib::control::requestCancel(CancelToken::Motion);
      }
      pros::delay(20);
    }
  });

  // The motion that is running stops at its next 10 ms tick, brakes and
  // returns Cancelled. The request stays set, so a motion started after it
  // would return Cancelled without moving.
  if (reached("out", driveTo(48_in, 4_s)) &&
      reached("turn", turnToAngle(180_deg, 2_s))) {
    reached("back", driveTo(48_in, 4_s));
  }

  mclib::control::requestCancel(CancelToken::Motion);
  button_watch.join();
  mclib::control::clearCancel(CancelToken::Motion);
}

// ---------------------------------------------------------------------------
// Sensor faults
// ---------------------------------------------------------------------------

// A motion that reads NaN from the IMU, a drive motor or the pose stops the
// drive and returns InvalidValue. It does not say which sensor. Asking first
// does.
bool sensorsAreReporting() {
  if (!imu->rotation().has_value()) {
    master.setText(2, 0, "NO IMU");
    return false;
  }
  if (!vertical_tracker.isInstalled()) {
    master.setText(2, 0, "NO TRACKER");
    return false;
  }
  if (!front_distance.distance().has_value()) {
    master.setText(2, 0, "NO DISTANCE");
    return false;
  }
  return true;
}

void checkSensorsThenDrive() {
  if (!sensorsAreReporting()) {
    return;
  }
  const MotionResult result = driveTo(36_in, 3_s);
  if (result == MotionResult::InvalidValue) {
    // A sensor stopped reporting during the drive. The pose may be wrong
    // from here on, so this routine ends.
    master.setText(2, 0, "SENSOR LOST");
    return;
  }
  if (!reached("drive", result)) {
    return;
  }
  reached("turn", turnToAngle(90_deg, 1_s));
}

void initialize() {
  // Calibrate before anything reads the IMU.
  imu->reset(true);

  // One odometry, on its own task, from here until the program ends. Nothing
  // else in mclib writes the pose.
  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  // False means the setup was rejected or odometry is already running: every
  // motion would then steer on a pose that never moves. Say so, on line 1: the
  // selector below writes line 0.
  if (!mclib::control::startOdometry(odometry)) {
    master.setText(1, 0, "ODOM NOT RUNNING");
  }
  // Set the IMU and the odometry pose together. The odometry tracks heading as
  // deltas from the pose it was reset to, while the motion routines steer on
  // the raw IMU rotation; starting the two in different frames leaves every
  // moveToPoint aiming off by a constant.
  chassis.setPose(0_in, 0_in, 0_deg);

  // A registered subsystem is updated on every scheduler pass, and
  // CommandScheduler::disable() calls its onDisabled().
  drive.setName("drive");
  drive.registerSelf();
  lift_motors.tarePosition();
  lift.setName("lift");
  lift.setDefaultCommand(lift.makeStopCommand());
  lift.registerSelf();

  buildPartwayRoutine();
  buildBudgetedRoutine();
  // A routine with a configuration error runs none of its steps. The
  // terminal says what the error is.
  if (partway_routine.hasConfigurationError() ||
      budgeted_routine.hasConfigurationError()) {
    master.setText(2, 0, "ROUTINE ERROR");
  }

  selector.add("act at 12 in", closeClawPartway);
  selector.add("act at 0.4 s", openClawAfterATime);
  selector.add("stop short", stopShortOfAnObstacle);
  selector.add("watch a move", watchAMotion);
  selector.add("new target", changeTargetMidMotion);
  selector.add("hand over", handOverBeforeTheTarget);
  selector.add("in a routine", runPartwayRoutine);
  selector.add("time budget", runBudgetedRoutine);
  selector.add("budget plain", budgetBlockingMotions);
  selector.add("stop button", stopOnAButton);
  selector.add("sensor check", checkSensorsThenDrive);
  // Three columns: 11 buttons are too small to press in the default two.
  selector.setLayout({.columns = 3});
  selector.bindController(master);
  selector.loadSelection();
  selector.startPolling();
}

// The field can disable the robot in the middle of anything. It ends the
// task that ran autonomous(), and nothing else: other tasks keep running.
void disabled() {
  // Stops a motion that runs on a task of its own: an AsyncMotion, or a
  // Routine step. It stops at its next 10 ms tick and brakes.
  mclib::control::requestCancel(CancelToken::Motion);
  // correctHeading() has its own request.
  mclib::control::requestCancel(CancelToken::HeadingCorrection);
  // Ends every command and calls onDisabled() on every registered subsystem:
  // here the drive and the lift.
  CommandScheduler::disable();
  chassis.stop();
  // Nothing above reaches a device that is driven straight from its port,
  // with no subsystem. A motor like that is yours to stop here. This
  // example's only one is the claw's piston, and it stays as it is.
}

void competition_initialize() {}

void autonomous() {
  // disabled() left both requests set. A blocking motion that starts while
  // its request is set returns Cancelled without moving.
  mclib::control::clearCancel(CancelToken::Motion);
  mclib::control::clearCancel(CancelToken::HeadingCorrection);
  selector.stopPolling();
  selector.runSelected();
}

void opcontrol() {}
