// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: typed units in mclib/units, the field frame and pose helpers in
// mclib/math.hpp, the PID controller in mclib/pid.hpp used by itself, and
// the mutex in mclib/sync.hpp. A PID holds a lift at a height. A second PID,
// in a task of its own, holds an intake at a speed. A GPS pose is turned into
// "how far and which way is the goal" for the driver. Change every port and
// every number to match your robot.
#include "main.h"

#include <cmath>
#include <cstdio>
#include <optional>

using namespace mclib::units::literals;
using mclib::device::DigitalButton;

// ---------------------------------------------------------------------------
// Units. A QLength, QTime, QAngle, QVoltage or QCurrent is a number that
// carries its unit. A length plus a time does not compile, and a length
// divided by a time is a QVelocity. Each is a mclib::units::Quantity, and so
// is any product or quotient of them, named or not.
//
// Three ways to make one:
//   24_in                     a literal
//   24 * mclib::units::inch   a number times a unit, for a number the program
//                             only has when it runs
//   QLength(0.6096)           the bare constructor, which takes metres,
//                             seconds, radians, volts or amperes. QLength(24)
//                             compiles and means 24 metres.
//
// The literals: _in _ft _tile _mm _cm _m, _ms _s _min, _deg _rad _rot,
// _V _mV, _A _mA, _rpm _degps _radps, _inps _mps, _mps2.
//
// Getting the number back means naming the unit: length.in(), time.ms(),
// angle.deg(). The value is stored as a double, so (24_in).in() is
// 23.999999999999996. Compare with a tolerance, never with ==.
//
// The type names and the literals are in the global namespace. If another
// library in the project has its own QLength or _in, as okapilib does, define
// MCLIB_NO_GLOBAL_UNITS before including mclib and write
// mclib::units::QLength.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);

mclib::device::MotorGroup lift_motors({-3, 4}, mclib::device::Gearset::Blue);
// On the arm's pivot, so it reads the arm's own angle.
mclib::device::Rotation lift_sensor(8);
mclib::device::MotorGroup intake_motors({-20, -21},
                                        mclib::device::Gearset::Blue);
mclib::device::Rotation vertical_tracker(-6);
// Mounted 4 in behind the turning centre.
mclib::device::Gps gps(1, 0.0, -4.0);

// ---------------------------------------------------------------------------
// The robot's measurements, each written once, with its unit.
// ---------------------------------------------------------------------------

// A Wheel cannot be made from a bare number. Say which measurement you took:
// calipers across it (fromDiameter) or a tape around the tread
// (fromCircumference).
const mclib::units::DriveGeometry drive_geometry{
    mclib::units::Wheel::fromDiameter(2.75_in),
    // Between the middles of the left and right wheels.
    11.375_in,
    // Wheel turns for one motor turn. A 36-tooth gear driving a 48 is
    // 36.0 / 48.0.
    1.0};

// The wheel, how far it sits from the turning centre, and wheel turns for one
// turn of its sensor.
const mclib::units::TrackingWheel vertical_wheel{
    mclib::units::Wheel::fromDiameter(2.0_in), 0.0_in, 1.0};

// A blue motor with nothing on it.
const QAngularVelocity motor_free_speed = 600_rpm;

const QLength arm_length = 14_in;
// Arm angles are measured up from level. The arm rests on its bottom stop
// when the program starts, and the rotation sensor is zeroed there.
const QAngle arm_rest_angle = -35_deg;
const QAngle arm_top_angle = 60_deg;
// The height of the arm's tip above its pivot when it scores.
const QLength high_goal_height = 9_in;
const QLength scoring_range = 1.5_tile;

// percent is the number 0.01.
const QAngularVelocity intake_speed =
    motor_free_speed * (80 * mclib::units::percent);

// rimVelocity: the speed of the edge of a wheel turning at a given rate. With
// the motors flat out that is the robot's top speed.
QVelocity topSpeed() {
  const QAngularVelocity wheel_speed =
      motor_free_speed * drive_geometry.gear_ratio;
  return mclib::units::rimVelocity(drive_geometry.wheel.radius(), wheel_speed);
}

void reportIfRefused(bool accepted) {
  if (!accepted) {
    std::printf("the controller did not take a status line\n");
  }
}

// ---------------------------------------------------------------------------
// Sums with units: facts about the robot, printed at startup. Check them
// against the robot. They are also the numbers a path's speed limits want.
// ---------------------------------------------------------------------------
void printRobotFacts() {
  const mclib::units::Wheel wheel = drive_geometry.wheel;
  std::printf("wheel: %.3f in across, %.2f in around, %.1f mm radius\n",
              wheel.diameter().in(), wheel.circumference().in(),
              wheel.radius().mm());

  const QVelocity top_speed = topSpeed();
  std::printf("top speed: %.1f in/s, which is %.2f m/s\n", top_speed.inps(),
              top_speed.mps());

  // The result of a sum has the right type by itself. A length over a
  // velocity is a time, and a velocity over a time is an acceleration.
  const QTime tile_time = 1_tile / top_speed;
  const QAcceleration launch = top_speed / 0.5_s;
  // convert() gives the value in any unit, for a type with no accessor of
  // its own.
  std::printf("one tile takes %.0f ms; reaching top speed in 0.5 s is "
              "%.0f in/s2\n",
              tile_time.ms(), launch.convert(1_inps / 1_s));

  const QAngle tile_turn = drive_geometry.distanceToEncoder(1_tile);
  std::printf("one tile is %.0f motor degrees, %.2f turns; one turn is "
              "%.2f in\n",
              tile_turn.deg(), tile_turn.convert(mclib::units::rotation),
              drive_geometry.encoderToDistance(1_rot).in());

  // Turning on the spot, each side rolls round a circle of radius
  // turnRadius(), half the track width.
  std::printf("a 90 degree spin rolls each side %.2f in\n",
              drive_geometry.spinArc(90_deg).in());

  // turnRate: how fast the robot turns when it moves at a speed along a
  // curve. A curvature is 1 over a radius.
  const QCurvature spin_curvature = 1.0 / drive_geometry.turnRadius();
  const QAngularVelocity top_spin =
      mclib::units::turnRate(top_speed, spin_curvature);
  // turnCurvature is the other way round: the curve the robot is on, from
  // its speed and how fast it is turning.
  const QCurvature bend = mclib::units::turnCurvature(40_inps, 90_degps);
  std::printf("fastest spin: %.0f deg/s; 40 in/s while turning 90 deg/s is "
              "an arc of radius %.1f in\n",
              top_spin.degps(), (1.0 / bend).in());

  // arcLength: how far a point at a radius moves for an angle.
  const QLength tip_travel =
      mclib::units::arcLength(arm_length, arm_top_angle - arm_rest_angle);
  // inches(x) is x.in() written as a function. degrees() and milliseconds()
  // are the same idea.
  std::printf("the arm's tip moves %.1f in from rest to top\n",
              mclib::units::inches(tip_travel));
}

// ---------------------------------------------------------------------------
// The lift's geometry. sin, cos and tan take a QAngle. asin, acos and atan
// return one.
// ---------------------------------------------------------------------------
QAngle armAngleForHeight(QLength tip_above_pivot) {
  // A length over a length is a plain number, and becomes a double by
  // itself. mclib::clamp keeps it inside what asin accepts.
  const double height_share =
      mclib::clamp(tip_above_pivot / arm_length, -1.0, 1.0);
  // The typed clamp, min, max and abs keep the unit.
  return mclib::units::clamp(mclib::units::asin(height_share), arm_rest_angle,
                             arm_top_angle);
}

// ---------------------------------------------------------------------------
// PID for a position: the lift.
//
// PIDController<Input, Output> checks units. This one reads an angle and
// answers with a voltage, so its gains are volts per degree, and a bare
// number for a gain does not compile.
// ---------------------------------------------------------------------------
using LiftPid = PIDController<QAngle, QVoltage>;
const LiftPid::Gain volts_per_degree = 1_V / 1_deg;

LiftPid lift_pid(0.30 * volts_per_degree, 0.01 * volts_per_degree,
                 1.5 * volts_per_degree);

// How close counts as there. arcAngle turns "the tip within a quarter of an
// inch" into an arm angle, about 1 degree.
const QAngle lift_tolerance = mclib::units::arcAngle(0.25_in, arm_length);

// The voltage that holds the arm level against its own weight. Raise it
// until a level arm stays where it is put.
const QVoltage lift_level_hold = 1.1_V;

void setUpLiftPid() {
  // A typed controller starts with both tolerances at zero and never reports
  // arrival until it is given some. Arrival is the small tolerance held for
  // the first time below, or the big one held for the second.
  lift_pid.setSmallBigErrorTolerance(lift_tolerance, 3.0 * lift_tolerance);
  // Plain milliseconds, not a QTime.
  lift_pid.setSmallBigErrorDuration(100, 300);
  // The lift also has to have stopped moving: the error may change by no
  // more than this from one update to the next.
  lift_pid.setDerivativeTolerance(0.2_deg);

  // The I term builds only within 10 degrees of the target, and adds at most
  // 2 V.
  lift_pid.setIntegralRange(10_deg);
  lift_pid.setIntegralMax(2_V);

  // As built, a PID outputs zero from the moment it arrives until reset().
  // That suits a drive move, which should stop dead. A lift would drop.
  lift_pid.setHoldOutput(true);

  // As built, arrival sticks once it has happened. With this off,
  // targetArrived() is about now, and goes false again if the lift is knocked
  // off its target.
  lift_pid.setLatchArrival(false);
}

void aimLiftAt(QAngle arm_angle) {
  // reset() empties the I term and clears arrival. setTarget() alone keeps
  // both from the last move.
  lift_pid.reset();
  lift_pid.setTarget(
      mclib::units::clamp(arm_angle, arm_rest_angle, arm_top_angle));
}

void setLiftCarrying(bool carrying) {
  // setCoefficient() swaps the gains and keeps everything else, including
  // what the I term built up under the old ones. clearSumError() empties the
  // I term and nothing else.
  if (carrying) {
    lift_pid.setCoefficient(0.42 * volts_per_degree, 0.015 * volts_per_degree,
                            1.8 * volts_per_degree);
  } else {
    lift_pid.setCoefficient(0.30 * volts_per_degree, 0.01 * volts_per_degree,
                            1.5 * volts_per_degree);
  }
  lift_pid.clearSumError();

  // getI() is the ki gain, not the size of the I term.
  std::printf("lift ki is now %.3f V per degree\n",
              lift_pid.getI().convert(volts_per_degree));
}

// One pass of the lift loop. Returns the arm angle it steered by, or nothing
// when the sensor is not answering.
//
// Call it every 10 ms for as long as the lift has to stay up. Unless
// setUseDt(true) is on, the I and D terms count updates, not time, so the
// gains only hold for the loop period they were tuned at.
std::optional<QAngle> updateLift() {
  const std::optional<QAngle> sensor_angle = lift_sensor.position();
  if (!sensor_angle) {
    lift_motors.stop();
    return std::nullopt;
  }
  const QAngle arm_angle = *sensor_angle + arm_rest_angle;

  // The PID empties its I term inside the small tolerance and each time the
  // lift crosses the target, so by itself it lets a loaded arm sag. This
  // second term carries the weight: all of it with the arm level, none with
  // the arm straight up.
  QVoltage command = lift_pid.update(arm_angle) +
                     lift_level_hold * mclib::units::cos(arm_angle);

  // Slow into both ends of the travel.
  if (arm_angle > arm_top_angle - 10_deg) {
    command = mclib::units::min(command, 4_V);
  }
  if (arm_angle < arm_rest_angle + 10_deg) {
    command = mclib::units::max(command, -4_V);
  }

  lift_motors.setVoltage(command);
  return arm_angle;
}

// Runs the lift to an arm angle. False if it ran out of time or lost its
// sensor. After a true, keep calling updateLift() to hold the lift there.
bool moveLiftTo(QAngle arm_angle, QTime limit) {
  aimLiftAt(arm_angle);
  const std::uint32_t start_ms = mclib::time::millis();
  while (!lift_pid.targetArrived()) {
    if (mclib::time::hasElapsed(start_ms, limit)) {
      lift_motors.stop();
      // getOutput() is what the last update() returned.
      std::printf("lift did not reach %.0f deg in %.0f ms; the PID was "
                  "asking for %.1f V\n",
                  arm_angle.deg(), limit.ms(), lift_pid.getOutput().volts());
      return false;
    }
    if (!updateLift()) {
      std::printf("lift stopped: its rotation sensor is not answering\n");
      return false;
    }
    pros::delay(10);
  }
  return true;
}

// ---------------------------------------------------------------------------
// PID for a speed, in a task of its own: the intake.
//
// PID is PIDController<double, double>: plain numbers, in whatever units the
// caller has in mind. Here it reads rpm and answers in volts.
//
// A PID is not safe to use from two tasks. Only the intake task touches
// intake_pid. The rest of the program talks to that task through
// intake_status, and only while holding the mutex.
// ---------------------------------------------------------------------------
struct IntakeStatus {
  // No target means the intake is off.
  std::optional<QAngularVelocity> target;
  // No speed means a motor is not answering.
  std::optional<QAngularVelocity> measured;
};

// sync::Mutex is pros::Mutex on the brain. A LockGuard holds it from the line
// it is made on to the end of that block. Hold it only to copy values in or
// out, never across a delay or a device read.
mclib::sync::Mutex intake_mutex;
IntakeStatus intake_status;

void setIntakeTarget(std::optional<QAngularVelocity> target) {
  const mclib::sync::LockGuard lock(intake_mutex);
  intake_status.target = target;
}

IntakeStatus readIntakeStatus() {
  const mclib::sync::LockGuard lock(intake_mutex);
  return intake_status;
}

const QTime intake_loop_period = 10_ms;
const QAngularVelocity intake_speed_tolerance = 25_rpm;

// The voltage that holds a speed before any correction: 12 V at the speed
// the motor turns with nothing on it.
const auto intake_volts_per_rpm = 12_V / motor_free_speed;

PID intake_pid(0.01, 0.06, 0.0);

void setUpIntakePid() {
  // A speed loop is never finished, so there is no arrival to detect.
  intake_pid.setArrive(false);
  // The plain PID starts with tolerances of 1 and 3, and it empties its I
  // term inside the small one. Zero keeps the I term working all the way to
  // the target speed.
  intake_pid.setSmallBigErrorTolerance(0, 0);
  // The I term adds at most 4 V.
  intake_pid.setIntegralMax(4);

  // Works the I and D terms out per second instead of per update, so the
  // gains mean the same at any loop period. Turning this on changes what ki
  // and kd mean: choose it first, then tune.
  intake_pid.setUseDt(true);
  // A step outside this range counts as the nearest end of it, so one pass
  // that was held up does not land as one huge step.
  intake_pid.setDtRange(5_ms, 50_ms);
}

void runIntakeSpeedLoop() {
  std::uint32_t wake_ms = pros::millis();
  std::uint32_t last_report_ms = mclib::time::millis();
  while (true) {
    const std::optional<QAngularVelocity> target = readIntakeStatus().target;
    const std::optional<QAngularVelocity> speed =
        intake_motors.averageVelocity();

    if (!target || !speed) {
      // reset() empties the I term. Left to build against motors that are
      // off or not answering, it would throw the intake to full power at the
      // next start.
      intake_pid.reset();
      intake_motors.stop();
    } else {
      // The plain PID takes and returns bare numbers: name the unit on the
      // way in and put it back on the way out.
      intake_pid.setTarget(target->rpm());
      // The second argument is the time since the last update. This loop
      // runs on an exact period, so it says so. update(speed) alone
      // measures the step from the clock.
      const QVoltage correction =
          intake_pid.update(speed->rpm(), intake_loop_period) * 1_V;

      // The I term empties each time the speed crosses the target, so it
      // cannot carry a steady load. The first term here does that, and the
      // PID corrects what is left.
      intake_motors.setVoltage(*target * intake_volts_per_rpm + correction);

      if (mclib::time::hasElapsed(last_report_ms, 1_s)) {
        last_report_ms = mclib::time::millis();
        // getLastDt() is the step the last update used, in seconds, after
        // setDtRange() was applied.
        std::printf("intake correction %.2f V over a %.0f ms step\n",
                    correction.volts(),
                    (intake_pid.getLastDt() * 1_s).ms());
      }
    }

    {
      const mclib::sync::LockGuard lock(intake_mutex);
      intake_status.measured = speed;
    }
    // PROS takes plain milliseconds.
    pros::Task::delay_until(
        &wake_ms, static_cast<std::uint32_t>(intake_loop_period.ms()));
  }
}

// ---------------------------------------------------------------------------
// The field frame. A Pose2D is x and y in inches and a heading in radians.
// Heading 0 faces +Y. Headings grow clockwise, so 90 degrees faces +X. On the
// robot, +Y is forward and +X is to its right. The GPS puts (0, 0) at the
// middle of the field. Pose2D and Vec2 hold plain numbers, not typed ones.
// ---------------------------------------------------------------------------
const mclib::Vec2 near_side_goal(48.0, 48.0);

// The far side of the field is the near side turned half-way round its
// middle. rotationMatrix() and rotate() turn a point counter-clockwise as the
// field is drawn, with +X to the right and +Y up. Headings count clockwise,
// so the heading goes down by the angle the point went round.
//
// Do not use these two to go between the field and the robot. That is
// fieldToRobot() and robotToField(), further down.
mclib::Pose2D turnedAboutCentre(const mclib::Pose2D& pose, double angle_rad) {
  const mclib::Mat2 turn = mclib::rotationMatrix(angle_rad);
  const mclib::Vec2 turned = turn * pose.translation();
  return mclib::Pose2D(turned.x(), turned.y(),
                       mclib::wrapAngle(pose.theta - angle_rad));
}

// Set these from your autonomous selector.
const bool starts_far_side = false;
const mclib::Pose2D near_side_start(-48.0, -60.0, 0.0);

mclib::Vec2 goal() {
  // rotate() is the same turn for one point.
  return starts_far_side ? mclib::rotate(near_side_goal, mclib::kPi)
                         : near_side_goal;
}

// The frame worked through with a robot at (24, -36) facing +X. The numbers
// in the comments are what this prints.
void printFrameCheck() {
  // degToRad() and radToDeg() go between the degrees people use and the
  // radians a Pose2D holds.
  const mclib::Pose2D robot(24.0, -36.0, degToRad(90.0));

  // headingVector: the way the robot faces, as a vector of length 1. (1, 0).
  const mclib::Vec2 facing = mclib::headingVector(robot.theta);

  // compose: a move in the robot's own frame, x to its right and y forward.
  // 12 in forward from here is (36, -36).
  const mclib::Pose2D ahead =
      mclib::compose(robot, mclib::Pose2D(0.0, 12.0, 0.0));

  // operator+ adds the three numbers and takes no notice of the heading:
  // (24, -24). It is for an offset in field coordinates, never for
  // "forward".
  const mclib::Pose2D shifted = robot + mclib::Pose2D(0.0, 12.0, 0.0);

  // robotPointToField: a point on the robot, on the field. The intake's
  // mouth, 8 in ahead of the turning centre, is at (32, -36).
  const mclib::Vec2 intake_mouth =
      mclib::robotPointToField(mclib::Vec2(0.0, 8.0), robot);

  // fieldPointToRobot: a point on the field, as (right, forward) from the
  // robot. The goal at (48, 48) is 84 in to the left and 24 in ahead:
  // (-84, 24).
  const mclib::Vec2 goal_seen =
      mclib::fieldPointToRobot(near_side_goal, robot);

  // robotToField and fieldToRobot turn a movement without shifting it. A
  // shove of 5 in to the robot's right moves it (0, -5) on the field.
  const mclib::Vec2 shove =
      mclib::robotToField(mclib::Vec2(5.0, 0.0), robot.theta);

  // headingToward: the heading that points from one place at another, 15.9
  // degrees here. wrapAngle brings any angle into -pi to pi, which makes a
  // difference of headings the short way round: -74.1 degrees, a turn to the
  // left.
  const double bearing =
      mclib::headingToward(robot.translation(), near_side_goal);
  const double turn = mclib::wrapAngle(bearing - robot.theta);

  // arcRadius: the radius of the one arc that leaves the robot along its
  // heading and passes through a point. -45.4 in: negative curves left. It
  // is infinity for a point dead ahead or dead behind. getRadius() in
  // utils.hpp is an older version that gets the frame wrong: use this one.
  const double radius_in = mclib::arcRadius(robot, near_side_goal);
  // An infinite radius makes a curvature of 0 here, which is a straight
  // line. At 40 in/s this arc is a turn of -50 deg/s.
  const QCurvature curvature = 1.0 / (radius_in * mclib::units::inch);
  const QAngularVelocity arc_turn_rate =
      mclib::units::turnRate(40_inps, curvature);

  std::printf("facing (%.2f, %.2f); 12 in ahead is (%.1f, %.1f); plus "
              "(0, 12) is (%.1f, %.1f)\n",
              facing.x(), facing.y(), ahead.x, ahead.y, shifted.x, shifted.y);
  std::printf("intake mouth at (%.1f, %.1f); goal seen at (%.1f, %.1f); a "
              "shove right is (%.1f, %.1f)\n",
              intake_mouth.x(), intake_mouth.y(), goal_seen.x(), goal_seen.y(),
              shove.x(), shove.y());
  std::printf("goal bearing %.1f deg, turn %.1f deg, %.1f in away; arc "
              "radius %.1f in, %.0f deg/s at 40 in/s\n",
              radToDeg(bearing), radToDeg(turn),
              robot.distanceTo(mclib::Pose2D(near_side_goal.x(),
                                             near_side_goal.y(), 0.0)),
              radius_in, arc_turn_rate.degps());
}

// A GPS fix to act on, or nothing. Empty when the sensor is not answering
// and when it is unsure of its position by more than 3 in.
std::optional<mclib::Pose2D> trustedFix() {
  const std::optional<mclib::Pose2D> fix = gps.pose();
  const std::optional<QLength> uncertainty = gps.error();
  if (!fix || !uncertainty || *uncertainty > 3_in) {
    return std::nullopt;
  }
  return fix;
}

// Each line of the controller screen is redrawn this often, so this is also
// the time between two looks at the GPS.
const QTime screen_line_period = 300_ms;

// False when the robot cannot have got from one fix to the next.
bool fixIsBelievable(const mclib::Pose2D& fix, const mclib::Pose2D& last_fix) {
  // vector() is (x, y, heading) as one Eigen vector.
  const mclib::Vec3 change = fix.vector() - last_fix.vector();

  // The third number is a heading, and from 179 degrees to -179 is 2
  // degrees, not 358. wrap() is wrapAngle() for a QAngle.
  const QAngle turned = mclib::units::wrap(change.z() * mclib::units::radian);

  // Over a short move a tank drive travels along the heading half-way
  // between the two fixes. fieldToRobot makes the movement on the field
  // (right, forward) for a robot at that heading.
  const double travel_heading = last_fix.theta + turned.rad() / 2.0;
  const mclib::Vec2 moved =
      mclib::fieldToRobot(change.head<2>(), travel_heading);

  // A tank drive does not slide sideways, and nothing outruns the top speed.
  // A velocity times a time is a length.
  const QLength reach = topSpeed() * screen_line_period;
  return std::abs(moved.x()) <= 3.0 && std::abs(moved.y()) <= reach.in();
}

void showGoal() {
  static std::optional<mclib::Pose2D> last_fix;

  const std::optional<mclib::Pose2D> fix = trustedFix();
  if (!fix) {
    last_fix.reset();
    reportIfRefused(master.setText(2, 0, "NO GPS FIX     "));
    return;
  }
  const bool believable = !last_fix || fixIsBelievable(*fix, *last_fix);
  last_fix = fix;
  if (!believable) {
    reportIfRefused(master.setText(2, 0, "GPS JUMPED     "));
    return;
  }

  // Give a plain number its unit as it leaves a Pose2D or a Vec2.
  const mclib::Vec2 goal_seen = mclib::fieldPointToRobot(goal(), *fix);
  const QLength right = goal_seen.x() * mclib::units::inch;
  const QLength forward = goal_seen.y() * mclib::units::inch;

  // hypot and atan2 on typed lengths. atan2(right, forward) is the angle
  // from dead ahead, positive to the right: the turn that faces the goal.
  const QLength range = mclib::units::hypot(right, forward);
  const QAngle turn = mclib::units::atan2(right, forward);
  const char side = mclib::units::sign(turn) < 0.0 ? 'L' : 'R';
  const double turn_deg = mclib::units::abs(turn).deg();

  // square() of a length is a QArea. Comparing squares saves the square
  // root.
  const bool in_range =
      mclib::units::square(right) + mclib::units::square(forward) <=
      mclib::units::square(scoring_range);

  if (in_range) {
    reportIfRefused(master.print(2, 0, "IN RANGE   %c%3.0f", side, turn_deg));
  } else {
    reportIfRefused(
        master.print(2, 0, "GOAL %3.0fin %c%3.0f", range.in(), side, turn_deg));
  }
}

// ---------------------------------------------------------------------------
// Checking a tracking wheel's size. Press Y, push the robot straight forward
// three tiles by hand, then press B.
// ---------------------------------------------------------------------------
const QLength check_push = 3_tile;

void checkTrackingWheel() {
  const std::optional<QAngle> turned = vertical_tracker.position();
  if (!turned) {
    std::printf("wheel check: the rotation sensor is not answering\n");
    return;
  }
  if (mclib::units::abs(*turned) < 90_deg) {
    std::printf("wheel check: press Y, push the robot %.0f in, press B\n",
                check_push.in());
    return;
  }

  // encoderToDistance: how far the wheel rolled for the angle its sensor
  // turned. distanceToEncoder is the other way round.
  const QLength measured = vertical_wheel.encoderToDistance(*turned);
  const QAngle expected = vertical_wheel.distanceToEncoder(check_push);
  std::printf("wheel check: pushed %.0f in, the wheel says %.2f in (%.0f of "
              "%.0f degrees)\n",
              check_push.in(), measured.in(), turned->deg(), expected.deg());

  if (mclib::units::abs(measured - check_push) <= 0.1_in) {
    std::printf("wheel check: the wheel's size is right\n");
    return;
  }

  // The distance one turn of the wheel covers is its circumference.
  const double wheel_turns =
      mclib::units::abs(*turned).convert(mclib::units::rotation) *
      vertical_wheel.gear_ratio;
  const mclib::units::Wheel corrected =
      mclib::units::Wheel::fromCircumference(check_push / wheel_turns);
  std::printf("wheel check: use Wheel::fromDiameter(%.3f_in)\n",
              corrected.diameter().in());
}

// ---------------------------------------------------------------------------
// The controller screen. It takes one write about every 50 ms, so the three
// lines take turns. Every message is 15 characters, the width of a line, so
// nothing of the message before it is left showing.
// ---------------------------------------------------------------------------
void showLift(std::optional<QAngle> arm_angle) {
  if (!arm_angle) {
    reportIfRefused(master.setText(0, 0, "LIFT NO SENSOR "));
    return;
  }
  reportIfRefused(master.print(0, 0, "LIFT %3.0f %s", arm_angle->deg(),
                               lift_pid.targetArrived() ? "THERE " : "MOVING"));
}

void showIntake() {
  const IntakeStatus status = readIntakeStatus();
  if (!status.measured) {
    reportIfRefused(master.setText(1, 0, "INTAKE NO MOTOR"));
  } else if (!status.target) {
    reportIfRefused(master.setText(1, 0, "INTAKE OFF     "));
  } else {
    const QAngularVelocity speed_error =
        mclib::units::abs(*status.target - *status.measured);
    const bool at_speed = speed_error <= intake_speed_tolerance;
    reportIfRefused(master.print(1, 0, "INTAKE %3.0f %s",
                                 status.measured->rpm(),
                                 at_speed ? "OK  " : "WAIT"));
  }
}

void showNextLine(std::optional<QAngle> arm_angle) {
  static int line = 0;
  if (line == 0) {
    showLift(arm_angle);
  } else if (line == 1) {
    showIntake();
  } else {
    showGoal();
  }
  line = (line + 1) % 3;
}

void initialize() {
  lift_motors.setBrakeMode(mclib::device::BrakeMode::Hold);
  lift_sensor.resetPosition();
  setUpLiftPid();
  setUpIntakePid();

  printRobotFacts();
  printFrameCheck();

  // One task from here until the program ends. PROS stops the tasks of
  // autonomous() and opcontrol(), not this one.
  static pros::Task intake_task(runIntakeSpeedLoop, "intake speed");
}

void disabled() {
  setIntakeTarget(std::nullopt);
  lift_motors.stop();
}

void competition_initialize() {}

void autonomous() {
  const mclib::Pose2D start =
      starts_far_side ? turnedAboutCentre(near_side_start, mclib::kPi)
                      : near_side_start;
  if (gps.setPose(start) == PROS_ERR) {
    std::printf("the GPS did not take the start pose\n");
  }

  setIntakeTarget(intake_speed);
  if (moveLiftTo(armAngleForHeight(high_goal_height), 1500_ms)) {
    const std::uint32_t arrived_ms = mclib::time::millis();
    while (!mclib::time::hasElapsed(arrived_ms, 1_s) && updateLift()) {
      pros::delay(10);
    }
  }
  setIntakeTarget(std::nullopt);
  moveLiftTo(arm_rest_angle, 1500_ms);
  lift_motors.stop();
}

void opcontrol() {
  aimLiftAt(arm_rest_angle);
  bool carrying = false;
  bool intake_on = false;

  std::uint32_t last_screen_ms = mclib::time::millis();
  while (true) {
    if (master.getNewPress(DigitalButton::L1)) {
      aimLiftAt(armAngleForHeight(high_goal_height));
    }
    if (master.getNewPress(DigitalButton::L2)) {
      aimLiftAt(arm_rest_angle);
    }
    if (master.getNewPress(DigitalButton::X)) {
      carrying = !carrying;
      setLiftCarrying(carrying);
    }
    if (master.getNewPress(DigitalButton::R1)) {
      intake_on = !intake_on;
      setIntakeTarget(intake_on ? std::optional(intake_speed) : std::nullopt);
    }
    if (master.getNewPress(DigitalButton::Y)) {
      vertical_tracker.resetPosition();
    }
    if (master.getNewPress(DigitalButton::B)) {
      checkTrackingWheel();
    }

    const std::optional<QAngle> arm_angle = updateLift();

    if (mclib::time::hasElapsed(last_screen_ms, screen_line_period / 3.0)) {
      last_screen_ms = mclib::time::millis();
      showNextLine(arm_angle);
    }
    pros::delay(10);
  }
}
