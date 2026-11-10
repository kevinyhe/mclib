// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: paths and the ways to follow them. Splines, arcs, speed limits
// and speed zones; followTrajectory() with RAMSETE; pure pursuit; motion
// profiles; feedforward, and the routines that measure it. Pick a routine on
// the brain screen or the controller, then start autonomous. Every routine
// starts from (0, 0) facing 0 deg. Give the robot 6 ft of clear floor on
// every side. Routines that work out numbers print them to the PROS terminal.
// Change every port and every number to match your robot.
#include "main.h"

#include "mclib/control/characterize.hpp"
#include "mclib/path/arc.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>

using namespace mclib::units::literals;
using mclib::control::inps2;
using mclib::control::inps3;
using mclib::control::MotionProfile;
using mclib::control::MotionResult;
using mclib::path::Path;
using mclib::path::Trajectory;
using mclib::path::TrajectoryConstraints;
using mclib::path::Waypoint;

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

mclib::auton::AutonSelector selector;

// ---------------------------------------------------------------------------
// Numbers measured on the robot. Every follower below turns a wheel speed
// into a voltage with them, so measure yours first: run "tuning log" from the
// selector, then `python3 tools/tune_drive.py tune000.csv` on a computer. The
// script prints these values.
// ---------------------------------------------------------------------------

// For one side of the drive: volts to overcome friction (kS), volts per in/s
// of wheel speed (kV), volts per in/s^2 of acceleration (kA).
const mclib::control::FeedforwardGains drive_feedforward{
    .kS = 0.7_V,
    .kV = 0.13_V / mclib::units::inps,
    .kA = 0.02_V / inps2,
};

// The track width that matches how fast the robot turns. The wheels scrub
// sideways in a turn, so this is wider than the 11.375 in a ruler gives.
const QLength effective_track_width = 13.5_in;

const mclib::control::RamseteConfig follower_config{
    // b is how hard the follower steers back onto the path. zeta is the
    // damping, 0 to 1. These are this robot's. "tuning log" records a
    // followed run, and the script says which way to move each.
    .gains = {.b = 50.0, .zeta = 0.7},
    // kV must be above 0. Without it followTrajectory() returns InvalidValue.
    .feedforward = drive_feedforward,
    // 0 uses the chassis track width, and the robot then lags on every curve.
    .track_width = effective_track_width,
    // The follower stops correcting when the planned speed reaches 0. With
    // these two on, a trajectory that stops then turns to its final heading
    // and closes what is left of the position miss.
    .turn_to_final_heading = true,
    .settle_position = true,
    // A sideways miss larger than this is closed by backing off and coming in
    // again, which takes 2 to 5 s. A smaller one is left.
    .settle_side_tolerance = 3_in,
};

// How fast a planned run may go. Start low. Raise them while the robot still
// stays on the path.
TrajectoryConstraints cruiseLimits() {
  TrajectoryConstraints limits;
  // printDriveLimits() works out how fast the drive can go.
  limits.max_velocity = 48_inps;
  limits.max_acceleration = 96 * inps2;
  // Braking. Left at 0, it equals max_acceleration.
  limits.max_deceleration = 72 * inps2;
  // Sideways acceleration in curves. Lower it if the robot slides wide.
  limits.max_lateral_acceleration = 60 * inps2;
  // Keeps the outer wheels under max_velocity in a curve.
  limits.track_width = effective_track_width;
  return limits;
}

// A drive loop you write yourself must stop for the same two things the
// library's motions stop for.
bool mayKeepDriving() {
  return !pros::competition::is_disabled() &&
         !mclib::control::cancelRequested(mclib::control::CancelToken::Motion);
}

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
// Paths
// ---------------------------------------------------------------------------

// A spline passes through every waypoint. The first one is where the robot
// is: a trajectory is not moved to the robot's position.
const std::vector<Waypoint> s_curve_waypoints = {
    Waypoint{0_in, 0_in},
    Waypoint{0_in, 18_in},
    Waypoint{24_in, 42_in},
    Waypoint{24_in, 60_in},
};

void printPathFacts() {
  const Path smooth = mclib::path::generateSpline(
      s_curve_waypoints,
      mclib::path::SplineConfig{
          // Distance between the points the path stores.
          .spacing = 0.5_in,
          // 0 to 1. Higher pulls the curve toward straight lines between
          // the waypoints.
          .tension = 0.2,
      });
  // Straight lines from corner to corner. Pure pursuit can follow this. A
  // trajectory cannot: the plan would reach each corner at full speed.
  const Path corners = Path::fromWaypoints({
      Waypoint{0_in, 0_in},
      Waypoint{0_in, 36_in},
      Waypoint{36_in, 36_in},
  });
  // Fewer than two different waypoints give a path nothing can follow.
  if (!smooth.valid() || !corners.valid()) {
    master.setText(2, 0, "BAD PATH");
    return;
  }

  const mclib::path::PathPoint one_foot_along = smooth.atDistance(12_in);
  const mclib::path::PathPoint halfway = smooth.atParameter(0.5);
  const QCurvature tightest = smooth.maxAbsCurvature(0_in, smooth.length());

  std::printf("spline: %.1f in long, ends at (%.1f, %.1f)\n",
              smooth.length().in(), smooth.back().x.in(), smooth.back().y.in());
  std::printf("12 in along: (%.1f, %.1f), heading %.0f deg\n",
              one_foot_along.x.in(), one_foot_along.y.in(),
              one_foot_along.heading.deg());
  std::printf("halfway: (%.1f, %.1f)\n", halfway.x.in(), halfway.y.in());
  // Curvature is 1 / turn radius. Positive curves to the right.
  std::printf("tightest turn: %.1f in radius\n", (1.0 / tightest).in());
  std::printf("corners: %.1f in long\n", corners.length().in());
}

// ---------------------------------------------------------------------------
// Trajectories, followed with RAMSETE
// ---------------------------------------------------------------------------

void followASpline() {
  const Path path = mclib::path::generateSpline(s_curve_waypoints);
  // The plan: where the robot should be at each moment, and how fast.
  const Trajectory trajectory = Trajectory::generate(path, cruiseLimits());
  // Empty means the path was not valid, or a limit was 0 or less.
  // followTrajectory() would return InvalidValue for it.
  if (trajectory.empty()) {
    master.setText(2, 0, "EMPTY PLAN");
    return;
  }
  // The time limit covers the planned run and the settling after it. A turn
  // and a short straight take about 1 s. The back-off takes 2 to 5 s.
  reached("spline", followTrajectory(trajectory, follower_config,
                                     trajectory.duration() + 3_s));
}

void slowDownForAStretch() {
  TrajectoryConstraints limits = cruiseLimits();
  // 15 in/s from 20 in to 36 in along the path, measured from its first
  // point. The plan brakes before the zone and speeds up after it.
  limits.speed_zones = {mclib::path::SpeedZone{20_in, 36_in, 15_inps}};

  const Trajectory trajectory = Trajectory::generate(
      mclib::path::generateSpline(s_curve_waypoints), limits);
  reached("zone", followTrajectory(trajectory, follower_config,
                                   trajectory.duration() + 3_s));
}

void followAPathBackward() {
  // The waypoints are still in the order the robot reaches them.
  const Path path = mclib::path::generateSpline({
      Waypoint{0_in, 0_in},
      Waypoint{0_in, -18_in},
      Waypoint{-24_in, -42_in},
  });
  TrajectoryConstraints limits = cruiseLimits();
  limits.reversed = true;

  const Trajectory trajectory = Trajectory::generate(path, limits);
  reached("back", followTrajectory(trajectory, follower_config,
                                   trajectory.duration() + 3_s));
}

// A straight line into an arc, without stopping between them: the first
// trajectory ends at 30 in/s and the second starts at 30 in/s.
void chainTwoTrajectories() {
  const QVelocity handoff_speed = 30_inps;
  // Where the line ends and the arc starts: x and y in inches, heading in
  // radians.
  const mclib::Pose2D handoff{0.0, 30.0, 0.0};

  TrajectoryConstraints line_limits = cruiseLimits();
  line_limits.end_velocity = handoff_speed;
  const Path straight = mclib::path::generateSpline(
      {Waypoint{0_in, 0_in}, Waypoint{0_in, 30_in}});
  const Trajectory line = Trajectory::generate(straight, line_limits);

  TrajectoryConstraints arc_limits = cruiseLimits();
  arc_limits.start_velocity = handoff_speed;
  const Trajectory arc = Trajectory::generate(
      mclib::path::planArc(handoff, 90_deg, 24_in).path, arc_limits);

  // `false` is `exit`: return with the drive still powered, and skip the
  // settling. The last argument returns 2 in before the end of the path.
  if (!reached("line", followTrajectory(line, follower_config,
                                        line.duration() + 1_s, false, 12_V,
                                        2_in))) {
    return;
  }
  reached("arc", followTrajectory(arc, follower_config, arc.duration() + 3_s));
}

// The arc curveCircle(90_deg, 24_in, ...) drives, as a path: from the
// robot's pose to heading 90 deg, around a centre 24 in to its right.
void followAnArc() {
  const mclib::path::ArcPlan arc =
      mclib::path::planArc(chassis.getPose(), 90_deg, 24_in);
  TrajectoryConstraints limits = cruiseLimits();
  // The radius sign and the turn direction decide whether the robot drives
  // the arc forward or backward. The plan says which.
  limits.reversed = arc.reversed;

  const Trajectory trajectory = Trajectory::generate(arc.path, limits);
  reached("arc", followTrajectory(trajectory, follower_config,
                                  trajectory.duration() + 3_s));
}

void followArcsWithCurveCircle() {
  // From here on, a curveCircle() that stops at its end plans its arc and
  // follows it with followTrajectory(). Call this once, in initialize(), to
  // have it for every routine.
  mclib::control::useRamseteForArcs(follower_config, cruiseLimits());
  curveCircle(90_deg, 24_in, 5_s);
  curveCircleReverse(0_deg, 24_in, 5_s);

  // Back to curveCircle()'s own controller.
  mclib::control::useEncoderArcs();
}

// The steps inside followTrajectory(), for a loop or a command of your own.
// This one ends when the plan does. It does not settle on the end point.
void followWithYourOwnLoop() {
  const Trajectory trajectory = Trajectory::generate(
      mclib::path::generateSpline(s_curve_waypoints), cruiseLimits());
  if (trajectory.empty()) {
    master.setText(2, 0, "EMPTY PLAN");
    return;
  }
  const mclib::control::Ramsete ramsete(follower_config.gains);
  const mclib::control::SimpleMotorFeedforward wheel_volts(drive_feedforward);

  QVelocity fastest_wheel_speed_asked = 0_inps;
  const QTime started = mclib::time::now();
  QTime elapsed = 0_s;
  while (elapsed < trajectory.duration() && mayKeepDriving()) {
    const mclib::path::TrajectoryState target = trajectory.sample(elapsed);
    const mclib::control::RamseteOutput command =
        ramsete.calculate(mclib::control::robotState().pose(), target);
    const mclib::control::DriveVoltages volts = mclib::control::ramseteVoltages(
        command, target.acceleration, effective_track_width, wheel_volts, 12_V);
    chassis.tankVoltage(volts.left, volts.right);

    const mclib::path::WheelSpeeds wheels =
        mclib::control::tankWheelSpeeds(command, effective_track_width);
    fastest_wheel_speed_asked =
        std::max({fastest_wheel_speed_asked, mclib::units::abs(wheels.left),
                  mclib::units::abs(wheels.right)});

    pros::delay(10);
    elapsed = mclib::time::now() - started;
  }
  chassis.stop();
  // Above what printDriveLimits() says the drive can hold, the plan is too
  // fast for the battery.
  std::printf("fastest wheel speed asked for: %.1f in/s\n",
              fastest_wheel_speed_asked.inps());
}

// ---------------------------------------------------------------------------
// Pure pursuit: steer at a point a fixed distance ahead on the path. It needs
// no plan made beforehand, and it follows a path with corners.
// ---------------------------------------------------------------------------

mclib::path::PurePursuitConfig pursuitConfig() {
  mclib::path::PurePursuitConfig config;
  // How far ahead on the path the robot aims. Shorter stays closer to the
  // path and weaves more.
  config.lookahead = 12_in;
  config.track_width = effective_track_width;
  config.max_velocity = 40_inps;
  // Never slower than this, so the robot does not stall short of the end.
  config.min_velocity = 6_inps;
  config.max_lateral_accel = 60 * inps2;
  config.max_decel = 40 * inps2;
  // The tightest turn it may ask for: a 6 in radius.
  config.max_curvature = 1.0 / 6_in;
  // Done this close to the last point.
  config.finish_tolerance = 2_in;
  // How far ahead of its last position it looks for the closest point. This
  // is what keeps it in order on a path that crosses itself.
  config.search_window = 24_in;
  return config;
}

void followCornersWithPurePursuit() {
  const Path route = Path::fromWaypoints({
      Waypoint{0_in, 0_in},
      Waypoint{0_in, 36_in},
      Waypoint{36_in, 36_in},
  });
  mclib::path::PurePursuit pursuit(route, pursuitConfig());
  const mclib::control::SimpleMotorFeedforward wheel_volts(drive_feedforward);

  mclib::path::PurePursuitOutput step;
  QLength farthest_off_path = 0_in;
  const QTime give_up_at = mclib::time::now() + 6_s;
  while (mayKeepDriving() && mclib::time::now() < give_up_at) {
    step = pursuit.update(mclib::control::robotState().pose());
    if (step.finished) {
      break;
    }
    chassis.tankVoltage(wheel_volts.calculate(step.wheels.left),
                        wheel_volts.calculate(step.wheels.right));
    farthest_off_path =
        std::max(farthest_off_path, mclib::units::abs(step.cross_track_error));
    pros::delay(10);
  }
  chassis.stop();

  if (!step.finished) {
    master.setText(2, 0, "PURSUIT CUT OFF");
    return;
  }
  // `finished` means no path is left ahead. It does not mean the robot is on
  // the last point: check how far from the path it ended.
  std::printf("ended %.1f in from the path, was %.1f in off at most\n",
              step.path_error.in(), farthest_off_path.in());
}

// Pure pursuit picks its own speed. This takes the speed from a motion
// profile instead and keeps the steering.
void purePursuitAtProfileSpeed() {
  const Path route = mclib::path::generateSpline(s_curve_waypoints);
  const MotionProfile speed_plan = MotionProfile::generate(
      route.length(), mclib::control::ProfileConstraints{
                          .max_velocity = 40_inps,
                          .max_acceleration = 80 * inps2,
                      });
  mclib::path::PurePursuit pursuit(route, pursuitConfig());
  const mclib::control::SimpleMotorFeedforward wheel_volts(drive_feedforward);

  mclib::path::PurePursuitOutput step;
  const QTime give_up_at = mclib::time::now() + 6_s;
  while (mayKeepDriving() && mclib::time::now() < give_up_at) {
    step = pursuit.update(mclib::control::robotState().pose());
    if (step.finished) {
      break;
    }
    // The profile is 0 in/s at both ends, and a robot told to go 0 in/s
    // never leaves. Hence the 6 in/s floor.
    const QVelocity profile_speed =
        std::max(speed_plan.velocityAtDistance(step.distance_along), 6_inps);
    const QVelocity speed = mclib::path::curvatureSpeedLimit(
        step.curvature, profile_speed, 60 * inps2);
    const mclib::path::WheelSpeeds wheels =
        mclib::path::wheelSpeeds(speed, step.curvature, effective_track_width);
    chassis.tankVoltage(wheel_volts.calculate(wheels.left),
                        wheel_volts.calculate(wheels.right));
    pros::delay(10);
  }
  chassis.stop();

  if (!step.finished) {
    master.setText(2, 0, "PURSUIT CUT OFF");
  }
}

// ---------------------------------------------------------------------------
// Motion profiles: speed against time for a straight move
// ---------------------------------------------------------------------------

const mclib::control::ProfileConstraints forward_profile_limits{
    .max_velocity = 48_inps,
    .max_acceleration = 96 * inps2,
    // Left at 0, it equals max_acceleration.
    .max_deceleration = 72 * inps2,
    // How fast the acceleration may change. 0 is no limit. Set it for a robot
    // that tips or drops what it carries when the drive kicks in.
    .max_jerk = 384 * inps3,
};

const mclib::control::ProfileConstraints reverse_profile_limits{
    .max_velocity = 36_inps,
    .max_acceleration = 72 * inps2,
};

void printProfileFacts() {
  const MotionProfile sharp =
      MotionProfile::trapezoidal(48_in, forward_profile_limits);
  const MotionProfile smooth =
      MotionProfile::sCurve(48_in, forward_profile_limits);
  std::printf("48 in: %.2f s without the jerk limit, %.2f s with it\n",
              sharp.duration().s(), smooth.duration().s());

  const mclib::control::ProfileState after_half_a_second =
      smooth.sample(500_ms);
  std::printf("after 0.5 s: %.1f in, %.1f in/s\n",
              after_half_a_second.position.in(),
              after_half_a_second.velocity.inps());
  std::printf("at 24 in: %.2f s in, %.1f in/s, top speed %.1f in/s\n",
              smooth.timeAtDistance(24_in).s(),
              smooth.velocityAtDistance(24_in).inps(),
              smooth.peakVelocity().inps());

  // Already moving at 48 in/s with 6 in to go: the profile brakes as hard as
  // the limits allow and still runs past.
  const MotionProfile too_fast_to_stop =
      MotionProfile::generate(6_in, forward_profile_limits, 48_inps);
  if (too_fast_to_stop.overshoots()) {
    std::printf("asked for 6 in, stops after %.1f in\n",
                too_fast_to_stop.netDisplacement().in());
  }
}

void driveAlongProfile(const MotionProfile& profile) {
  // A 0 distance, speed limit or acceleration limit gives an empty profile.
  if (profile.duration() <= 0_s) {
    master.setText(2, 0, "EMPTY PROFILE");
    return;
  }
  // Feedforward supplies the voltage the planned speed needs. kp, in volts
  // per inch behind the plan, corrects what is left.
  mclib::control::ProfileFollower follower({
      .gains = drive_feedforward,
      .kp = 1.0_V / mclib::units::inch,
  });

  const QLength start = chassis.averageDistance();
  QVoltage largest_correction = 0_V;
  follower.follow(profile);
  while (!follower.isFinished() && mayKeepDriving()) {
    const QVoltage command = follower.update(chassis.averageDistance() - start);
    chassis.tankVoltage(command, command);
    largest_correction = std::max(largest_correction,
                                  mclib::units::abs(follower.lastFeedback()));
    pros::delay(10);
  }
  chassis.stop();
  // A correction that is large beside the feedforward means the feedforward
  // gains are off. Measure them again before raising kp.
  std::printf("largest correction %.1f V, ended %.1f in from the plan\n",
              largest_correction.volts(), follower.lastError().in());
}

void driveProfilesThereAndBack() {
  const mclib::control::DirectionalConstraints limits{
      .forward = forward_profile_limits,
      .reverse = reverse_profile_limits,
  };
  driveAlongProfile(MotionProfile::generate(48_in, limits));
  // A negative distance drives backward, on the reverse limits.
  driveAlongProfile(MotionProfile::generate(-48_in, limits));
}

// ---------------------------------------------------------------------------
// Feedforward
// ---------------------------------------------------------------------------

// What the drive can do, worked out from the gains. Use it to choose
// max_velocity and max_acceleration.
void printDriveLimits() {
  const mclib::control::SimpleMotorFeedforward model(drive_feedforward);

  std::printf("48 in/s takes %.1f V, and %.1f V while gaining 96 in/s^2\n",
              model.calculate(48_inps).volts(),
              model.calculate(48_inps, 96 * inps2).volts());
  // 10 V, not 12: the follower needs the rest to correct with, and the
  // battery gives less under load.
  std::printf("10 V holds %.1f in/s\n",
              model.maxAchievableVelocity(10_V, 0 * inps2).inps());
  std::printf("at 48 in/s, 10 V can still add %.0f in/s^2\n",
              model.maxAchievableAcceleration(10_V, 48_inps).convert(inps2));
}

// Holds one voltage on both sides, lets the speed settle for 0.5 s, then
// measures it over 0.2 s.
mclib::control::VelocitySample measureSpeedAt(QVoltage volts) {
  chassis.tankVoltage(volts, volts);
  pros::delay(500);
  const QLength before = chassis.averageDistance();
  pros::delay(200);
  const QLength after = chassis.averageDistance();
  chassis.stop();
  pros::delay(300);
  return {volts, (after - before) / 200_ms};
}

// Ramps the voltage from 0 to 9 V over 1.5 s and fits kA to how the robot
// sped up.
mclib::control::AccelerationFit fitAccelerationGainOnARamp(
    const mclib::control::FeedforwardGains& known_gains) {
  constexpr std::size_t tick_count = 75;
  const QTime tick = 20_ms;
  std::array<QVoltage, tick_count> volts;
  std::array<QLength, tick_count> position;
  for (std::size_t i = 0; i < tick_count; ++i) {
    if (!mayKeepDriving()) {
      chassis.stop();
      return {};
    }
    volts[i] = 9_V * (static_cast<double>(i) / tick_count);
    chassis.tankVoltage(volts[i], volts[i]);
    position[i] = chassis.averageDistance();
    pros::delay(20);
  }
  chassis.stop();

  // Speed, then acceleration, each as the slope between the ticks on either
  // side.
  const QTime two_ticks = 2.0 * tick;
  const auto speedAt = [&](std::size_t i) {
    return (position[i + 1] - position[i - 1]) / two_ticks;
  };
  std::vector<mclib::control::AccelerationSample> ramp;
  for (std::size_t i = 2; i + 2 < tick_count; ++i) {
    const QAcceleration acceleration =
        (speedAt(i + 1) - speedAt(i - 1)) / two_ticks;
    ramp.push_back({volts[i], speedAt(i), acceleration});
  }
  return mclib::control::fitAccelerationGain(ramp.data(), ramp.size(),
                                             known_gains);
}

// The fit done on the robot, with no SD card and no computer. logForTuning()
// below is the better way: it also gives the effective track width.
void fitFeedforwardOnTheRobot() {
  std::vector<mclib::control::VelocitySample> steady_speeds;
  for (const QVoltage volts : {2_V, 3_V, 4_V, 5_V, 6_V, 7_V, 8_V}) {
    for (const double direction : {1.0, -1.0}) {
      if (!mayKeepDriving()) {
        return;
      }
      steady_speeds.push_back(measureSpeedAt(volts * direction));
    }
  }
  const mclib::control::VelocityFit steady = mclib::control::fitVelocityGains(
      steady_speeds.data(), steady_speeds.size());
  // Not valid means too few samples were faster than 1 in/s to fit a line.
  if (!steady.valid) {
    master.setText(2, 0, "NO kS kV FIT");
    return;
  }
  // r^2 under about 0.98 is bad data: the speed had not settled, the battery
  // was low, or the drive binds.
  std::printf("kS %.3f V, kV %.4f V per in/s, r^2 %.3f from %u samples\n",
              steady.kS.volts(), (steady.kV * 1_inps).volts(), steady.r_squared,
              static_cast<unsigned>(steady.used));

  const mclib::control::AccelerationFit ramp =
      fitAccelerationGainOnARamp({.kS = steady.kS, .kV = steady.kV});
  // kA matters least. Leave it at 0 if this fit fails.
  if (!ramp.valid) {
    master.setText(2, 0, "NO kA FIT");
    return;
  }
  std::printf("kA %.4f V per in/s^2\n",
              (ramp.kA * (1 * inps2)).volts());
}

// Logs what tools/tune_drive.py needs to /usd/tune000.csv, then tune001.csv
// and so on. The script prints kS, kV, kA and the effective track width, and
// for the followed run how far behind and how far to the side the robot was,
// with which RAMSETE gain to change.
void logForTuning() {
  // Static, because 512 rows do not fit on a task's stack.
  static mclib::telemetry::RowBuffer<512> rows;
  mclib::telemetry::SdCardSink sd_card("tune");
  // Without a card the logger writes nothing and says nothing.
  if (!sd_card.isAvailable()) {
    master.setText(2, 0, "NO SD CARD");
    return;
  }
  mclib::telemetry::Logger logger(sd_card, rows, {.period = 10_ms});
  mclib::control::TuningLog tuning_log(logger);
  mclib::telemetry::FlushTask card_writer(logger);

  // Forward and back at each voltage, 1.2 s each way. At 9 V this drive
  // covers about 6 ft in that time, so start with the whole field ahead. With
  // less room, use lower voltages.
  if (!reached("line", mclib::control::characterizeDrive(
                           tuning_log, {3_V, 6_V, 9_V}, 1200_ms))) {
    return;
  }
  // Spins both ways at each voltage, for the effective track width.
  if (!reached("spins", mclib::control::characterizeSpin(
                            tuning_log, {4_V, 6_V, 8_V}, 1500_ms))) {
    return;
  }

  // A followed run, logged with the point the follower was chasing. It only
  // means something once the feedforward and track width above are your own:
  // log the first two, paste the script's numbers in, then log again.
  const Trajectory trajectory = Trajectory::generate(
      mclib::path::generateSpline(s_curve_waypoints), cruiseLimits());
  reached("trace", mclib::control::traceMotion(tuning_log, [&] {
    return followTrajectory(trajectory, follower_config,
                            trajectory.duration() + 3_s);
  }));

  card_writer.stop();
  if (sd_card.writeFailed()) {
    master.setText(2, 0, "SD WRITE FAILED");
  }
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

  selector.add("tuning log", logForTuning);
  selector.add("fit on robot", fitFeedforwardOnTheRobot);
  selector.add("drive limits", printDriveLimits);
  selector.add("path facts", printPathFacts);
  selector.add("spline", followASpline);
  selector.add("speed zone", slowDownForAStretch);
  selector.add("backward", followAPathBackward);
  selector.add("chain two", chainTwoTrajectories);
  selector.add("planned arc", followAnArc);
  selector.add("curveCircle", followArcsWithCurveCircle);
  selector.add("own loop", followWithYourOwnLoop);
  selector.add("pure pursuit", followCornersWithPurePursuit);
  selector.add("pursuit+prof", purePursuitAtProfileSpeed);
  selector.add("profile facts", printProfileFacts);
  selector.add("profile", driveProfilesThereAndBack);
  // Three columns: 15 buttons are too small to press in the default two.
  selector.setLayout({.columns = 3});
  selector.bindController(master);
  selector.loadSelection();
  selector.startPolling();
}

void disabled() {
  mclib::control::requestCancel(mclib::control::CancelToken::Motion);
  mclib::control::requestCancel(mclib::control::CancelToken::HeadingCorrection);
  CommandScheduler::disable();
  chassis.stop();
}

void competition_initialize() {}

void autonomous() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);
  selector.stopPolling();
  selector.runSelected();
}

void opcontrol() {}
