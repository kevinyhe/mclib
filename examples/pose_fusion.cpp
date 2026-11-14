// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: correcting odometry from other sensors. Odometry drifts: every
// wheel slip adds error that stays. This program shows the three corrections
// mclib has.
//
//   PoseFusion    distance sensors and a GPS sensor, blended in all the time
//                 while the robot drives
//   snapshot      distance sensors, one fix while the robot stands still
//   wallReset()   push into a wall and take the pose from it, with a distance
//                 sensor to check it is the wall
//
// The first two measure against the field map in
// mclib/snapshot/collision_map.hpp. This example uses one part of that map,
// the perimeter wall. On the map x and y run from 0 to 144 in from one corner
// of the field and heading 0 points along +Y. This program keeps its odometry
// in map coordinates, so every pose and target below is a place on the map.
// Change every port and every number to match your robot.
#include "main.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

using namespace mclib::units::literals;

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

mclib::device::Distance left_distance(3);
mclib::device::Distance right_distance(4);
mclib::device::Distance back_distance(5);

// The port, then where the sensor sits from the tracking centre: inches to the
// right, inches forward. This one is 4 in behind the centre.
//
// The GPS reports from the centre of the field, and the code strip on the
// field wall fixes which way its +Y points. The map's +Y has to point the
// same way, which makes the map's corner (0, 0) the GPS's (-72 in, -72 in).
mclib::device::Gps gps(8, 0.0, -4.0);

constexpr double map_centre_in = snapshot::FIELD_SIZE_IN / 2.0;

// The tracking centre is this far ahead of the back of the robot.
const QLength centre_to_back = 7_in;

// ---------------------------------------------------------------------------
// Where each distance sensor sits. The filter and the snapshot both read this
// list.
//
//   x_right_in, y_fwd_in   inches to the right of the tracking centre and
//                          inches ahead of it
//   rel_deg                the way the sensor points: 0 forward, 90 right,
//                          180 back, -90 left
//
// Three more fields are left at their defaults here. field_mask_override
// picks which objects on the map the beam can stop on: by default the
// perimeter wall alone. max_range_in, 78 in, is the longest reading to
// believe. use_confidence_gate with min_confidence (0 to 63) drops a reading
// the sensor itself rates lower than that.
//
// Sensors that point along one line, such as left and right alone, fix the
// position along that line only. The back sensor is what lets one snapshot
// fix both x and y.
// ---------------------------------------------------------------------------

const std::vector<snapshot::DistanceSensorConfig> distance_sensors{
    {.dev = &left_distance,
     .x_right_in = -6.0f,
     .y_fwd_in = 2.0f,
     .rel_deg = -90.0f},
    {.dev = &right_distance,
     .x_right_in = 6.0f,
     .y_fwd_in = 2.0f,
     .rel_deg = 90.0f},
    {.dev = &back_distance,
     .x_right_in = 0.0f,
     .y_fwd_in = -5.0f,
     .rel_deg = 180.0f},
};

// The filter skips a reading that is zero, negative or NaN.
constexpr double no_reading = -1.0;

// One reading for the filter, under the rules the snapshot applies to its own
// readings. A sensor with nothing in range reports 9999 mm, which is not a
// distance to anything.
double inchesFrom(const snapshot::DistanceSensorConfig& sensor) {
  const std::optional<QLength> reading = sensor.dev->distance();
  const bool port_is_answering = reading.has_value();
  if (!port_is_answering) {
    return no_reading;
  }
  if (sensor.use_confidence_gate &&
      sensor.dev->getConfidence() < sensor.min_confidence) {
    return no_reading;
  }
  if (reading->in() > sensor.max_range_in) {
    return no_reading;
  }
  return reading->in();
}

// ---------------------------------------------------------------------------
// The filter. It keeps its own pose and how unsure it is of it. Driving makes
// it less sure. A distance or GPS reading that fits makes it more sure and
// moves its pose. A reading too far from what the map predicts is rejected: a
// robot, a stack or a Goal was between the sensor and the wall.
// ---------------------------------------------------------------------------

// true: the odometry task runs the filter and moves the odometry position
// toward the filter's, so every motion steers on the corrected pose.
// false: odometry runs uncorrected and watchPoseFilter() only shows what the
// filter makes of the same drive. Start with false. Drive around, stop on a
// spot you have measured, and see which of the two poses on the screen is
// right.
constexpr bool filter_corrects_odometry = true;

// odom_along_var_per_in and odom_side_var_per_in say how fast odometry error
// grows, forward and sideways, as variance in square inches per inch driven.
// Set each from what measures that direction on your robot:
//
//                          along     side
//   drive encoders only    0.09      0.09
//   vertical wheel         0.0005    0.09
//   horizontal wheel       0.09      0.0005
//   both wheels            0.0005    0.0005
//
// 0.09 is an error of about 3 in after 100 in driven. 0.0005 is about 0.2 in.
// This robot has a vertical wheel.
//
// The field offsets are where the odometry origin is on the map. Zero, because
// this odometry is in map coordinates already. For an odometry that starts at
// (0, 0) in the centre of the field they are 72 and 72.
const mclib::control::PoseFusionConfig fusion_config{
    .filter = {.odom_along_var_per_in = 0.0005, .odom_side_var_per_in = 0.09},
    .field_offset_x_in = 0.0,
    .field_offset_y_in = 0.0,
};

std::vector<mclib::control::DistanceSensorInput> filterInputs() {
  std::vector<mclib::control::DistanceSensorInput> inputs;
  for (const snapshot::DistanceSensorConfig& sensor : distance_sensors) {
    inputs.push_back({.geometry = sensor.geometry(),
                      .read_in = [sensor] { return inchesFrom(sensor); }});
  }
  return inputs;
}

// The odometry task keeps a pointer to this, so it has to live as long as the
// program does.
mclib::control::PoseFusion fusion(fusion_config, filterInputs());

// The GPS is optional, and works with or without distance sensors. The filter
// trusts a GPS reading to position_sigma_in, or to the sensor's own error
// estimate when that is larger. PoseFusion uses the GPS position and never
// its heading.
//
// latency_ms is how old a GPS pose already is when the program reads it. The
// filter applies the reading where the robot was then, not where it is now:
// at 60 in/s, 50 ms is 3 in. 0 applies it where the robot is now. Measure it
// before setting it; mclib has no measured number for the V5 GPS. It can be
// at most the filter's history_ms, 250 ms by default, or setGps() refuses it.
void giveFilterTheGps() {
  const bool accepted = fusion.setGps({
      .read = [] { return gps.pose(); },
      .error_in = [] { return (gps.getErrorM() * 1_m).in(); },
      .position_sigma_in = 1.0,
      .period_ms = 20,
      .latency_ms = 0,
      .origin_x_in = map_centre_in,
      .origin_y_in = map_centre_in,
  });
  if (!accepted) {
    master.setText(2, 0, "GPS LATENCY TOO LONG");
  }
}

void showFilter(const mclib::Pose2D& filtered,
                int readings_used,
                int readings_rejected,
                double position_sigma_in) {
  const mclib::Pose2D odometry = chassis.getPose();
  pros::screen::print(pros::E_TEXT_MEDIUM, 0,
                      "odometry  x %6.1f  y %6.1f in  %6.1f deg", odometry.x,
                      odometry.y, radToDeg(odometry.theta));
  pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                      "filter    x %6.1f  y %6.1f in  %6.1f deg", filtered.x,
                      filtered.y, radToDeg(filtered.theta));
  // The filter's own figure for its position error, one standard deviation. A
  // rejected count that climbs with nothing in front of the sensors means a
  // mounting number or the start pose is wrong.
  pros::screen::print(pros::E_TEXT_MEDIUM, 2,
                      "filter +/- %4.1f in  used %6d  rejected %6d",
                      position_sigma_in, readings_used, readings_rejected);
}

// The same filter run by hand. It only watches: nothing is written back to
// odometry. Run this way it also estimates the heading, here from the GPS.
//
// It needs odometry poses in map coordinates, which this program's are. It
// loops no faster than the slowest sensor it reads, because a reading read
// twice is counted twice. A distance sensor has a new one about every 33 ms.
constexpr std::uint32_t watch_period_ms = 40;
constexpr double gps_position_sigma_in = 1.0;
constexpr double gps_heading_sigma_deg = 2.0;

void watchPoseFilter() {
  mclib::control::PoseFilter filter(fusion_config.filter);
  mclib::control::OdometryReading previous = mclib::control::readOdometry();
  // The start pose is known to 0.5 in and 0.5 deg.
  filter.reset(previous.pose, 0.5, 0.5);

  while (true) {
    const mclib::control::OdometryReading odometry =
        mclib::control::readOdometry();
    if (odometry.jumps != previous.jumps) {
      // setPose(), a wall reset or a snapshot set the pose. That is not
      // distance driven, so the filter starts again from the new pose.
      filter.reset(odometry.pose, 0.5, 0.5);
    } else {
      filter.predict(previous.pose, odometry.pose);
    }
    previous = odometry;

    for (const snapshot::DistanceSensorConfig& sensor : distance_sensors) {
      filter.updateDistance(sensor.geometry(), inchesFrom(sensor));
    }

    const std::optional<mclib::Pose2D> from_field_centre = gps.pose();
    const std::optional<QLength> gps_error = gps.error();
    if (from_field_centre && gps_error) {
      const mclib::Pose2D on_map{from_field_centre->x + map_centre_in,
                                 from_field_centre->y + map_centre_in,
                                 from_field_centre->theta};
      filter.updateGps(on_map,
                       std::max(gps_position_sigma_in, gps_error->in()),
                       gps_heading_sigma_deg);
    }

    showFilter(filter.pose(), filter.accepted(), filter.rejected(),
               filter.positionSigmaIn());
    pros::delay(watch_period_ms);
  }
}

// ---------------------------------------------------------------------------
// The snapshot. One call reads every distance sensor a few times, works out
// the x and y that fit the readings best, and moves the odometry position
// there. It takes the heading from odometry as it is.
// ---------------------------------------------------------------------------

// The library's own settings, with a tighter limit on the correction. A
// SnapshotConfig you build yourself starts with field_mask 0, which drops
// every reading, and one sample per sensor: set both.
const snapshot::SnapshotConfig snapshot_config{
    .field_mask = snapshot::MAP_PERIMETER,
    // Five readings per sensor, 35 ms apart, and the median is used. The call
    // takes 140 ms.
    .samples = 5,
    .sample_delay_ms = 35,
    .min_sensors = 2,
    // Odometry is never this far out on this robot, so a longer correction
    // means a sensor saw something that is not on the map.
    .max_correction_in = 8.0f,
    .quadrant_margin_in = 2.0f,
};

void setUpSnapshot() {
  snapshot::snapshot_config_set_sensors(distance_sensors);
  snapshot::snapshot_config_set_config(snapshot_config);
  // The pose the snapshot starts from and corrects is the odometry pose.
  // Without this call every snapshot is refused.
  snapshot::snapshot_config_use_robot_state();

  // To a snapshot, a sensor on a wrong port and a sensor with no wall in
  // range look the same: no reading. This count tells the two apart, so check
  // it once, here.
  const std::size_t answering = snapshot::snapshot_config_live_sensor_count();
  const std::size_t configured = snapshot::snapshot_config_sensor_count();
  if (answering < configured) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 6,
                        "DISTANCE SENSORS: %d OF %d ANSWER",
                        static_cast<int>(answering),
                        static_cast<int>(configured));
  }
}

const char* refusalReason(snapshot::SnapshotReject reject) {
  switch (reject) {
    case snapshot::SnapshotReject::NONE: return "none";
    case snapshot::SnapshotReject::NO_SENSORS: return "no sensor list";
    case snapshot::SnapshotReject::TOO_FEW_SENSORS: return "too few readings";
    case snapshot::SnapshotReject::DEGENERATE: return "walls fix one axis";
    case snapshot::SnapshotReject::CHI2: return "readings disagree";
    case snapshot::SnapshotReject::CORRECTION_TOO_LARGE: return "too far";
    case snapshot::SnapshotReject::OFF_FIELD: return "off the field";
    case snapshot::SnapshotReject::NO_RUNTIME: return "no pose source";
  }
  return "unknown";
}

// Call it with the robot standing still. `quadrant` is the quarter of the
// field the robot is known to be in: BL and BR have y under 72 in, TL and TR
// have y over it, and the L ones have x under 72 in. The snapshot pulls its
// starting guess into that quarter. Quadrant::ANY starts from the odometry
// pose as it is.
void correctPoseFromSnapshot(snapshot::Quadrant quadrant) {
  const snapshot::SnapshotResult result =
      snapshot::snapshot_setpose_quadrant(quadrant);
  if (result.success) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 4,
                        "snapshot moved the pose %4.1f in, %d sensors   ",
                        result.correction_in,
                        static_cast<int>(result.used_samples));
  } else {
    // A refused snapshot leaves the pose as it was.
    pros::screen::print(pros::E_TEXT_MEDIUM, 4,
                        "snapshot refused: %-22s",
                        refusalReason(result.reject));
  }
}

// ---------------------------------------------------------------------------
// The wall reset
// ---------------------------------------------------------------------------

void resetAgainstBackWall() {
  // The wall along y = 0 fixes y and the heading. It says nothing about x, so
  // x keeps the value it has.
  const QLength x_now = chassis.getPose().x * 1_in;

  // -3 V backs into the wall until the motors stall, then sets the pose and
  // the IMU heading. A stall is the motors drawing 1000 mA while turning
  // slower than 5 RPM. Push too hard and the wheels slip against the wall, keep
  // turning, and never count: use the lowest voltage that moves the robot, and
  // a current under what the motors draw at it. Pass keep_current_heading in
  // place of 0_deg to leave the IMU alone.
  //
  // A game piece or another robot behind this one stalls the drive too. The
  // back distance sensor is 5 in behind the centre, so on the wall it reads
  // centre_to_back - 5 in. Anything else means something else stopped the
  // robot, and the pose is not reset.
  const mclib::control::WallDistanceCheck back_on_wall{
      [] { return back_distance.distance(); }, centre_to_back - 5_in, 1_in};
  const bool touched_wall =
      wallReset(x_now, centre_to_back, 0_deg, -3_V, 2_s, back_on_wall, 1000_mA);
  if (!touched_wall) {
    // No stall inside 2 s, or the back sensor did not read the wall. The pose
    // was left as it was.
    master.setText(2, 0, "NO WALL");
  }
}

void initialize() {
  // Calibrate before anything reads the IMU.
  imu->reset(true);

  // The start: backed up against the y = 0 wall, 36 in along it, facing up
  // the field. setPose() sets the IMU heading, the odometry pose and the
  // heading driveTo() holds together.
  //
  // The filter starts from the pose odometry has when startOdometry() runs.
  // When the pose is set later, by setPose(), a wall reset or a snapshot, the
  // odometry task restarts the filter at the new pose. A filter you step
  // yourself has to do the same, as watchPoseFilter() does.
  chassis.setPose(36_in, centre_to_back, 0_deg);

  auto odometry = mclib::control::odometrySetupFrom(chassis);
  odometry.vertical = mclib::control::TrackingWheelSensor{
      mclib::control::encoderReader(vertical_tracker), vertical_wheel};
  if (filter_corrects_odometry) {
    giveFilterTheGps();
    odometry.fusion = &fusion;
  }
  // False means the setup was rejected or odometry is already running: every
  // motion would then steer on a pose that never moves.
  if (!mclib::control::startOdometry(odometry)) {
    master.setText(1, 0, "ODOM NOT RUNNING");
  }
  if (!filter_corrects_odometry) {
    static pros::Task filter_watch(watchPoseFilter, "pose filter watch");
  }

  setUpSnapshot();

  static std::unique_ptr<Command> default_drive = drive.makeArcadeDriveCommand(
      master, mclib::control::DriveCurveConfig{.deadzone = 0.05, .gain = 5.0});
  CommandScheduler::registerSubsystem(&drive, default_drive.get());
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

  // The filter corrects odometry all through these moves. On a robot that
  // has stopped it moves the odometry position by 0.2 in a second at most, so
  // the pose holds still while a motion settles on it.
  moveToPoint(36_in, 48_in, 1, 2_s);

  // The snapshot is the full fix, at once, for a robot that has stopped.
  correctPoseFromSnapshot(snapshot::Quadrant::BL);

  moveToPoint(36_in, 18_in, -1, 2_s);
  turnToAngle(0_deg, 1_s);
  resetAgainstBackWall();

  moveToPoint(60_in, 36_in, 1, 2_s);
}

void opcontrol() {
  mclib::control::clearCancel(mclib::control::CancelToken::Motion);
  mclib::control::clearCancel(mclib::control::CancelToken::HeadingCorrection);

  constexpr std::uint32_t screen_period_ms = 100;
  std::uint32_t screen_updated_ms = 0;
  while (true) {
    CommandScheduler::run();

    // These four reads are safe from any task. The pose is in odometry
    // coordinates.
    if (filter_corrects_odometry &&
        pros::millis() - screen_updated_ms >= screen_period_ms) {
      screen_updated_ms = pros::millis();
      showFilter(fusion.pose(), fusion.accepted(), fusion.rejected(),
                 fusion.positionSigmaIn());
    }
    pros::delay(10);
  }
}
