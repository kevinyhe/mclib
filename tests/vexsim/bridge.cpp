// Test-only ABI between the real mclib algorithms and vexsim's physics engine.
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion.hpp"
#include "mclib/control/motion_config.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/chassis/chassis_math.hpp"
#include "mclib/control/pose_filter.hpp"
#include "mclib/control/ramsete.hpp"
#include "mclib/path/arc.hpp"
#include "mclib/path/spline.hpp"
#include "mclib/path/trajectory.hpp"
#include "mclib/math.hpp"
#include <cmath>
#include <cstdint>
#include <memory>

struct Sample {
  double heading, left, right, current_ma, velocity_rpm;
  std::uint32_t millis;
  int disabled, cancel;
  double vertical, horizontal;
};
using Advance = void (*)(std::uint32_t, Sample*);
using Output = void (*)(int side, int mode, double volts);

namespace {
Sample sample{};
Advance advance = nullptr;
Output output = nullptr;
mclib::Pose2D last_integrated_pose{};
double boomerang_lead = 0.5;
mclib::control::RamseteConfig ramsete_config;

// PoseFusion in the loop, the same class the odometry task runs on a robot.
// The simulated robot starts at the field centre, which is the odometry
// origin, so the snapshot map's field frame is odometry + 72 in.
constexpr double kFieldOffsetIn = 72.0;
struct FusionRig {
  std::unique_ptr<mclib::control::PoseFusion> fusion;
  std::vector<double> readings;  // latest, -1 = none since last tick
} fusion_rig;
mclib::path::TrajectoryConstraints ramsete_limits;

struct SimDrive : mclib::control::DriveHardware {
  mclib::units::DriveGeometry geometry{
      mclib::units::Wheel::fromDiameter(3.25 * mclib::units::inch),
      11.5 * mclib::units::inch, 1.0};
  double heading_offset = 0, left_offset = 0, right_offset = 0;
  bool encoder_heading = false;
  void setDriveVoltage(double left, double right) override {
    output(0, 0, left); output(1, 0, right);
  }
  void setSideVoltage(bool left, double volts) override { output(left ? 0 : 1, 0, volts); }
  void brakeDrive(mclib::device::BrakeMode mode) override {
    brakeSide(true, mode); brakeSide(false, mode);
  }
  void brakeSide(bool left, mclib::device::BrakeMode mode) override {
    using mclib::device::BrakeMode;
    output(left ? 0 : 1, mode == BrakeMode::Coast ? 1 : mode == BrakeMode::Brake ? 2 : 3, 0);
  }
  void tareDrive() override {
    const double heading = headingDeg();
    left_offset = sample.left; right_offset = sample.right;
    setHeadingDeg(heading);
  }
  double leftPositionDeg() override { return sample.left - left_offset; }
  double rightPositionDeg() override { return sample.right - right_offset; }
  double rawHeading() {
    using namespace mclib::units;
    if (!encoder_heading) return sample.heading;
    return mclib::chassis_math::encoderHeadingDeg(
        geometry.encoderToDistance(leftPositionDeg() * degree).in(),
        geometry.encoderToDistance(rightPositionDeg() * degree).in(), geometry.track_width.in());
  }
  double headingDeg() override { return rawHeading() + heading_offset; }
  void setHeadingDeg(double heading) override { heading_offset = heading - rawHeading(); }
  std::vector<double> driveCurrentsMa() override { return {sample.current_ma}; }
  std::vector<double> driveVelocitiesRpm() override { return {sample.velocity_rpm}; }
  const mclib::units::DriveGeometry& driveGeometry() const override { return geometry; }
} drive;

// What startOdometry() does with OdometrySetup::fusion, after each tick.
void runFusion() {
  const auto corrected =
      fusion_rig.fusion->step(mclib::control::robotState().pose(), sample.millis);
  if (corrected.has_value())
    mclib::control::correctOdometryPosition(corrected->x(), corrected->y());
}

void refresh(std::uint32_t ms) {
  advance(ms, &sample);
  if (sample.cancel) mclib::control::requestCancel();
  last_integrated_pose = mclib::control::odometryTick(
      {drive.headingDeg() * std::acos(-1.0) / 180,
       drive.leftPositionDeg(), drive.rightPositionDeg(), sample.vertical, sample.horizontal});
  if (fusion_rig.fusion) runFusion();
}
}

namespace pros {
extern "C" std::uint32_t millis() { return sample.millis; }
extern "C" void delay(std::uint32_t ms) { refresh(ms); }
namespace competition {
std::uint8_t is_disabled() { return sample.disabled; }
std::uint8_t is_autonomous() { return 1; }
std::uint8_t get_status() { return 2 | (sample.disabled ? 1 : 0); }
}
}
namespace mclib::time {
std::uint32_t systemMillis() { return sample.millis; }
}

extern "C" void sim_init(Advance step, Output write, double diameter_in,
                          double track_in, double ratio, int encoder_heading) {
  using namespace mclib::units;
  using namespace mclib::control;
  advance = step; output = write;
  sample = {};
  boomerang_lead = 0.5;
  ramsete_config = {};
  ramsete_limits = {};
  // The arc switch is library-global; each simulated robot starts without it.
  mclib::control::useEncoderArcs();
  fusion_rig = FusionRig{};
  drive.geometry = {Wheel::fromDiameter(diameter_in * inch), track_in * inch, ratio};
  drive.heading_offset = drive.left_offset = drive.right_offset = 0;
  drive.encoder_heading = encoder_heading;
  bindDrive(&drive);
  clearCancel(); clearCancel(CancelToken::HeadingCorrection);
  robotState().clearMotionOutputs();
  robotState().setCorrectAngleDeg(0);
  setMotionConfig(MotionConfig{});
  OdometryConfig config;
  config.drive_inches_per_revolution = drive.geometry.encoderToDistance(360 * degree);
  setOdometryConfig(config);
  resetOdometry({0, 0, 0});
  refresh(0);
}

namespace {
// Circular arcs as baked PathPoints with exact heading and curvature: the
// geometry curveCircle() drives, so the two can be compared on one target.
// Planned in the field frame from the origin, facing +Y, so a scenario can
// start the robot off the path and see whether the follower recovers.
struct ArcEnd { double x, y, travel; };

// Appends a turn of @p turn_rad at @p radius_in, leaving the travel direction
// @p start.travel. Positive turns right.
ArcEnd appendArc(std::vector<mclib::path::PathPoint>& points, ArcEnd start,
                 double turn_rad, double radius_in) {
  using namespace mclib::units;
  const double r = (turn_rad < 0 ? -1.0 : 1.0) * std::fabs(radius_in);
  // Centre is r along the travel direction's right-hand side.
  const double cx = start.x + r * std::cos(start.travel);
  const double cy = start.y - r * std::sin(start.travel);
  const int samples = std::max(2, static_cast<int>(std::fabs(turn_rad * r)) * 2);
  for (int i = points.empty() ? 0 : 1; i <= samples; ++i) {
    const double phi = start.travel + turn_rad * i / samples;
    mclib::path::PathPoint point;
    point.x = (cx - r * std::cos(phi)) * inch;
    point.y = (cy + r * std::sin(phi)) * inch;
    point.heading = QAngle::fromBase(mclib::wrapAngle(phi));
    point.curvature = QCurvature::fromBase(1.0 / (r * inch).raw());
    points.push_back(point);
  }
  const double end = start.travel + turn_rad;
  return {cx - r * std::cos(end), cy + r * std::sin(end), end};
}

// Right then left, 90 deg each: from the origin to (2r, 2r), facing +Y again.
mclib::path::Path sCurvePath(double radius_in) {
  const double pi = std::acos(-1.0);
  std::vector<mclib::path::PathPoint> points;
  const ArcEnd mid = appendArc(points, {0, 0, 0}, pi / 2, radius_in);
  appendArc(points, mid, -pi / 2, radius_in);
  return mclib::path::Path(points);
}
}  // namespace

// Follow a centripetal Catmull-Rom spline from the current odometry position
// through @p n field points (x, y pairs, inches) with followTrajectory(), the
// same way an autonomous would plan one. Returns the MotionResult as an int,
// or -1 when the points don't make a followable path.
extern "C" int sim_follow_spline(const double* xy, int n, int reversed,
                                 double timeout_ms, double volts) {
  using namespace mclib::units;
  const auto pose = mclib::control::robotState().pose();
  std::vector<mclib::path::Waypoint> waypoints{{pose.x * inch, pose.y * inch}};
  for (int i = 0; i < n; ++i) waypoints.push_back({xy[2 * i] * inch, xy[2 * i + 1] * inch});
  auto limits = ramsete_limits;
  limits.reversed = reversed != 0;
  const auto trajectory =
      mclib::path::Trajectory::generate(mclib::path::generateSpline(waypoints), limits);
  if (trajectory.empty()) return -1;
  return static_cast<int>(followTrajectory(trajectory, ramsete_config,
                                           timeout_ms * millisecond, true, volts * volt));
}

// Run a PoseFusion alongside odometry with @p n distance sensors, given as
// (x_right_in, y_fwd_in, rel_deg) triples, correcting odometry every tick.
extern "C" void sim_filter_enable(const double* sensors, int n, double odom_var_per_in,
                                  double correction_floor_in, double correction_per_in) {
  fusion_rig = FusionRig{};
  fusion_rig.readings.assign(n, -1);
  mclib::control::PoseFusionConfig config;
  config.filter.odom_along_var_per_in = config.filter.odom_side_var_per_in = odom_var_per_in;
  config.field_offset_x_in = config.field_offset_y_in = kFieldOffsetIn;
  config.correction_floor_in = correction_floor_in;
  config.correction_per_in = correction_per_in;
  std::vector<mclib::control::DistanceSensorInput> inputs;
  for (int i = 0; i < n; ++i) {
    mclib::control::DistanceSensorInput input;
    input.geometry.x_right_in = static_cast<float>(sensors[3 * i]);
    input.geometry.y_fwd_in = static_cast<float>(sensors[3 * i + 1]);
    input.geometry.rel_deg = static_cast<float>(sensors[3 * i + 2]);
    // Python supplies each reading once; take it and clear it.
    input.read_in = [i]() {
      const double value = fusion_rig.readings[i];
      fusion_rig.readings[i] = -1;
      return value;
    };
    input.period_ms = 0;
    inputs.push_back(input);
  }
  fusion_rig.fusion = std::make_unique<mclib::control::PoseFusion>(config, inputs);
  fusion_rig.fusion->reset(mclib::control::robotState().pose());
}

// A distance reading for sensor @p index, used on the next tick. Called from
// the Python advance() callback.
extern "C" void sim_filter_reading(int index, double reading_in) {
  if (index >= 0 && static_cast<std::size_t>(index) < fusion_rig.readings.size())
    fusion_rig.readings[index] = reading_in;
}

// Filter pose in the odometry frame, its accepted / rejected counts, sigma.
extern "C" void sim_filter_state(double* out) {
  const mclib::Pose2D p = fusion_rig.fusion->pose();
  out[0] = p.x;
  out[1] = p.y;
  out[2] = p.theta * 180 / std::acos(-1.0);
  out[3] = fusion_rig.fusion->accepted();
  out[4] = fusion_rig.fusion->rejected();
  out[5] = fusion_rig.fusion->positionSigmaIn();
}

// Make curveCircle() / curveCircleReverse() use RAMSETE with the current
// sim_set_ramsete() tuning (on != 0), or the encoder arcs.
extern "C" void sim_arcs_use_ramsete(int on) {
  if (on) mclib::control::useRamseteForArcs(ramsete_config, ramsete_limits);
  else mclib::control::useEncoderArcs();
}

// Plan (don't drive) a spline from (sx, sy) through @p n points with the
// current sim_set_ramsete() limits. Writes up to @p max states as
// (t s, x in, y in, speed in/s), about every 20 ms, into @p out and returns
// how many; -1 when the points don't make a path.
extern "C" int sim_plan_spline(double sx, double sy, const double* xy, int n, int reversed,
                               double* out, int max) {
  using namespace mclib::units;
  std::vector<mclib::path::Waypoint> waypoints{{sx * inch, sy * inch}};
  for (int i = 0; i < n; ++i) waypoints.push_back({xy[2 * i] * inch, xy[2 * i + 1] * inch});
  auto limits = ramsete_limits;
  limits.reversed = reversed != 0;
  const auto trajectory =
      mclib::path::Trajectory::generate(mclib::path::generateSpline(waypoints), limits);
  if (trajectory.empty() || max < 2) return -1;
  const double duration = trajectory.duration().s();
  const int count = std::min(max, std::max(2, static_cast<int>(duration / 0.02) + 1));
  for (int i = 0; i < count; ++i) {
    const double t = duration * i / (count - 1);
    const auto state = trajectory.sample(t * second);
    out[4 * i] = t;
    out[4 * i + 1] = state.x.in();
    out[4 * i + 2] = state.y.in();
    out[4 * i + 3] = std::fabs(state.velocity.inps());
  }
  return count;
}

extern "C" void sim_set_ramsete(double ks_v, double kv_v_per_ips, double b, double zeta,
                                double max_ips, double max_ips2, double max_lateral_ips2,
                                double track_in) {
  using namespace mclib::units;
  ramsete_config.track_width = track_in * inch;
  ramsete_config.feedforward.kS = ks_v * volt;
  ramsete_config.feedforward.kV = kv_v_per_ips * volt / inps;
  ramsete_config.gains = {b, zeta};
  ramsete_limits.max_velocity = max_ips * inps;
  ramsete_limits.max_acceleration = max_ips2 * mclib::control::inps2;
  ramsete_limits.max_lateral_acceleration = max_lateral_ips2 * mclib::control::inps2;
  ramsete_limits.track_width = drive.geometry.track_width;
}

extern "C" int sim_run(int action, double a, double b, double heading,
                        double timeout_ms, int direction, int exit,
                        double volts, double current_threshold_ma) {
  using namespace mclib::units;
  const auto limit = timeout_ms * millisecond;
  switch (action) {
    case 0: turnToAngle(a * degree, limit, exit, volts * volt); break;
    case 1: driveTo(a * inch, limit, exit, volts * volt); break;
    case 2: curveCircle(a * degree, b * inch, limit, exit, volts * volt); break;
    case 3: curveCircleReverse(a * degree, b * inch, limit, exit, volts * volt); break;
    case 4: swing(a * degree, direction, limit, exit, volts * volt); break;
    case 5: turnToPoint(a * inch, b * inch, direction, limit); break;
    case 6: moveToPoint(a * inch, b * inch, direction, limit, exit, volts * volt); break;
    case 7: boomerang(a * inch, b * inch, direction, heading * degree, boomerang_lead,
                       limit, exit, volts * volt); break;
    case 9:
    case 10:
    case 13: {
      // Planned from the origin so an off-path start stays off the path.
      // Actions 9 and 10 differ only in the scenario's name: planArc() works
      // out the travel direction from the signs, as curveCircle() does.
      const mclib::path::ArcPlan plan =
          action == 13 ? mclib::path::ArcPlan{sCurvePath(b), false}
                       : mclib::path::planArc(mclib::Pose2D{0, 0, 0}, a * degree, b * inch);
      auto limits = ramsete_limits;
      limits.reversed = plan.reversed;
      const auto trajectory = mclib::path::Trajectory::generate(plan.path, limits);
      return static_cast<int>(followTrajectory(trajectory, ramsete_config, limit, exit,
                                               volts * volt));
    }
    case 11: {
      // Open-loop: both sides at `volts` for `timeout_ms`, to measure kS/kV.
      drive.setDriveVoltage(volts, volts);
      for (double t = 0; t < timeout_ms; t += 10) pros::delay(10);
      drive.setDriveVoltage(0, 0);
      return 0;
    }
    case 12: {
      // Open-loop spin in place: left at +volts, right at -volts.
      drive.setDriveVoltage(volts, -volts);
      for (double t = 0; t < timeout_ms; t += 10) pros::delay(10);
      drive.setDriveVoltage(0, 0);
      return 0;
    }
    case 8: return wallReset(a * inch, b * inch, heading * degree, volts * volt,
                             limit, current_threshold_ma * milliampere, 5 * rpm);
    default: return -1;
  }
  return 0;
}

extern "C" void sim_pose(double* pose) {
  const auto value = mclib::control::robotState().pose();
  pose[0] = value.x; pose[1] = value.y; pose[2] = value.theta * 180 / std::acos(-1.0);
}

// Fixed ABI: phase, target heading, heading error, remaining, carrot x/y,
// pre-mix drive/yaw requests, slew-limited, voltage-limited. Missing = NaN.
// Phase IDs and request semantics are documented in robot_state.hpp.
extern "C" void sim_motion_telemetry(double* values) {
  const auto value = mclib::control::robotState().motionTelemetry();
  values[0] = static_cast<int>(value.phase);
  values[1] = value.target_heading_deg;
  values[2] = value.heading_error_deg;
  values[3] = value.remaining_in;
  values[4] = value.carrot_x;
  values[5] = value.carrot_y;
  values[6] = value.drive_volts;
  values[7] = value.yaw_volts;
  values[8] = value.slew_limited;
  values[9] = value.voltage_limited;
}

extern "C" void sim_correct(double x, double y) {
  mclib::control::correctOdometryPosition(x, y);
}

extern "C" void sim_refresh(std::uint32_t ms) { refresh(ms); }

extern "C" void sim_set_pose(double x, double y, double heading) {
  advance(0, &sample);
  drive.setHeadingDeg(heading);
  mclib::control::resetOdometry({x, y, heading * std::acos(-1.0) / 180});
  mclib::control::robotState().setCorrectAngleDeg(heading);
  refresh(0);
}

extern "C" void sim_set_trackers(double vertical_circumference,
                                  double vertical_right_offset,
                                  double horizontal_circumference,
                                  double horizontal_forward_offset) {
  using namespace mclib::units;
  auto config = mclib::control::getOdometryConfig();
  config.use_vertical_tracker = config.use_horizontal_tracker = true;
  config.vertical_circumference = vertical_circumference * inch;
  config.vertical_offset_right = vertical_right_offset * inch;
  config.horizontal_circumference = horizontal_circumference * inch;
  config.horizontal_offset_forward = horizontal_forward_offset * inch;
  mclib::control::setOdometryConfig(config);
  refresh(0);
}

extern "C" void sim_set_gains(const double* gains) {
  auto config = mclib::control::motionConfig();
  config.distance_pid = {gains[0], gains[1], gains[2]};
  config.heading_pid = {gains[3], gains[4], gains[5]};
  config.turn_pid = {gains[6], gains[7], gains[8]};
  mclib::control::setMotionConfig(config);
}

extern "C" void sim_set_lead(double lead) { boomerang_lead = lead; }

extern "C" double sim_heading_target() {
  return mclib::control::robotState().correctAngleDeg();
}

extern "C" void sim_last_integrated_pose(double* pose) {
  pose[0] = last_integrated_pose.x;
  pose[1] = last_integrated_pose.y;
  pose[2] = last_integrated_pose.theta * 180 / std::acos(-1.0);
}
