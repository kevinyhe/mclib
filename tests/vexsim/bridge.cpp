// Test-only ABI between the real mclib algorithms and vexsim's physics engine.
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion.hpp"
#include "mclib/control/motion_config.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/chassis/chassis_math.hpp"
#include <cmath>
#include <cstdint>

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

void refresh(std::uint32_t ms) {
  advance(ms, &sample);
  if (sample.cancel) mclib::control::requestCancel();
  last_integrated_pose = mclib::control::odometryTick(
      {drive.headingDeg() * std::acos(-1.0) / 180,
       drive.leftPositionDeg(), drive.rightPositionDeg(), sample.vertical, sample.horizontal});
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
