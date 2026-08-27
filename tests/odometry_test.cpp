// mclib
#include "mclib/control/odometry.hpp"

#include "mclib/control/robot_state.hpp"
#include "mclib/math.hpp"
#include "mclib/units/units.hpp"

#include "test_assert.hpp"

#include <cmath>
#include <cstdio>

// Odometry is arithmetic that compiles perfectly while being wrong, and the
// robot is the one place you cannot check it. Every number below is either
// closed-form geometry or a regression test for a specific bug in the code
// this replaced.

using mclib::Pose2D;
using mclib::control::Odometry;
using mclib::control::OdometryConfig;
using mclib::control::OdometrySample;

namespace {

constexpr double kPi = mclib::kPi;

/// @brief Track width of the reference chassis, inches (config.cpp value).
constexpr double kTrackWidthIn = 11.375;
/// @brief Inches of travel per revolution of a drive wheel (config.cpp value).
constexpr double kDriveInchesPerRev = 9.06;

/// @brief Encoder degrees for a given wheel travel, in inches.
double degForInches(double inches, double inches_per_rev) {
  return inches * 360.0 / inches_per_rev;
}

OdometryConfig driveConfig() {
  OdometryConfig config;
  config.drive_inches_per_revolution = kDriveInchesPerRev * mclib::units::inch;
  return config;
}

/**
 * @brief Feed an arc to a drive-encoder odometry in @p steps ticks.
 *
 * @param odom      Integrator, already reset to its starting pose.
 * @param start_rad Heading the encoders start at.
 * @param dtheta    Total heading change over the whole arc, radians.
 * @param centre_in Distance the tracking centre travels along the arc, inches.
 * @param steps     How many samples to break the arc into.
 */
void driveArc(Odometry& odom, double start_rad, double dtheta, double centre_in,
              int steps) {
  // Differential-drive kinematics: the left wheel is track/2 to the left of
  // the centre, so a clockwise turn makes it travel further.
  const double left_in = centre_in + dtheta * kTrackWidthIn * 0.5;
  const double right_in = centre_in - dtheta * kTrackWidthIn * 0.5;

  OdometrySample sample;
  sample.heading_rad = start_rad;
  sample.left_deg = 0.0;
  sample.right_deg = 0.0;
  odom.update(sample);  // baseline

  for (int i = 1; i <= steps; ++i) {
    const double f = static_cast<double>(i) / static_cast<double>(steps);
    sample.heading_rad = start_rad + dtheta * f;
    sample.left_deg = degForInches(left_in * f, kDriveInchesPerRev);
    sample.right_deg = degForInches(right_in * f, kDriveInchesPerRev);
    odom.update(sample);
  }
}

// -------------------------------------------------------------------------

void testFrameStraightAhead() {
  std::printf("-- straight 10 in at heading 0 lands on +Y, not +X\n");
  Odometry odom(driveConfig());
  odom.reset(Pose2D{});
  driveArc(odom, 0.0, 0.0, 10.0, 10);

  const Pose2D pose = odom.getPose();
  CHECK_NEAR(pose.x, 0.0, 1e-9);
  CHECK_NEAR(pose.y, 10.0, 1e-9);
  CHECK_NEAR(pose.theta, 0.0, 1e-12);
  std::printf("   pose = (%.10f, %.10f, %.10f)\n", pose.x, pose.y, pose.theta);
}

void testFrameMatchesHeadingVector() {
  std::printf("-- straight 10 in at 35 deg equals headingVector(35 deg) * 10\n");
  const double heading = 35.0 * kPi / 180.0;
  Odometry odom(driveConfig());
  odom.reset(Pose2D{0.0, 0.0, heading});
  driveArc(odom, heading, 0.0, 10.0, 10);

  const mclib::Vec2 expected = mclib::headingVector(heading) * 10.0;
  const Pose2D pose = odom.getPose();
  CHECK_NEAR(pose.x, expected.x(), 1e-9);
  CHECK_NEAR(pose.y, expected.y(), 1e-9);
  std::printf("   pose = (%.10f, %.10f), headingVector*10 = (%.10f, %.10f)\n",
              pose.x, pose.y, expected.x(), expected.y());
}

void testQuarterArcClosedForm() {
  std::printf("-- 90 deg arc of radius 24 in, against closed-form geometry\n");
  // Start at the origin facing +Y, turn 90 deg clockwise about a centre 24 in
  // to the robot's right. The circle centre is (24, 0); the robot ends a
  // quarter turn round it, at (24, 24), facing +X.
  const double radius = 24.0;
  const double dtheta = kPi / 2.0;

  Odometry odom(driveConfig());
  odom.reset(Pose2D{});
  driveArc(odom, 0.0, dtheta, radius * dtheta, 200);

  const Pose2D pose = odom.getPose();
  // The chord formula is exact for constant curvature, so splitting the arc
  // finely does not buy accuracy and does not lose any either.
  CHECK_NEAR(pose.x, 24.0, 1e-9);
  CHECK_NEAR(pose.y, 24.0, 1e-9);
  CHECK_NEAR(pose.theta, dtheta, 1e-12);
  std::printf("   pose = (%.10f, %.10f, %.10f), exact (24, 24, %.10f)\n",
              pose.x, pose.y, pose.theta, dtheta);

  // One single sample across the whole 90 deg lands on the same point. A
  // midpoint approximation - what Chassis::updateOdometry() used to do -
  // would put this at (26.65, 26.65), 3.7 in out.
  Odometry coarse(driveConfig());
  coarse.reset(Pose2D{});
  driveArc(coarse, 0.0, dtheta, radius * dtheta, 1);
  const Pose2D coarse_pose = coarse.getPose();
  CHECK_NEAR(coarse_pose.x, 24.0, 1e-9);
  CHECK_NEAR(coarse_pose.y, 24.0, 1e-9);
  std::printf("   single-step pose = (%.10f, %.10f)\n", coarse_pose.x,
              coarse_pose.y);
}

void testInPlaceRotationHasNoTranslation() {
  std::printf("-- BUG 1 regression: in-place spin must not translate\n");
  // The old trackNoOdomWheel() added +track/2 to BOTH sides. Spinning in place
  // then produced an average radius of track/2 instead of zero, so a 360 deg
  // pivot walked the pose most of a track width. This is the test that fails
  // with the +/+ formula and passes with the signs done right.
  Odometry odom(driveConfig());
  odom.reset(Pose2D{});
  driveArc(odom, 0.0, 2.0 * kPi, 0.0, 360);

  const Pose2D pose = odom.getPose();
  CHECK_NEAR(pose.x, 0.0, 1e-9);
  CHECK_NEAR(pose.y, 0.0, 1e-9);
  const double drift = std::hypot(pose.x, pose.y);
  std::printf("   full pivot drift = %.12f in (the +/+ formula gives ~%.3f)\n",
              drift, kTrackWidthIn * kPi);
  CHECK(drift < 1e-9);

  // Quarter pivot, one coarse step, same answer.
  Odometry quarter(driveConfig());
  quarter.reset(Pose2D{});
  driveArc(quarter, 0.0, kPi / 2.0, 0.0, 1);
  CHECK_NEAR(quarter.getPose().x, 0.0, 1e-12);
  CHECK_NEAR(quarter.getPose().y, 0.0, 1e-12);
  CHECK_NEAR(quarter.getPose().theta, kPi / 2.0, 1e-12);
}

void testNonZeroStartHeadingHasNoFirstTickJump() {
  std::printf("-- BUG 2 regression: a non-zero starting heading is not a delta\n");
  // The old code set prev_heading_rad = 0 without reading the IMU, so a robot
  // starting at 90 deg saw a 90 deg heading change on tick one and swung the
  // pose. Here the first sample only seeds the baseline.
  const double start = kPi / 2.0;  // facing +X
  Odometry odom(driveConfig());
  odom.reset(Pose2D{0.0, 0.0, start});

  OdometrySample sample;
  sample.heading_rad = start;
  sample.left_deg = 1234.5;  // encoders are not at zero either
  sample.right_deg = -987.6;

  const Pose2D after_first = odom.update(sample);
  CHECK_EQ(after_first.x, 0.0);
  CHECK_EQ(after_first.y, 0.0);
  CHECK_NEAR(after_first.theta, start, 1e-15);
  std::printf("   after first sample = (%.10f, %.10f, %.10f)\n", after_first.x,
              after_first.y, after_first.theta);

  // The second sample, unchanged, still moves nothing.
  const Pose2D after_second = odom.update(sample);
  CHECK_EQ(after_second.x, 0.0);
  CHECK_EQ(after_second.y, 0.0);

  // And a real 10 in move from there goes along +X, not somewhere bent.
  const double ten_in_deg = degForInches(10.0, kDriveInchesPerRev);
  sample.left_deg += ten_in_deg;
  sample.right_deg += ten_in_deg;
  const Pose2D moved = odom.update(sample);
  CHECK_NEAR(moved.x, 10.0, 1e-9);
  CHECK_NEAR(moved.y, 0.0, 1e-9);
  std::printf("   after a 10 in move = (%.10f, %.10f)\n", moved.x, moved.y);
}

void testNanHeadingDoesNotPoisonPose() {
  std::printf("-- BUG 3 regression: a NAN IMU reading does not poison the pose\n");
  // device::Inertial::getRotationDeg() returns NAN on PROS_ERR_F. Nothing used
  // to check, so one dropout made every later pose NaN for the rest of the
  // match.
  Odometry odom(driveConfig());
  odom.reset(Pose2D{});

  OdometrySample sample;
  sample.heading_rad = 0.0;
  odom.update(sample);

  const double five_in_deg = degForInches(5.0, kDriveInchesPerRev);
  sample.left_deg = five_in_deg;
  sample.right_deg = five_in_deg;
  const Pose2D good = odom.update(sample);
  CHECK_NEAR(good.y, 5.0, 1e-9);

  // Two faulted samples in a row.
  OdometrySample faulted = sample;
  faulted.heading_rad = NAN;
  faulted.left_deg = 2.0 * five_in_deg;
  faulted.right_deg = 2.0 * five_in_deg;
  const Pose2D during = odom.update(faulted);
  CHECK(std::isfinite(during.x));
  CHECK(std::isfinite(during.y));
  CHECK_NEAR(during.y, 5.0, 1e-9);

  faulted.heading_rad = INFINITY;
  odom.update(faulted);
  CHECK_EQ(static_cast<double>(odom.faultCount()), 2.0);

  // The IMU comes back. The travel that happened during the dropout is caught
  // up, not lost: 10 in total from the start, not 5.
  OdometrySample recovered = faulted;
  recovered.heading_rad = 0.0;
  const Pose2D after = odom.update(recovered);
  CHECK(std::isfinite(after.x));
  CHECK(std::isfinite(after.y));
  CHECK_NEAR(after.x, 0.0, 1e-9);
  CHECK_NEAR(after.y, 10.0, 1e-9);
  std::printf("   during dropout = (%.10f, %.10f), after recovery = (%.10f, %.10f)\n",
              during.x, during.y, after.x, after.y);

  // A NaN encoder reading is rejected the same way.
  OdometrySample bad_encoder = recovered;
  bad_encoder.left_deg = NAN;
  const Pose2D unchanged = odom.update(bad_encoder);
  CHECK_NEAR(unchanged.y, 10.0, 1e-9);
  CHECK_EQ(static_cast<double>(odom.faultCount()), 3.0);
}

void testVerticalTrackingWheel() {
  std::printf("-- one vertical tracking wheel, offset from the centre\n");
  OdometryConfig config;
  config.use_vertical_tracker = true;
  config.vertical_circumference = 2.0 * mclib::units::pi * mclib::units::inch;
  config.vertical_offset_right = 3.0 * mclib::units::inch;
  const double circumference = config.vertical_circumference.in();

  Odometry odom(config);
  odom.reset(Pose2D{});

  OdometrySample sample;
  sample.heading_rad = 0.0;
  sample.vertical_deg = 0.0;
  odom.update(sample);

  // Straight 10 in: an offset wheel reads the same as a centred one.
  sample.vertical_deg = degForInches(10.0, circumference);
  Pose2D pose = odom.update(sample);
  CHECK_NEAR(pose.x, 0.0, 1e-12);
  CHECK_NEAR(pose.y, 10.0, 1e-9);
  std::printf("   straight = (%.10f, %.10f)\n", pose.x, pose.y);

  // Pivot in place by 90 deg clockwise. A wheel 3 in to the right of the
  // centre of rotation rolls BACKWARDS by dtheta * 3 = -4.712 in, and the
  // centre must not move.
  odom.reset(Pose2D{});
  sample.heading_rad = 0.0;
  sample.vertical_deg = 0.0;
  odom.update(sample);

  const double dtheta = kPi / 2.0;
  sample.heading_rad = dtheta;
  sample.vertical_deg = degForInches(-dtheta * 3.0, circumference);
  pose = odom.update(sample);
  CHECK_NEAR(pose.x, 0.0, 1e-9);
  CHECK_NEAR(pose.y, 0.0, 1e-9);
  CHECK_NEAR(pose.theta, dtheta, 1e-12);
  std::printf("   pivot with a +3 in offset wheel = (%.12f, %.12f)\n", pose.x,
              pose.y);

  // With the offset forgotten (set to zero) the same reading would fake a
  // 4.24 in translation. Show the number so the offset term is visibly doing
  // work.
  OdometryConfig no_offset = config;
  no_offset.vertical_offset_right = 0.0 * mclib::units::inch;
  Odometry naive(no_offset);
  naive.reset(Pose2D{});
  OdometrySample s2;
  s2.heading_rad = 0.0;
  s2.vertical_deg = 0.0;
  naive.update(s2);
  s2.heading_rad = dtheta;
  s2.vertical_deg = degForInches(-dtheta * 3.0, circumference);
  const Pose2D naive_pose = naive.update(s2);
  std::printf("   same reading with the offset ignored = (%.6f, %.6f)\n",
              naive_pose.x, naive_pose.y);
  CHECK(std::hypot(naive_pose.x, naive_pose.y) > 4.0);
}

void testHorizontalTrackingWheel() {
  std::printf("-- two tracking wheels: a pure sideways push is captured\n");
  OdometryConfig config;
  config.use_vertical_tracker = true;
  config.vertical_circumference = 2.0 * mclib::units::pi * mclib::units::inch;
  config.use_horizontal_tracker = true;
  config.horizontal_circumference = 2.0 * mclib::units::pi * mclib::units::inch;
  const double circumference = config.horizontal_circumference.in();

  Odometry odom(config);
  odom.reset(Pose2D{});

  OdometrySample sample;
  odom.update(sample);

  // Pushed 4 in to the robot's right with no heading change and no forward
  // travel. A vertical-only odometry reports nothing at all here.
  sample.horizontal_deg = degForInches(4.0, circumference);
  Pose2D pose = odom.update(sample);
  CHECK_NEAR(pose.x, 4.0, 1e-9);
  CHECK_NEAR(pose.y, 0.0, 1e-12);
  std::printf("   push right at heading 0 = (%.10f, %.10f)\n", pose.x, pose.y);

  // Same push while facing +X (90 deg): it should come out along -Y.
  odom.reset(Pose2D{0.0, 0.0, kPi / 2.0});
  sample.heading_rad = kPi / 2.0;
  odom.update(sample);
  sample.horizontal_deg += degForInches(4.0, circumference);
  pose = odom.update(sample);
  CHECK_NEAR(pose.x, 0.0, 1e-12);
  CHECK_NEAR(pose.y, -4.0, 1e-9);
  std::printf("   push right at 90 deg = (%.10f, %.10f)\n", pose.x, pose.y);

  // A horizontal wheel 5 in ahead of the centre of rotation rolls sideways
  // during a pure pivot; the offset term must cancel it.
  OdometryConfig offset = config;
  offset.horizontal_offset_forward = 5.0 * mclib::units::inch;
  Odometry odom2(offset);
  odom2.reset(Pose2D{});
  OdometrySample s;
  odom2.update(s);
  const double dtheta = kPi / 2.0;
  s.heading_rad = dtheta;
  s.horizontal_deg = degForInches(dtheta * 5.0, circumference);
  const Pose2D pivot = odom2.update(s);
  CHECK_NEAR(pivot.x, 0.0, 1e-9);
  CHECK_NEAR(pivot.y, 0.0, 1e-9);
  std::printf("   pivot with a +5 in forward lateral wheel = (%.12f, %.12f)\n",
              pivot.x, pivot.y);
}

void testResetIsNotADelta() {
  std::printf("-- reset re-seeds the encoder baseline\n");
  Odometry odom(driveConfig());
  odom.reset(Pose2D{});

  OdometrySample sample;
  odom.update(sample);
  const double ten_in_deg = degForInches(10.0, kDriveInchesPerRev);
  sample.left_deg = ten_in_deg;
  sample.right_deg = ten_in_deg;
  odom.update(sample);
  CHECK_NEAR(odom.getPose().y, 10.0, 1e-9);

  // Teleport. The encoders have not moved, and the next tick must not
  // integrate the 10 in of history all over again.
  odom.reset(Pose2D{50.0, -20.0, kPi});
  CHECK(!odom.hasBaseline());
  Pose2D pose = odom.update(sample);
  CHECK_EQ(pose.x, 50.0);
  CHECK_EQ(pose.y, -20.0);
  pose = odom.update(sample);
  CHECK_EQ(pose.x, 50.0);
  CHECK_EQ(pose.y, -20.0);
  std::printf("   after reset + two ticks = (%.10f, %.10f, %.10f)\n", pose.x,
              pose.y, pose.theta);

  // A NaN heading in the reset pose keeps the heading we had.
  odom.reset(Pose2D{1.0, 2.0, NAN});
  CHECK(std::isfinite(odom.getPose().theta));
  CHECK_NEAR(std::fabs(odom.getPose().theta), kPi, 1e-12);
}

void testTickPublishesToRobotState() {
  std::printf("-- odometryTick() publishes into the one RobotState\n");
  mclib::control::setOdometryConfig(driveConfig());
  mclib::control::resetOdometry(Pose2D{});
  CHECK_EQ(mclib::control::robotState().pose().x, 0.0);
  CHECK_EQ(mclib::control::robotState().pose().y, 0.0);

  OdometrySample sample;
  mclib::control::odometryTick(sample);
  const double seven_in_deg = degForInches(7.0, kDriveInchesPerRev);
  sample.left_deg = seven_in_deg;
  sample.right_deg = seven_in_deg;
  mclib::control::odometryTick(sample);

  const Pose2D published = mclib::control::robotState().pose();
  CHECK_NEAR(published.y, 7.0, 1e-9);
  CHECK_NEAR(published.x, 0.0, 1e-12);
  std::printf("   robotState().pose() = (%.10f, %.10f)\n", published.x,
              published.y);
}

}  // namespace

int main() {
  testFrameStraightAhead();
  testFrameMatchesHeadingVector();
  testQuarterArcClosedForm();
  testInPlaceRotationHasNoTranslation();
  testNonZeroStartHeadingHasNoFirstTickJump();
  testNanHeadingDoesNotPoisonPose();
  testVerticalTrackingWheel();
  testHorizontalTrackingWheel();
  testResetIsNotADelta();
  testTickPublishesToRobotState();
  return mclib::test::summary("odometry");
}
