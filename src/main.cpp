// mclib
#include "main.h"

void initialize() {
  // Calibrate before anything reads the IMU. The odometry tracks heading as
  // deltas from the pose it was reset to, while motion.cpp steers on raw
  // getInertialHeading() degrees; starting the two in different frames leaves
  // every moveToPoint aiming off by a constant.
  inertial_sensor.reset(true);
  inertial_sensor.setRotationDeg(0.0);

  // One odometry, on its own task, from here until the program ends. Nothing
  // else in mclib writes the pose.
  mclib::control::startOdometry();
  mclib::control::resetOdometry(mclib::Pose2D{});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  const mclib::Pose2D start{0.0, 0.0, 0.0};
  const mclib::Pose2D target = start + mclib::Pose2D{12.0, 6.0, 0.5};

  const double distance = target.distanceTo(start);
  const double heading = mclib::wrapAngle(target.theta);

  (void)distance;
  (void)heading;
}

void opcontrol() {
  mclib::Pose2D pose{0.0, 0.0, 0.0};

  while (true) {
    pose = pose + mclib::Pose2D{0.0, 0.0, 0.01};
    pose.theta = mclib::wrapAngle(pose.theta);
    pros::delay(20);
  }
}
