// mclib
#include "main.h"

void initialize() {
  const mclib::Pose2D origin{};
  (void)origin;
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
