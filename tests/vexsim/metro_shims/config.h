#pragma once

// Metro's declarations, without its unused LemLib SDK dependency. Definitions
// and tuning remain in the original, externally supplied src/config.cpp.
#include "main.h"
#include "device/imu.hpp"

extern pros::Controller master;
extern pros::MotorGroup intake;
extern pros::MotorGroup left_chassis;
extern pros::MotorGroup right_chassis;
extern MockIMU inertial_sensor;
extern bool abort_autonomous;
extern pros::Motor intake_top_motor;
extern pros::Motor intake_bottom_motor;
extern pros::Distance left_reset;
extern pros::Distance right_reset;
extern pros::Rotation vertical_tracker;
extern pros::adi::DigitalOut flap;
extern pros::adi::DigitalOut wing;
extern pros::adi::DigitalOut scraper;
extern pros::adi::DigitalOut middle_descore;
extern pros::adi::DigitalOut tilter;
extern pros::adi::DigitalOut hood;
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double vertical_tracker_diameter;
extern double vertical_tracker_dist_from_center;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;
extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;
