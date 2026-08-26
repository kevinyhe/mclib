// mclib
#pragma once

#include "mclib/device/controller.hpp"
#include "mclib/device/distance.hpp"
#include "mclib/device/inertial.hpp"
#include "mclib/device/motor.hpp"
#include "mclib/device/motor_group.hpp"
#include "mclib/device/pneumatic.hpp"
#include "mclib/device/rotation.hpp"

extern mclib::device::Controller master;

extern mclib::device::MotorGroup left_chassis;
extern mclib::device::MotorGroup right_chassis;

extern mclib::device::Inertial inertial_sensor;
extern mclib::device::Motor intake_top_motor;
extern mclib::device::Motor intake_bottom_motor;
extern mclib::device::MotorGroup intake;

extern mclib::device::Distance left_reset;
extern mclib::device::Distance right_reset;

extern mclib::device::Rotation vertical_tracker;

extern mclib::device::Pneumatic flap;
extern mclib::device::Pneumatic middle_descore;
extern mclib::device::Pneumatic scraper;
extern mclib::device::Pneumatic hood;
extern mclib::device::Pneumatic wing;
extern mclib::device::Pneumatic tilter;

extern double distance_between_wheels;
extern double wheel_distance_in;
extern double vertical_tracker_diameter;
extern double vertical_tracker_dist_from_center;

extern double distance_kp;
extern double distance_ki;
extern double distance_kd;
extern double turn_kp;
extern double turn_ki;
extern double turn_kd;
extern double heading_correction_kp;
extern double heading_correction_ki;
extern double heading_correction_kd;

extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;
