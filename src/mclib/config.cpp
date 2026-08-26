// mclib
#include "mclib/config.hpp"

mclib::device::Controller master(mclib::device::ControllerId::Master);

mclib::device::MotorGroup left_chassis({-11, 13, 14}, mclib::device::Gearset::Blue);
mclib::device::MotorGroup right_chassis({-16, 17, -18}, mclib::device::Gearset::Blue);

mclib::device::Inertial inertial_sensor(15, 0.9972299169);
mclib::device::Motor intake_top_motor(-20);
mclib::device::Motor intake_bottom_motor(-21);
mclib::device::MotorGroup intake({-2, 5}, mclib::device::Gearset::Blue);

mclib::device::Distance left_reset(10);
mclib::device::Distance right_reset(9);

mclib::device::Rotation vertical_tracker(-6);

mclib::device::Pneumatic flap('B');
mclib::device::Pneumatic middle_descore('C');
mclib::device::Pneumatic scraper('D');
mclib::device::Pneumatic hood('A');
mclib::device::Pneumatic wing('E');
mclib::device::Pneumatic tilter('F');

double distance_between_wheels = 11.375;
double wheel_distance_in = 9.06;
double vertical_tracker_diameter = 2;
double vertical_tracker_dist_from_center = 0.0;

double distance_kp = 0.4, distance_ki = 0, distance_kd = 3;
double turn_kp = 0.3, turn_ki = 0, turn_kd = 1.5;
double heading_correction_kp = 0.3, heading_correction_ki = 0, heading_correction_kd = 1.5;

bool heading_correction = true;
bool dir_change_start = true;
bool dir_change_end = true;
double min_output = 10;
double max_slew_accel_fwd = 1;
double max_slew_decel_fwd = 1;
double max_slew_accel_rev = 1;
double max_slew_decel_rev = 1;
double chase_power = 10;
