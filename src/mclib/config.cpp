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

// Legacy geometry globals. motion.cpp and odometry.cpp read these directly
// (10 reads of wheel_distance_in / distance_between_wheels across the two
// files, plus one each for the tracker pair), so they stay for now; flipping
// those call sites over is Phase 3.
//
// They are no longer independent values. Each one is derived from
// `mclib::config::robot_drive_geometry` / `vertical_tracking_wheel` in
// config.hpp, which state this geometry once, in units, with diameter and
// circumference distinguished by type. Edit the constants there; there is
// nothing to keep in sync here.
//
// The initialisers are constant expressions, so these are constant-initialised
// before any static constructor runs - no static initialisation order hazard.
//
// One caveat: `wheel_distance_in` now comes out as 9.059999999999998721
// instead of the literal 9.06, because 9.06 in -> metres -> inches is not a
// bit-exact round trip. That is 1.4e-16 relative, on a number measured with a
// tape measure to about 1%.
//
// NOTE: `wheel_distance_in` is a CIRCUMFERENCE, not a distance and not a
// diameter. Every use site is `deg * wheel_distance_in / 360.0`.
double distance_between_wheels = mclib::config::robot_drive_geometry.track_width.in();
double wheel_distance_in = mclib::config::robot_drive_geometry.wheel.circumference().in();
double vertical_tracker_diameter = mclib::config::vertical_tracking_wheel.wheel.diameter().in();
double vertical_tracker_dist_from_center = mclib::config::vertical_tracking_wheel.offset.in();

double distance_kp = 0.4, distance_ki = 0, distance_kd = 3;
double turn_kp = 0.3, turn_ki = 0, turn_kd = 1.5;
double heading_correction_kp = 0.3, heading_correction_ki = 0, heading_correction_kd = 1.5;

bool heading_correction = true;
bool dir_change_start = true;
bool dir_change_end = true;
// Units: VOLTS. This is the default floor fed to the min_speed parameter of
// every motion function, and that value ends up in setVoltage(), so 10 means
// 10 V out of a 12 V rail - 83% of full power as a MINIMUM. See the audit note
// in include/mclib/config.hpp.
double min_output = 10;
double max_slew_accel_fwd = 1;
double max_slew_decel_fwd = 1;
double max_slew_accel_rev = 1;
double max_slew_decel_rev = 1;
double chase_power = 10;
