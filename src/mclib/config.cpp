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

// The four geometry globals that used to live here are gone:
//
//   distance_between_wheels            -> config::robot_drive_geometry.track_width
//   wheel_distance_in (a CIRCUMFERENCE) -> config::robot_drive_geometry.wheel
//   vertical_tracker_diameter          -> config::vertical_tracking_wheel.wheel
//   vertical_tracker_dist_from_center  -> config::vertical_tracking_wheel.offset
//
// They were briefly kept as values derived from those constants, which fixed
// the disagreement but kept the confusion: four bare `double`s in three
// conventions, one of them (`wheel_distance_in`) named for a convention it did
// not use, and nothing in the type system to stop a diameter being read as a
// circumference. `units::Wheel` is what stops that, and it cannot be spelled
// as a `double`.
//
// Assigning to them at runtime did work - motion.cpp and odometry_task.cpp
// read them at call time - and that was the only way for a team using mclib as
// a PROS template to override the geometry, since those files ship precompiled
// in `mclib.a`. That override is preserved:
// `mclib::config::robot_drive_geometry` is mutable and read at the same
// points. See mclib/robot_geometry.hpp.
//
// Deleting them makes every reader name the geometry it wants and turns any
// stale use into a compile error at the one place that has to change.
//
// Note also that `wheel_distance_in` came out as 9.059999999999998721 rather
// than the literal 9.06 once it was derived, because 9.06 in -> metres ->
// inches is not a bit-exact round trip. Nothing carries that wart now.

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
