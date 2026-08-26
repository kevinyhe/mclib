// mclib
#pragma once

void turnToAngle(double turn_angle,
                 double time_limit_msec,
                 bool exit = true,
                 double max_output = 12.0,
                 double min_speed = -1.0);
void driveTo(double distance_in,
             double time_limit_msec,
             bool exit = true,
             double max_output = 12.0,
             double min_speed = -1.0);
void curveCircle(double result_angle_deg,
                 double center_radius,
                 double time_limit_msec,
                 bool exit = true,
                 double max_output = 12.0,
                 double min_speed = -1.0,
                 bool reverse = false);
void curveCircleReverse(double result_angle_deg,
                        double center_radius,
                        double time_limit_msec,
                        bool exit = true,
                        double max_output = 12.0,
                        double min_speed = -1.0);
void swing(double swing_angle,
           double drive_direction,
           double time_limit_msec,
           bool exit = true,
           double max_output = 12.0,
           double min_speed = -1.0);
void correctHeading();
void wallReset(double reset_x,
               double reset_y,
               double reset_heading,
               double drive_power,
               double time_limit_msec,
               double current_threshold = 2500.0,
               double velocity_threshold = 5.0);
void turnToPoint(double x,
                 double y,
                 int direction = 1,
                 double time_limit_msec = 1000.0,
                 double min_speed = -1.0);
void moveToPoint(double x,
                 double y,
                 int dir,
                 double time_limit_msec,
                 bool exit = true,
                 double max_output = 12.0,
                 bool overturn = true,
                 double min_speed = -1.0);
void boomerang(double x,
               double y,
               int dir,
               double a,
               double dlead,
               double time_limit_msec,
               bool exit = true,
               double max_output = 12.0,
               bool overturn = true,
               double min_speed = -1.0);
