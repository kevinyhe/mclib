// mclib
#include "mclib/control/odometry.hpp"

#include "api.h"
#include "mclib/config.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/state.hpp"
#include "mclib/utils.hpp"

#include <cmath>
void trackNoOdomWheel()
{
  resetChassis();
  double prev_heading_rad = 0;
  double prev_left_deg = 0, prev_right_deg = 0;
  double delta_local_y_in = 0;

  while (true)
  {
    double heading_rad = degToRad(getInertialHeading());
    double left_deg = getLeftRotationDegree();
    double right_deg = getRightRotationDegree();
    double delta_heading_rad = heading_rad - prev_heading_rad;                        // Change in heading (radians)
    double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;    // Left wheel delta (inches)
    double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // Right wheel delta (inches)
    // If no heading change, treat as straight movement
    if (fabs(delta_heading_rad) < 1e-6)
    {
      delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
    }
    else
    {
      // Calculate arc movement for each wheel
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
      double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
      delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
    }
    // Update global position using polar coordinates
    double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
    double polar_radius_in = delta_local_y_in;

    xpos += polar_radius_in * sin(polar_angle_rad);
    ypos += polar_radius_in * cos(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_left_deg = left_deg;
    prev_right_deg = right_deg;

    pros::delay(10);
  }
}

void trackYOdomWheel()
{
  resetChassis();
  double prev_heading_rad = 0;
  double prev_vertical_pos_deg = 0;
  double delta_local_y_in = 0;

  while (true)
  {
    double heading_rad = degToRad(getInertialHeading());
    double vertical_pos_deg = vertical_tracker.getPositionDeg(); // convert centidegrees to degrees
    double delta_heading_rad = heading_rad - prev_heading_rad;
    double delta_vertical_in = (vertical_pos_deg - prev_vertical_pos_deg) * vertical_tracker_diameter * M_PI / 360.0; // vertical tracker delta (inches)

    // Calculate local movement based on heading change
    if (fabs(delta_heading_rad) < 1e-6)
    {
      delta_local_y_in = delta_vertical_in;
    }
    else
    {
      double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
      delta_local_y_in = sin_multiplier * ((delta_vertical_in / delta_heading_rad) + vertical_tracker_dist_from_center);
    }

    double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
    double polar_radius_in = delta_local_y_in;

    // Heading zero aligns with +Y, so X updates use sin and Y uses cos.
    xpos += polar_radius_in * sin(polar_angle_rad);
    ypos += polar_radius_in * cos(polar_angle_rad);

    prev_heading_rad = heading_rad;
    prev_vertical_pos_deg = vertical_pos_deg;

    pros::delay(10);
  }
}
