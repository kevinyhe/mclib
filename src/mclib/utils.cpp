// mclib
#include "mclib/utils.hpp"

#include <cmath>
#include <limits>

double degToRad(double deg)
{
    return deg * M_PI / 180.0;
}

double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

double getRadius(double x, double y, double x1, double y1, double angle)
{
    const double delta_x = x1 - x;
    const double delta_y = y1 - y;
    const double denominator = 2.0 * delta_y * std::sin(degToRad(90.0 - angle));
    if (denominator == 0.0)
    {
        // Degenerate: delta_y == 0, or the heading is exactly +/-90 deg.
        // No finite value is meaningful here, and the old magic 999 silently
        // became a finite speed limit downstream. See the header warning --
        // this formula is frame-buggy; mclib::arcRadius() is the correct one.
        return std::numeric_limits<double>::infinity();
    }
    return (delta_x * delta_x + delta_y * delta_y) / denominator;
}
