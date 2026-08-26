// mclib
#include "mclib/utils.hpp"

#include <cmath>

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
    double delta_x = x1 - x;
    double delta_y = y1 - y;
    double denominator = 2 * delta_y * sin(degToRad(90 - angle));
    if (denominator == 0)
    {
        return 999;
    }
    return (delta_x * delta_x + delta_y * delta_y) / denominator;
}
