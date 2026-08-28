// mclib
#include "mclib/utils.hpp"

#include <cmath>
#include <limits>

namespace
{
/**
 * @brief Smallest `|sin(90 - angle)|` treated as a real heading term.
 *
 * `sin` is bounded by 1, so an absolute floor on it is already a relative
 * one. Near a multiple of pi the value is dominated by the rounding of the
 * argument: `degToRad(180.0)` is off by ~2.4e-16 and `|sin'| == 1` there, so
 * `sin(degToRad(90 - (-90)))` evaluates to 1.2246e-16 instead of 0. 1e-9 sits
 * seven orders above that noise and still only spans 5.7e-8 degrees either
 * side of +/-90 -- six orders finer than a V5 IMU resolves -- so no heading a
 * caller can actually hold gets swallowed.
 */
constexpr double kSinFloor = 1e-9;

/**
 * @brief Largest `|denominator| / chord^2` treated as degenerate.
 *
 * The heading floor alone is not enough: the denominator is
 * `2 * delta_y * sin(...)`, so a tiny `delta_y` shrinks it independently of
 * the heading. Testing the denominator against the chord length squared --
 * the numerator -- makes this a relative test on the quotient rather than an
 * absolute one on a quantity whose scale depends on the caller's units: the
 * branch fires exactly when `|result|` would exceed `1 / kRadiusFloor`, i.e.
 * 1e12 inches. A VEX field is 144 inches across; anything past 1e12 is a
 * straight line by any measure a caller cares about. It also subsumes
 * `delta_y == 0` (0 <= 0), including the `0 / 0` case where the target is the
 * current position.
 */
constexpr double kRadiusFloor = 1e-12;
}  // namespace

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
    const double sin_term = std::sin(degToRad(90.0 - angle));
    const double denominator = 2.0 * delta_y * sin_term;
    const double chord_sq = delta_x * delta_x + delta_y * delta_y;

    // Degenerate: the heading is (numerically) +/-90 deg, or the denominator
    // has collapsed relative to the numerator. Exact `denominator == 0.0` used
    // to be the test here, and it split two geometrically identical cases:
    // angle == 90 gives sin(0) == 0 exactly and returned infinity, while
    // angle == -90 gives sin(pi) == 1.22e-16 and returned 4.08e15 -- a finite
    // number that flowed downstream as if it were a real radius.
    //
    // No finite value is meaningful here, and the old magic 999 silently
    // became a finite speed limit downstream. See the header warning -- this
    // formula is frame-buggy; mclib::arcRadius() is the correct one.
    if (std::fabs(sin_term) <= kSinFloor ||
        std::fabs(denominator) <= kRadiusFloor * chord_sq)
    {
        return std::numeric_limits<double>::infinity();
    }
    return chord_sq / denominator;
}
