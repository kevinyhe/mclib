// mclib
#include "mclib/path/spline.hpp"

#include <cmath>
#include <cstddef>

namespace mclib {
namespace path {

namespace {

/// @brief Two positions closer than this (metres) count as the same waypoint.
constexpr double kDuplicateEpsilon = 1e-9;

/// @brief Floor on a knot spacing, so a near-duplicate cannot divide by zero.
constexpr double kMinKnotSpacing = 1e-9;

/// @brief Cubic Hermite position at @p u in [0, 1].
Vec2 hermite(const Vec2& p1, const Vec2& p2, const Vec2& m1, const Vec2& m2, double u) {
  const double u2 = u * u;
  const double u3 = u2 * u;
  return (2.0 * u3 - 3.0 * u2 + 1.0) * p1 + (u3 - 2.0 * u2 + u) * m1 +
         (-2.0 * u3 + 3.0 * u2) * p2 + (u3 - u2) * m2;
}

/// @brief First derivative of `hermite()` with respect to u.
Vec2 hermiteD1(const Vec2& p1, const Vec2& p2, const Vec2& m1, const Vec2& m2, double u) {
  const double u2 = u * u;
  return (6.0 * u2 - 6.0 * u) * p1 + (3.0 * u2 - 4.0 * u + 1.0) * m1 +
         (-6.0 * u2 + 6.0 * u) * p2 + (3.0 * u2 - 2.0 * u) * m2;
}

/// @brief Second derivative of `hermite()` with respect to u.
Vec2 hermiteD2(const Vec2& p1, const Vec2& p2, const Vec2& m1, const Vec2& m2, double u) {
  return (12.0 * u - 6.0) * p1 + (6.0 * u - 4.0) * m1 + (-12.0 * u + 6.0) * p2 +
         (6.0 * u - 2.0) * m2;
}

/**
 * @brief Build one baked sample from the Hermite derivatives at @p u.
 *
 * @details Heading is the compass bearing of the tangent, `atan2(x', y')`.
 * Curvature is `(y' x'' - x' y'') / |p'|^3`, which is the derivative of that
 * compass heading with respect to arc length - so it is positive when the
 * heading increases, i.e. when the path curves to the right. That is the same
 * sign convention as `mclib::arcRadius()`.
 */
PathPoint sampleAt(const Vec2& p1, const Vec2& p2, const Vec2& m1, const Vec2& m2, double u) {
  const Vec2 pos = hermite(p1, p2, m1, m2, u);
  const Vec2 d1 = hermiteD1(p1, p2, m1, m2, u);
  const Vec2 d2 = hermiteD2(p1, p2, m1, m2, u);

  PathPoint out;
  out.x = QLength::fromBase(pos.x());
  out.y = QLength::fromBase(pos.y());

  const double speed = d1.norm();
  if (speed > kMinKnotSpacing) {
    out.heading = QAngle::fromBase(std::atan2(d1.x(), d1.y()));
    const double cross = d1.y() * d2.x() - d1.x() * d2.y();
    out.curvature = QCurvature::fromBase(cross / (speed * speed * speed));
  }
  return out;
}

}  // namespace

Path generateSpline(const std::vector<Waypoint>& waypoints, const SplineConfig& config) {
  // Positions in metres (the Quantity base unit), so the geometry below is a
  // plain double problem and the units live only at the boundary.
  std::vector<Vec2> knots;
  knots.reserve(waypoints.size());
  for (const Waypoint& waypoint : waypoints) {
    const Vec2 candidate{waypoint.x.raw(), waypoint.y.raw()};
    if (!knots.empty() && (candidate - knots.back()).norm() < kDuplicateEpsilon) {
      continue;
    }
    knots.push_back(candidate);
  }

  if (knots.size() < 2) {
    std::vector<PathPoint> single;
    for (const Vec2& knot : knots) {
      PathPoint point;
      point.x = QLength::fromBase(knot.x());
      point.y = QLength::fromBase(knot.y());
      single.push_back(point);
    }
    return Path(std::move(single));
  }

  const std::size_t count = knots.size();
  const double alpha = config.centripetal ? 0.5 : 0.0;

  // Knot parameter spacing. alpha = 0.5 is the centripetal parameterisation,
  // alpha = 0 the uniform one.
  std::vector<double> spans(count - 1);
  for (std::size_t i = 0; i + 1 < count; ++i) {
    spans[i] = std::fmax(std::pow((knots[i + 1] - knots[i]).norm(), alpha), kMinKnotSpacing);
  }

  // Divided differences: the average slope across each knot interval, in the
  // knot parameterisation.
  std::vector<Vec2> slopes(count - 1);
  for (std::size_t i = 0; i + 1 < count; ++i) {
    slopes[i] = (knots[i + 1] - knots[i]) / spans[i];
  }

  // Tangent at every knot, in the knot parameterisation.
  //
  // Interior knots get the Bessel tangent - the two neighbouring slopes,
  // weighted by the *opposite* intervals. That is what plain Catmull-Rom
  // reduces to when the knots are evenly spaced, and it keeps the curve C1
  // because the two segments either side of a knot share the tangent.
  //
  // The ends are the part naive implementations get wrong. Reflecting a
  // phantom point through the first knot gives the one-sided slope, which is
  // exact for a straight line and badly wrong for anything curved: on a
  // circular arc it makes the first segment's curvature swing through zero and
  // overshoot to 2/R, which a curvature velocity limiter then believes. Using
  // the derivative of the quadratic through the first three knots instead
  // (`2 * slope - neighbouring tangent`) reproduces a circle to within a
  // fraction of a percent.
  std::vector<Vec2> tangents(count);
  for (std::size_t i = 1; i + 1 < count; ++i) {
    const double total = spans[i - 1] + spans[i];
    tangents[i] = (spans[i] * slopes[i - 1] + spans[i - 1] * slopes[i]) / total;
  }
  if (count == 2) {
    tangents[0] = slopes[0];
    tangents[1] = slopes[0];
  } else {
    tangents[0] = 2.0 * slopes[0] - tangents[1];
    tangents[count - 1] = 2.0 * slopes[count - 2] - tangents[count - 2];
  }

  const double tangent_scale = 1.0 - clamp(config.tension, 0.0, 1.0);
  const double spacing = std::fmax(config.spacing.raw(), kMinKnotSpacing);
  const int min_samples = config.min_samples_per_segment > 1 ? config.min_samples_per_segment : 2;

  std::vector<PathPoint> points;
  points.reserve(count * static_cast<std::size_t>(min_samples) + 1);

  for (std::size_t i = 0; i + 1 < count; ++i) {
    const Vec2& p1 = knots[i];
    const Vec2& p2 = knots[i + 1];
    // Hermite wants the tangent with respect to u in [0, 1], not to the knot
    // parameter, so scale by the interval.
    const Vec2 m1 = tangents[i] * spans[i] * tangent_scale;
    const Vec2 m2 = tangents[i + 1] * spans[i] * tangent_scale;

    // Coarse pass to estimate the segment's arc length, so the sample count
    // follows the requested spacing rather than the knot parameter.
    constexpr int kProbeSteps = 16;
    double estimated = 0.0;
    Vec2 previous = p1;
    for (int step = 1; step <= kProbeSteps; ++step) {
      const Vec2 current = hermite(p1, p2, m1, m2, static_cast<double>(step) / kProbeSteps);
      estimated += (current - previous).norm();
      previous = current;
    }

    int samples = static_cast<int>(std::ceil(estimated / spacing));
    if (samples < min_samples) {
      samples = min_samples;
    }

    // The knot at u = 0 was already emitted as the previous segment's u = 1.
    const int first = (i == 0) ? 0 : 1;
    for (int step = first; step <= samples; ++step) {
      points.push_back(sampleAt(p1, p2, m1, m2, static_cast<double>(step) / samples));
    }
  }

  return Path(std::move(points));
}

}  // namespace path
}  // namespace mclib
