// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/chassis/chassis_math.hpp"
#include "mclib/math.hpp"
#include "mclib/path/spline.hpp"
#include "mclib/units/units.hpp"
#include "mclib/utils.hpp"

#include "test_assert.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

// Randomized checks of the pure math that is easy to get wrong at edge values:
// angle wrapping, arcRadius, spline interpolation and unit conversions.
//
// The seed is fixed and printed, so a failure reproduces exactly. Each case
// counts as one check in the summary. A property prints its first few failing
// inputs and then stays quiet, so one broken property cannot flood the log.

using mclib::arcRadius;
using mclib::kPi;
using mclib::kTwoPi;
using mclib::Pose2D;
using mclib::Vec2;
using mclib::wrapAngle;
using mclib::chassis_math::normalizeHeadingTarget;
using mclib::path::generateSpline;
using mclib::path::Path;
using mclib::path::SplineConfig;
using mclib::path::Waypoint;

namespace units = mclib::units;

namespace {

constexpr std::uint32_t kSeed = 0x5eed2026u;
constexpr int kCases = 10000;
constexpr int kMaxPrinted = 3;

std::mt19937 rng(kSeed);

double uniform(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}

int uniformInt(int lo, int hi) {
  return std::uniform_int_distribution<int>(lo, hi)(rng);
}

/// @brief One property: counts cases and failures, prints the first few.
class Property {
 public:
  explicit Property(const char* name) : m_name(name) {}

  /// @brief Record one case. @p fmt describes the inputs when it fails.
  __attribute__((format(printf, 3, 4))) void expect(bool ok, const char* fmt, ...) {
    ++m_cases;
    ++mclib::test::checkCount();
    if (ok) {
      return;
    }
    ++m_failures;
    ++mclib::test::failureCount();
    if (m_failures > kMaxPrinted) {
      return;
    }
    std::printf("  FAIL %s: ", m_name);
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
    std::printf("\n");
  }

  ~Property() {
    std::printf("  %-44s %6d cases, %d failed\n", m_name, m_cases, m_failures);
  }

 private:
  const char* m_name;
  int m_cases = 0;
  int m_failures = 0;
};

/// @brief True when @p value is within @p tol of a whole number.
bool nearInteger(double value, double tol) {
  return std::fabs(value - std::round(value)) <= tol;
}

/// @brief True when @p a and @p b agree to @p rel of the larger magnitude.
bool relNear(double a, double b, double rel) {
  const double scale = std::fmax(std::fabs(a), std::fabs(b));
  return std::fabs(a - b) <= rel * scale;
}

/// @brief An angle in radians that exercises the wrap edges.
///
/// A third of the cases sit within 1e-9 rad of an odd multiple of pi (the
/// +/-180 deg seam), a third are large multiples of a full turn plus a small
/// offset, and the rest are spread over +/-10 turns.
double edgeAngleRad() {
  switch (uniformInt(0, 2)) {
    case 0: {
      const int k = uniformInt(-50, 50);
      return (2 * k + 1) * kPi + uniform(-1e-9, 1e-9);
    }
    case 1: {
      const int k = uniformInt(-100000, 100000);
      return k * kTwoPi + uniform(-kPi, kPi);
    }
    default:
      return uniform(-10.0 * kTwoPi, 10.0 * kTwoPi);
  }
}

// ---------------------------------------------------------------------------
// Angle wrapping
// ---------------------------------------------------------------------------

void testWrapAngle() {
  Property range("wrapAngle range [-pi, pi]");
  Property turns("wrapAngle differs by whole turns");
  for (int i = 0; i < kCases; ++i) {
    const double in = edgeAngleRad();
    const double out = wrapAngle(in);
    range.expect(out >= -kPi && out <= kPi, "case %d: wrapAngle(%.17g) = %.17g", i, in, out);
    // fmod is exact, so the only error is one rounding of the subtraction:
    // a few ulps of |in| measured in turns.
    const double tol = 1e-15 * (1.0 + std::fabs(in)) / kTwoPi + 1e-12;
    turns.expect(nearInteger((in - out) / kTwoPi, tol),
                 "case %d: wrapAngle(%.17g) = %.17g, (in - out) / 2pi = %.17g", i, in, out,
                 (in - out) / kTwoPi);
  }
}

void testUnitsWrap() {
  Property range("units::wrap range [-pi, pi]");
  Property turns("units::wrap differs by whole turns");
  Property same("units::wrap matches wrapAngle");
  for (int i = 0; i < kCases; ++i) {
    const double in = edgeAngleRad();
    const double out = units::wrap(in * units::radian).rad();
    range.expect(out >= -units::pi && out <= units::pi, "case %d: wrap(%.17g rad) = %.17g", i, in,
                 out);
    const double tol = 1e-15 * (1.0 + std::fabs(in)) / kTwoPi + 1e-12;
    turns.expect(nearInteger((in - out) / kTwoPi, tol), "case %d: wrap(%.17g rad) = %.17g", i, in,
                 out);
    // units.hpp promises the same algorithm and the same closed range.
    same.expect(out == wrapAngle(in), "case %d: wrap(%.17g) = %.17g, wrapAngle = %.17g", i, in,
                out, wrapAngle(in));
  }
}

void testNormalizeHeadingTarget() {
  Property range("normalizeHeadingTarget within 180 deg");
  Property turns("normalizeHeadingTarget moves by 360s");
  Property stays("normalizeHeadingTarget keeps near targets");
  for (int i = 0; i < kCases; ++i) {
    const double current = uniform(-7200.0, 7200.0);
    double target = 0.0;
    switch (uniformInt(0, 2)) {
      case 0:
        // Just either side of the 180 deg seam, some whole turns away.
        target = current + 180.0 * (uniformInt(0, 1) ? 1 : -1) + 360.0 * uniformInt(-20, 20) +
                 uniform(-1e-9, 1e-9);
        break;
      case 1:
        target = current + 360.0 * uniformInt(-100000, 100000) + uniform(-180.0, 180.0);
        break;
      default:
        target = uniform(-7200.0, 7200.0);
        break;
    }
    const double out = normalizeHeadingTarget(target, current);
    // The result is computed as target - 360k and then compared against
    // current, so allow the rounding of those two subtractions.
    const double slack = 1e-15 * (std::fabs(target) + std::fabs(current) + 360.0);
    range.expect(std::fabs(out - current) <= 180.0 + slack,
                 "case %d: target %.17g, current %.17g -> %.17g (off by %.17g)", i, target,
                 current, out, out - current);
    turns.expect(nearInteger((target - out) / 360.0, slack),
                 "case %d: target %.17g, current %.17g -> %.17g", i, target, current, out);
    if (std::fabs(target - current) <= 180.0) {
      stays.expect(out == target, "case %d: target %.17g, current %.17g -> %.17g", i, target,
                   current, out);
    }
  }
}

// ---------------------------------------------------------------------------
// arcRadius
// ---------------------------------------------------------------------------

void testArcRadius() {
  Property notNan("arcRadius never NaN");
  Property circle("arcRadius circle passes through target");
  Property sign("arcRadius sign matches side");
  Property ahead("arcRadius dead ahead/behind is +inf");
  Property tiny("arcRadius tiny chord is not NaN");
  Property self("arcRadius target == from is +inf");

  for (int i = 0; i < kCases; ++i) {
    const Pose2D from{uniform(-72.0, 72.0), uniform(-72.0, 72.0), edgeAngleRad()};

    // General target anywhere on the field. The circle tangent to the
    // heading has its centre on the robot's right axis, |r| away. The target
    // must lie on it.
    {
      const Vec2 target{uniform(-72.0, 72.0), uniform(-72.0, 72.0)};
      const double r = arcRadius(from, target);
      notNan.expect(!std::isnan(r), "case %d: pose (%.17g, %.17g, %.17g) target (%.17g, %.17g)",
                    i, from.x, from.y, from.theta, target.x(), target.y());
      const Vec2 local = mclib::fieldPointToRobot(target, from);
      if (std::isfinite(r)) {
        const Vec2 centre = mclib::robotPointToField(Vec2{r, 0.0}, from);
        const double miss = std::fabs((target - centre).norm() - std::fabs(r));
        circle.expect(miss <= 1e-9 * (1.0 + std::fabs(r)),
                      "case %d: r = %.17g, |target - centre| - |r| = %.17g", i, r, miss);
        sign.expect((r > 0.0) == (local.x() > 0.0), "case %d: r = %.17g, lateral = %.17g", i, r,
                    local.x());
      }
    }

    // Dead ahead or dead behind, at least an inch away. Straight line, so the
    // doc promises +infinity whatever the heading.
    //
    // The inch floor matters. arcRadius returns +infinity when
    // |lateral| <= 1e-12 * chord^2. Rounding in `target - from` leaves about
    // 1e-14 in of lateral noise at field coordinates, and that beats the
    // threshold once the chord is under ~0.1 in: at 0.01 in, 94% of dead-ahead
    // targets get a finite radius of 6e9 in or more, either sign. That is a
    // curvature under 2e-10 per inch, so no controller sees it. The tiny-chord
    // case below checks only that such a result is not NaN.
    {
      const double d = uniform(1.0, 200.0) * (uniformInt(0, 1) ? 1.0 : -1.0);
      const Vec2 target = from.translation() + d * mclib::headingVector(from.theta);
      const double r = arcRadius(from, target);
      ahead.expect(std::isinf(r) && r > 0.0,
                   "case %d: pose (%.17g, %.17g, %.17g), d = %.17g -> r = %.17g", i, from.x,
                   from.y, from.theta, d, r);
    }

    // Target a hair away from the robot in any direction, down to 1e-12 in.
    // The radius may be finite or infinite, but never NaN, and a finite one
    // is at least half the chord (the chord is at most a diameter).
    {
      const double chord = std::pow(10.0, uniform(-12.0, -3.0));
      const double dir = uniform(-kPi, kPi);
      const Vec2 target = from.translation() + chord * mclib::headingVector(dir);
      const double r = arcRadius(from, target);
      const double actual = (target - from.translation()).norm();
      tiny.expect(!std::isnan(r) && std::fabs(r) >= 0.5 * actual * (1.0 - 1e-9),
                  "case %d: chord %.17g, r = %.17g", i, actual, r);
    }

    {
      const double r = arcRadius(from, from.translation());
      self.expect(std::isinf(r) && r > 0.0, "case %d: r = %.17g", i, r);
    }
  }
}

// ---------------------------------------------------------------------------
// Splines
// ---------------------------------------------------------------------------

void testSplines() {
  Property through("spline passes through every waypoint");
  Property nonNeg("spline arc length never negative");
  Property grows("spline arc length grows along curve");
  Property chords("spline arc length >= waypoint chords");
  Property ends("spline starts and ends on waypoints");

  for (int i = 0; i < kCases; ++i) {
    // 2 to 8 waypoints on a 144 in field, at least 2 in apart so the path's
    // 1e-9 in duplicate collapse never merges a sample into a knot.
    const int n = uniformInt(2, 8);
    std::vector<Waypoint> waypoints;
    while (static_cast<int>(waypoints.size()) < n) {
      const Waypoint candidate{uniform(-72.0, 72.0) * units::inch,
                               uniform(-72.0, 72.0) * units::inch};
      if (!waypoints.empty() && (candidate.point() - waypoints.back().point()).norm() < 2.0) {
        continue;
      }
      waypoints.push_back(candidate);
    }

    SplineConfig config;
    config.spacing = uniform(0.25, 4.0) * units::inch;
    config.tension = uniformInt(0, 3) == 0 ? uniform(0.0, 1.0) : 0.0;
    config.centripetal = uniformInt(0, 4) != 0;

    const Path path = generateSpline(waypoints, config);
    const auto& points = path.points();

    // Every waypoint is a sample (u = 0 or u = 1 of a Hermite segment). The
    // search for waypoint w starts after waypoint w - 1's sample, so passing
    // also means the waypoints appear in order.
    std::size_t search_from = 0;
    std::vector<double> knot_distance;
    for (int w = 0; w < n; ++w) {
      const Vec2 want = waypoints[w].point();
      std::size_t best = points.size();
      double best_err = 1e300;
      for (std::size_t k = search_from; k < points.size(); ++k) {
        const double err = (points[k].point() - want).norm();
        if (err < best_err) {
          best_err = err;
          best = k;
        }
        if (err <= 1e-9) {
          break;
        }
      }
      through.expect(best_err <= 1e-9, "case %d: waypoint %d (%.17g, %.17g) missed by %.17g in",
                     i, w, want.x(), want.y(), best_err);
      if (best < points.size()) {
        search_from = best + 1;
        knot_distance.push_back(points[best].distance.in());
      }
    }

    ends.expect(!points.empty() && (points.front().point() - waypoints.front().point()).norm() <=
                                       1e-9 &&
                    (points.back().point() - waypoints.back().point()).norm() <= 1e-9,
                "case %d: path does not start and end on the first and last waypoint", i);

    bool all_non_negative = true;
    bool all_growing = true;
    for (std::size_t k = 0; k < points.size(); ++k) {
      if (!(points[k].distance.in() >= 0.0)) {
        all_non_negative = false;
      }
      if (k > 0 && !(points[k].distance > points[k - 1].distance)) {
        all_growing = false;
      }
    }
    nonNeg.expect(all_non_negative, "case %d: a sample has negative or NaN distance", i);
    grows.expect(all_growing, "case %d: distance does not strictly increase", i);

    // Between two knots the curve is at least as long as the straight chord.
    bool chords_ok = knot_distance.size() == static_cast<std::size_t>(n);
    for (std::size_t w = 1; chords_ok && w < knot_distance.size(); ++w) {
      const double chord = (waypoints[w].point() - waypoints[w - 1].point()).norm();
      if (knot_distance[w] - knot_distance[w - 1] < chord * (1.0 - 1e-12)) {
        chords_ok = false;
      }
    }
    chords.expect(chords_ok, "case %d: arc between waypoints shorter than the chord", i);
  }
}

// ---------------------------------------------------------------------------
// Unit conversions
// ---------------------------------------------------------------------------

constexpr double kRel = 1e-14;

/// @brief Magnitudes from 1e-6 to 1e6, both signs, so relative error is tested
/// across the range a robot actually uses and beyond.
double spreadValue() {
  const double magnitude = std::pow(10.0, uniform(-6.0, 6.0));
  return uniformInt(0, 1) ? magnitude : -magnitude;
}

void testUnitRoundTrips() {
  Property length("length round-trips (m, cm, mm, in, ft)");
  Property time("time round-trips (s, ms, min)");
  Property angle("angle round-trips (rad, deg, rot)");
  Property electric("voltage and current round-trips");
  Property speed("speed round-trips (mps, inps, radps, degps, rpm)");
  Property cross("length unit to unit (in -> cm -> ft -> in)");
  Property bridge("degToRad / radToDeg round-trip");
  Property agree("degToRad matches units::degree");

  using namespace units;
  for (int i = 0; i < kCases; ++i) {
    const double v = spreadValue();

    length.expect(relNear((v * metre).m(), v, kRel) && relNear((v * centimetre).cm(), v, kRel) &&
                      relNear((v * millimetre).mm(), v, kRel) &&
                      relNear((v * inch).in(), v, kRel) && relNear((v * foot).ft(), v, kRel) &&
                      relNear(inches(v * inch), v, kRel) && relNear(feet(v * foot), v, kRel),
                  "case %d: v = %.17g", i, v);

    time.expect(relNear((v * second).s(), v, kRel) && relNear((v * millisecond).ms(), v, kRel) &&
                    relNear(milliseconds(v * millisecond), v, kRel) &&
                    relNear((v * minute).s(), v * 60.0, kRel),
                "case %d: v = %.17g", i, v);

    angle.expect(relNear((v * radian).rad(), v, kRel) && relNear((v * degree).deg(), v, kRel) &&
                     relNear(degrees(v * degree), v, kRel) &&
                     relNear((v * rotation).deg(), v * 360.0, kRel),
                 "case %d: v = %.17g", i, v);

    electric.expect(relNear((v * volt).volts(), v, kRel) &&
                        relNear((v * millivolt).mV(), v, kRel) &&
                        relNear((v * ampere).amps(), v, kRel) &&
                        relNear((v * milliampere).mA(), v, kRel),
                    "case %d: v = %.17g", i, v);

    speed.expect(relNear((v * units::mps).mps(), v, kRel) &&
                     relNear((v * units::inps).inps(), v, kRel) &&
                     relNear((v * units::radps).radps(), v, kRel) &&
                     relNear((v * units::degps).degps(), v, kRel) &&
                     relNear((v * units::rpm).rpm(), v, kRel),
                 "case %d: v = %.17g", i, v);

    {
      const double cm = (v * inch).cm();
      const double ft = (cm * centimetre).ft();
      const double back = (ft * foot).in();
      cross.expect(relNear(back, v, kRel), "case %d: %.17g in -> %.17g", i, v, back);
    }

    bridge.expect(relNear(radToDeg(degToRad(v)), v, kRel) &&
                      relNear(degToRad(radToDeg(v)), v, kRel),
                  "case %d: v = %.17g", i, v);

    agree.expect(relNear(degToRad(v), (v * degree).rad(), kRel), "case %d: v = %.17g", i, v);
  }
}

}  // namespace

int main() {
  std::printf("randomized_math_test seed 0x%08x\n", static_cast<unsigned>(kSeed));
  testWrapAngle();
  testUnitsWrap();
  testNormalizeHeadingTarget();
  testArcRadius();
  testSplines();
  testUnitRoundTrips();
  return mclib::test::summary("randomized_math_test");
}
