// mclib
//
// Host tests for the units library. Standalone: compile and run with
//
//   g++ -std=gnu++20 -Iinclude -o /tmp/units_test tests/units_test.cpp && /tmp/units_test
//
// Returns 0 on success, 1 on the first runtime failure. Everything that can be
// checked at compile time is a static_assert instead, so most of this file
// costs nothing to run.

#include "mclib/units/units.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <type_traits>

using namespace mclib::units;

// ---------------------------------------------------------------------------
// Compile-time checks: dimension composition
// ---------------------------------------------------------------------------

static_assert(std::is_same_v<decltype(QLength{1.0} / QTime{1.0}), QVelocity>,
              "length over time must be a velocity");
static_assert(std::is_same_v<decltype(QVelocity{1.0} / QTime{1.0}), QAcceleration>,
              "velocity over time must be an acceleration");
static_assert(std::is_same_v<decltype(QAcceleration{1.0} / QTime{1.0}), QJerk>,
              "acceleration over time must be a jerk");
static_assert(std::is_same_v<decltype(QAngle{1.0} / QTime{1.0}), QAngularVelocity>,
              "angle over time must be an angular velocity");
static_assert(std::is_same_v<decltype(QVelocity{1.0} * QTime{1.0}), QLength>,
              "velocity times time must be a length");
static_assert(std::is_same_v<decltype(QLength{1.0} * QLength{1.0}), QArea>, "length squared is an area");
static_assert(std::is_same_v<decltype(1.0 / QLength{1.0}), QCurvature>, "one over a length is a curvature");
static_assert(std::is_same_v<decltype(1.0 / QTime{1.0}), QFrequency>, "one over a time is a frequency");
static_assert(std::is_same_v<decltype(QLength{1.0} / QLength{1.0}), QNumber>,
              "a ratio of lengths is dimensionless");
static_assert(std::is_same_v<decltype(square(QLength{1.0})), QArea>, "square doubles the exponents");
static_assert(std::is_same_v<decltype(QVoltage{1.0} / QCurrent{1.0}), Quantity<0, 0, 0, 1, -1>>,
              "volts over amps is a resistance");

// Same dimension in, same dimension out.
static_assert(std::is_same_v<decltype(QLength{1.0} + QLength{2.0}), QLength>);
static_assert(std::is_same_v<decltype(-QLength{1.0}), QLength>);
static_assert(std::is_same_v<decltype(QLength{1.0} * 2.0), QLength>);
static_assert(std::is_same_v<decltype(abs(QTime{-1.0})), QTime>);
static_assert(std::is_same_v<decltype(atan2(1_in, 1_in)), QAngle>);
static_assert(std::is_same_v<decltype(arcLength(1_in, 1_rad)), QLength>);
static_assert(std::is_same_v<decltype(rimVelocity(1_in, 1_rpm)), QVelocity>);

// A Quantity must cost nothing over the double it wraps.
static_assert(sizeof(QLength) == sizeof(double), "Quantity must be a bare double in memory");
static_assert(std::is_trivially_copyable_v<QLength>, "Quantity must pass in a register");
static_assert(std::is_standard_layout_v<QLength>);

// ---------------------------------------------------------------------------
// Compile-time checks: dimensionally wrong code does NOT compile
//
// Each concept below is a well-formedness probe; asserting its negation proves
// the bad expression is rejected. They have to be concepts rather than bare
// `requires{...}` expressions because a requires-expression over non-dependent
// types is checked eagerly by GCC and would be a hard error, not a false.
// If any of these ever start compiling, the type system has stopped working.
// ---------------------------------------------------------------------------

template <typename A, typename B>
concept CanAdd = requires(A a, B b) { a + b; };
template <typename A, typename B>
concept CanSubtract = requires(A a, B b) { a - b; };
template <typename A, typename B>
concept CanOrder = requires(A a, B b) { a < b; };
template <typename A, typename B>
concept CanCompareEqual = requires(A a, B b) { a == b; };
template <typename A>
concept HasInches = requires(A a) { a.in(); };
template <typename A>
concept HasMillis = requires(A a) { a.ms(); };
template <typename A>
concept HasVolts = requires(A a) { a.volts(); };

static_assert(!CanAdd<QLength, QTime>, "adding a length to a time must not compile");
static_assert(!CanSubtract<QLength, QAngle>, "subtracting an angle from a length must not compile");
static_assert(!CanOrder<QLength, QTime>, "comparing a length to a time must not compile");
static_assert(!CanCompareEqual<QArea, QLength>, "an area does not compare equal to a length");
static_assert(CanAdd<QLength, QLength>, "adding two lengths is fine");
static_assert(CanOrder<QTime, QTime>, "ordering two times is fine");

static_assert(!std::is_convertible_v<QTime, QLength>, "a time must not convert to a length");
static_assert(!std::is_convertible_v<decltype(QLength{1.0} * QTime{1.0}), QVelocity>,
              "length times time is not a velocity, and must not convert to one");
static_assert(!std::is_convertible_v<double, QLength>, "a raw double must not silently become a length");
static_assert(!std::is_convertible_v<QLength, double>, "a length must not silently become a raw double");
static_assert(std::is_constructible_v<QLength, double>, "but the explicit constructor is still there");

static_assert(!HasInches<QTime>, "a time has no length accessor");
static_assert(!HasMillis<QLength>, "a length has no time accessor");
static_assert(!HasVolts<QAngle>, "an angle has no voltage accessor");
static_assert(HasInches<QLength> && HasMillis<QTime> && HasVolts<QVoltage>,
              "each dimension does have its own accessors");

// The one deliberate hole: dimensionless quantities convert to and from double,
// because ratios and gains have to interoperate with plain arithmetic.
static_assert(std::is_convertible_v<double, QNumber>, "dimensionless quantities take a raw double");
static_assert(std::is_convertible_v<QNumber, double>, "dimensionless quantities give back a raw double");

// ---------------------------------------------------------------------------
// Compile-time checks: literal and constant values
// ---------------------------------------------------------------------------

static_assert(24_in == inch * 24.0);
static_assert((24_in).mm() > 609.59 && (24_in).mm() < 609.61, "24 in is 609.6 mm");
static_assert((1_ft).in() > 11.999 && (1_ft).in() < 12.001, "a foot is twelve inches");
static_assert((1_tile).in() > 23.999 && (1_tile).in() < 24.001, "a VEX tile is 24 in");
static_assert((90_deg).rad() > 1.5707 && (90_deg).rad() < 1.5709, "90 deg is pi/2 rad");
static_assert((180_deg).rad() > 3.1415 && (180_deg).rad() < 3.1416, "180 deg is pi rad");
static_assert((1_rot).deg() > 359.99 && (1_rot).deg() < 360.01, "a rotation is 360 deg");
static_assert((1.5_s).ms() > 1499.9 && (1.5_s).ms() < 1500.1, "1.5 s is 1500 ms");
static_assert((250_ms).s() > 0.2499 && (250_ms).s() < 0.2501, "250 ms is a quarter second");
static_assert((12_V).mV() > 11999.9 && (12_V).mV() < 12000.1, "12 V is 12000 mV");
static_assert((2500_mA).amps() > 2.4999 && (2500_mA).amps() < 2.5001, "2500 mA is 2.5 A");
static_assert((600_rpm).radps() > 62.83 && (600_rpm).radps() < 62.84, "600 rpm is 20*pi rad/s");
static_assert((1_rpm).rpm() > 0.9999 && (1_rpm).rpm() < 1.0001, "rpm round-trips");
static_assert((1_mm).mm() > 0.9999 && (1_mm).mm() < 1.0001, "mm round-trips");
static_assert((1_rad).rad() == 1.0, "radian is the stored angle unit");

// ---------------------------------------------------------------------------
// Compile-time checks: the old QTime semantics still hold
//
// Before this file had types, QTime was a double with millisecond == 1.0. Every
// existing call site compares QTime against QTime, so what has to survive is
// the *relative* arithmetic, not the stored number.
// ---------------------------------------------------------------------------

static_assert(second == 1000.0 * millisecond, "a second is still a thousand milliseconds");
static_assert(250 * millisecond < second, "250 ms is still less than a second");
static_assert(100 * millisecond + 150 * millisecond == 250 * millisecond);
static_assert((1000u * millisecond).ms() > 999.9 && (1000u * millisecond).ms() < 1000.1,
              "pros::millis() * millisecond must scale as milliseconds");
static_assert(2.0 * second - 500 * millisecond == 1500 * millisecond);
static_assert(max(QTime{0.0}, 250 * millisecond - 300 * millisecond) == QTime{0.0},
              "the pto_mechanism remaining-settle-time clamp");
static_assert(std::is_same_v<decltype(250 * millisecond), QTime>);

// ---------------------------------------------------------------------------
// Runtime checks
// ---------------------------------------------------------------------------

static int failures = 0;

static void check(bool condition, const char* what) {
  if (!condition) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

static void checkNear(double actual, double expected, double tolerance, const char* what) {
  if (!(std::fabs(actual - expected) <= tolerance)) {
    std::printf("FAIL: %s (got %.10g, expected %.10g +/- %.10g)\n", what, actual, expected, tolerance);
    ++failures;
  }
}

int main() {
  // Literals convert to the units a VEX programmer actually types.
  checkNear((24_in).mm(), 609.6, 1e-9, "24_in in millimetres");
  checkNear((24_in).in(), 24.0, 1e-12, "24_in round-trips through inches");
  checkNear((90_deg).rad(), M_PI / 2.0, 1e-12, "90_deg in radians");
  checkNear((90_deg).deg(), 90.0, 1e-12, "90_deg round-trips through degrees");
  checkNear((1.5_s).ms(), 1500.0, 1e-9, "1.5_s in milliseconds");
  checkNear((12_V).volts(), 12.0, 1e-12, "12_V in volts");
  checkNear((12_V).mV(), 12000.0, 1e-9, "12_V in millivolts");
  checkNear((2500_mA).mA(), 2500.0, 1e-9, "2500_mA round-trips");
  checkNear((600_rpm).rpm(), 600.0, 1e-9, "600_rpm round-trips");

  // Free-function accessors agree with the member ones.
  checkNear(inches(24_in), (24_in).in(), 0.0, "inches() matches .in()");
  checkNear(milliseconds(250_ms), (250_ms).ms(), 0.0, "milliseconds() matches .ms()");
  checkNear(degrees(90_deg), (90_deg).deg(), 0.0, "degrees() matches .deg()");

  // Dimension composition produces the right numbers, not just the right types.
  const QLength distance = 24_in;
  const QTime elapsed = 2_s;
  const QVelocity speed = distance / elapsed;
  checkNear(speed.inps(), 12.0, 1e-12, "24 in over 2 s is 12 in/s");
  checkNear((speed * elapsed).in(), 24.0, 1e-12, "speed times time recovers the distance");

  const QAcceleration accel = speed / elapsed;
  checkNear(accel.raw(), (12.0 * 0.0254) / 2.0, 1e-12, "12 in/s over 2 s in m/s^2");

  // v^2 = 2*a*d, the trapezoid-profile identity later phases will lean on.
  const QAcceleration a = 20_mps2;
  const QLength d = 2.5_m;
  checkNear(std::sqrt((2.0 * a * d).raw()), 10.0, 1e-12, "sqrt(2*a*d) with a=20 m/s^2, d=2.5 m");

  // Angle helpers.
  checkNear(mclib::units::sin(90_deg), 1.0, 1e-12, "sin(90 deg)");
  checkNear(mclib::units::cos(180_deg), -1.0, 1e-12, "cos(180 deg)");
  checkNear(atan2(1_in, 1_in).deg(), 45.0, 1e-12, "atan2 of equal legs is 45 deg");
  checkNear(hypot(3_in, 4_in).in(), 5.0, 1e-12, "3-4-5 triangle");
  checkNear(wrap(270_deg).deg(), -90.0, 1e-12, "270 deg wraps to -90 deg");
  checkNear(wrap(-190_deg).deg(), 170.0, 1e-12, "-190 deg wraps to 170 deg");
  // The interval is half-open at +180, so exactly +180 must not flip sign. Test
  // the magnitude rather than the value: whether `180.0 * (pi/180.0)` lands on
  // pi exactly or one ulp above decides between +180 and -180, and that is a
  // toolchain detail, not a contract.
  checkNear(std::fabs(wrap(180_deg).deg()), 180.0, 1e-12, "180 deg stays at the wrap boundary");

  // Angle-length crossings.
  checkNear(arcLength(2_in, 1_rad).in(), 2.0, 1e-12, "arc length at r=2 in, 1 rad");
  checkNear(arcAngle(2_in, 2_in).rad(), 1.0, 1e-12, "arcAngle inverts arcLength");
  // A 4 in wheel (2 in radius) at 600 rpm.
  checkNear(rimVelocity(2_in, 600_rpm).inps(), 2.0 * 600.0 * 2.0 * M_PI / 60.0, 1e-9,
            "rim speed of a 4 in wheel at 600 rpm");

  // Ordering, min/max/clamp/abs/sign.
  check(1_in < 1_ft, "an inch is shorter than a foot");
  check(max(1_in, 1_ft) == 1_ft, "max picks the foot");
  check(min(1_in, 1_ft) == 1_in, "min picks the inch");
  check(clamp(5_ft, 1_in, 1_ft) == 1_ft, "clamp saturates high");
  check(clamp(-5_ft, 1_in, 1_ft) == 1_in, "clamp saturates low");
  check(abs(-3_in) == 3_in, "abs of a negative length");
  checkNear(sign(-3_in), -1.0, 0.0, "sign of a negative length");
  checkNear(sign(0_in), 0.0, 0.0, "sign of zero");

  // Compound assignment.
  QTime timer = 0_ms;
  timer += 250_ms;
  timer += 250_ms;
  checkNear(timer.ms(), 500.0, 1e-9, "accumulated timer");
  timer -= 500_ms;
  checkNear(timer.ms(), 0.0, 1e-9, "drained timer");
  QLength scaled = 10_in;
  scaled *= 3.0;
  scaled /= 2.0;
  checkNear(scaled.in(), 15.0, 1e-12, "scaled length");

  // The idiom the whole codebase uses for "now": pros::millis() returns a
  // uint32_t count of milliseconds.
  const std::uint32_t fake_millis = 1234;
  const QTime now = fake_millis * millisecond;
  checkNear(now.ms(), 1234.0, 1e-9, "pros::millis() * millisecond");
  const QTime start = 1000u * millisecond;
  check(now - start >= 200 * millisecond, "elapsed-time comparison against a timeout");
  check(!(now - start >= 300 * millisecond), "timeout that has not expired yet");

  // Dimensionless quantities are ordinary numbers.
  const QNumber gearing = QLength{36.0} / QLength{60.0};
  checkNear(gearing, 0.6, 1e-12, "a gear ratio is a plain number");
  checkNear(2.0 * gearing, 1.2, 1e-12, "and multiplies like one");

  // Default construction is zero, as the mechanism layer's `QTime x{}` relies on.
  const QTime zero{};
  check(zero == QTime{0.0}, "default-constructed QTime is zero");

  if (failures == 0) {
    std::printf("units_test: all checks passed\n");
    return 0;
  }
  std::printf("units_test: %d failure(s)\n", failures);
  return 1;
}
