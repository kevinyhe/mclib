// mclib
#pragma once

#include <cmath>
#include <compare>

/**
 * @file units.hpp
 * @brief Compile-time dimensional analysis for mclib.
 *
 * Every dimensioned value in the library is a Quantity: a single `double`
 * wrapped in a type that carries five integer exponents (length, time, angle,
 * voltage, current). Multiplying two quantities adds the exponents, dividing
 * subtracts them, so `QLength / QTime` *is* `QVelocity` and nothing else. All
 * operations are `constexpr` and `inline`; a Quantity has the same size and
 * codegen as the bare `double` it replaces, which matters on the V5 brain.
 *
 * The stored value is always in SI base units - metres, seconds, radians,
 * volts, amperes. That is an implementation detail: you never see it unless you
 * ask for it. Getting a raw number out requires naming the unit you want it in
 * (`value.in()`, `value.ms()`, `value.deg()`, `value.volts()`), and building
 * one from a raw number requires either a literal (`24_in`), a named constant
 * (`250 * millisecond`) or the explicit constructor.
 *
 * Angles are stored in radians. The public motion API of this library speaks
 * degrees and `Pose2D::theta` speaks radians; QAngle makes that conversion
 * explicit at every boundary instead of implicit and wrong.
 *
 * Back-compat: `QTime`, `millisecond` and `second` are also visible in the
 * global namespace, as they were before, so existing call sites such as
 * `pros::millis() * millisecond` keep working unchanged.
 */

namespace mclib::units {

inline constexpr double pi = 3.14159265358979323846;

/**
 * @brief A double tagged with compile-time dimension exponents.
 *
 * @tparam L Length exponent (base unit: metre)
 * @tparam T Time exponent (base unit: second)
 * @tparam A Angle exponent (base unit: radian)
 * @tparam V Voltage exponent (base unit: volt)
 * @tparam C Current exponent (base unit: ampere)
 */
template <int L, int T, int A, int V, int C>
class Quantity {
 public:
  /// True when every exponent is zero, i.e. this is a plain number.
  static constexpr bool is_dimensionless = (L == 0 && T == 0 && A == 0 && V == 0 && C == 0);

  /// A quantity of the same dimension, spelled out for template code.
  using Self = Quantity<L, T, A, V, C>;

  constexpr Quantity() = default;

  /**
   * @brief Build from a raw value already expressed in SI base units.
   *
   * Explicit on purpose: reaching for this means you are asserting the number
   * is in metres / seconds / radians / volts / amperes. Prefer a literal
   * (`24_in`) or a named constant (`250 * millisecond`).
   */
  explicit constexpr Quantity(double base_value)
    requires(!is_dimensionless)
      : m_value(base_value) {}

  /// @brief Dimensionless quantities convert freely from double.
  constexpr Quantity(double base_value)
    requires(is_dimensionless)
      : m_value(base_value) {}

  /// @brief Dimensionless quantities convert freely to double.
  constexpr operator double() const
    requires(is_dimensionless)
  {
    return m_value;
  }

  /**
   * @brief The raw stored value, in SI base units.
   *
   * The escape hatch of last resort. The named accessors below say what the
   * number means; this one does not.
   */
  constexpr double raw() const { return m_value; }

  /// @brief This value expressed as a multiple of @p unit.
  constexpr double convert(Self unit) const { return m_value / unit.m_value; }

  // -------------------------------------------------------------------------
  // Named accessors - the readable escape hatch back to double.
  //
  // Each one is constrained to the dimension it makes sense for, so `t.in()`
  // on a QTime is a compile error, not a wrong number. The constraints are
  // written out as raw exponents because the QLength/QTime aliases below do
  // not exist yet at this point in the file.
  // -------------------------------------------------------------------------

  /// @brief Length in metres.
  constexpr double m() const
    requires(L == 1 && T == 0 && A == 0 && V == 0 && C == 0)
  {
    return m_value;
  }
  /// @brief Length in centimetres.
  constexpr double cm() const
    requires(L == 1 && T == 0 && A == 0 && V == 0 && C == 0)
  {
    return m_value * 100.0;
  }
  /// @brief Length in millimetres.
  constexpr double mm() const
    requires(L == 1 && T == 0 && A == 0 && V == 0 && C == 0)
  {
    return m_value * 1000.0;
  }
  /// @brief Length in inches.
  constexpr double in() const
    requires(L == 1 && T == 0 && A == 0 && V == 0 && C == 0)
  {
    return m_value / 0.0254;
  }
  /// @brief Length in feet.
  constexpr double ft() const
    requires(L == 1 && T == 0 && A == 0 && V == 0 && C == 0)
  {
    return m_value / (0.0254 * 12.0);
  }

  /// @brief Duration in seconds.
  constexpr double s() const
    requires(L == 0 && T == 1 && A == 0 && V == 0 && C == 0)
  {
    return m_value;
  }
  /// @brief Duration in milliseconds - the unit PROS speaks.
  constexpr double ms() const
    requires(L == 0 && T == 1 && A == 0 && V == 0 && C == 0)
  {
    return m_value * 1000.0;
  }

  /// @brief Angle in radians. This is the stored representation.
  constexpr double rad() const
    requires(L == 0 && T == 0 && A == 1 && V == 0 && C == 0)
  {
    return m_value;
  }
  /// @brief Angle in degrees - the unit the public motion API speaks.
  constexpr double deg() const
    requires(L == 0 && T == 0 && A == 1 && V == 0 && C == 0)
  {
    return m_value / (pi / 180.0);
  }

  /// @brief Potential in volts.
  constexpr double volts() const
    requires(L == 0 && T == 0 && A == 0 && V == 1 && C == 0)
  {
    return m_value;
  }
  /// @brief Potential in millivolts - the unit V5 motor voltage control uses.
  constexpr double mV() const
    requires(L == 0 && T == 0 && A == 0 && V == 1 && C == 0)
  {
    return m_value * 1000.0;
  }

  /// @brief Current in amperes.
  constexpr double amps() const
    requires(L == 0 && T == 0 && A == 0 && V == 0 && C == 1)
  {
    return m_value;
  }
  /// @brief Current in milliamps - the unit the V5 motor reports.
  constexpr double mA() const
    requires(L == 0 && T == 0 && A == 0 && V == 0 && C == 1)
  {
    return m_value * 1000.0;
  }

  /// @brief Linear speed in metres per second.
  constexpr double mps() const
    requires(L == 1 && T == -1 && A == 0 && V == 0 && C == 0)
  {
    return m_value;
  }
  /// @brief Linear speed in inches per second.
  constexpr double inps() const
    requires(L == 1 && T == -1 && A == 0 && V == 0 && C == 0)
  {
    return m_value / 0.0254;
  }

  /// @brief Angular speed in radians per second.
  constexpr double radps() const
    requires(L == 0 && T == -1 && A == 1 && V == 0 && C == 0)
  {
    return m_value;
  }
  /// @brief Angular speed in degrees per second.
  constexpr double degps() const
    requires(L == 0 && T == -1 && A == 1 && V == 0 && C == 0)
  {
    return m_value / (pi / 180.0);
  }
  /// @brief Angular speed in revolutions per minute - the unit V5 motors use.
  constexpr double rpm() const
    requires(L == 0 && T == -1 && A == 1 && V == 0 && C == 0)
  {
    return m_value / (2.0 * pi / 60.0);
  }

  constexpr Self operator+() const { return *this; }
  constexpr Self operator-() const { return Self::fromBase(-m_value); }

  constexpr Self& operator+=(Self rhs) {
    m_value += rhs.m_value;
    return *this;
  }
  constexpr Self& operator-=(Self rhs) {
    m_value -= rhs.m_value;
    return *this;
  }
  constexpr Self& operator*=(double rhs) {
    m_value *= rhs;
    return *this;
  }
  constexpr Self& operator/=(double rhs) {
    m_value /= rhs;
    return *this;
  }

  // These are all suppressed for the dimensionless case. A dimensionless
  // Quantity converts to and from double implicitly, so defining them there
  // would make every `double + QNumber` ambiguous against the built-in
  // operator. Dimensionless quantities just fall through to plain double
  // arithmetic, which is what they are.

  friend constexpr Self operator+(Self lhs, Self rhs)
    requires(!is_dimensionless)
  {
    return Self::fromBase(lhs.m_value + rhs.m_value);
  }
  friend constexpr Self operator-(Self lhs, Self rhs)
    requires(!is_dimensionless)
  {
    return Self::fromBase(lhs.m_value - rhs.m_value);
  }
  friend constexpr Self operator*(Self lhs, double rhs)
    requires(!is_dimensionless)
  {
    return Self::fromBase(lhs.m_value * rhs);
  }
  friend constexpr Self operator*(double lhs, Self rhs)
    requires(!is_dimensionless)
  {
    return Self::fromBase(lhs * rhs.m_value);
  }
  friend constexpr Self operator/(Self lhs, double rhs)
    requires(!is_dimensionless)
  {
    return Self::fromBase(lhs.m_value / rhs);
  }

  friend constexpr bool operator==(Self lhs, Self rhs)
    requires(!is_dimensionless)
  {
    return lhs.m_value == rhs.m_value;
  }
  friend constexpr std::partial_ordering operator<=>(Self lhs, Self rhs)
    requires(!is_dimensionless)
  {
    return lhs.m_value <=> rhs.m_value;
  }

  /**
   * @brief Wrap a raw base-unit value without the explicit-constructor dance.
   *
   * Plumbing for the free operators below, which construct results whose
   * dimension differs from either operand.
   */
  static constexpr Self fromBase(double base_value) {
    Self result;
    result.m_value = base_value;
    return result;
  }

 private:
  double m_value{0.0};
};

/// @brief Product of two quantities; exponents add.
template <int L1, int T1, int A1, int V1, int C1, int L2, int T2, int A2, int V2, int C2>
constexpr Quantity<L1 + L2, T1 + T2, A1 + A2, V1 + V2, C1 + C2> operator*(
    Quantity<L1, T1, A1, V1, C1> lhs, Quantity<L2, T2, A2, V2, C2> rhs) {
  return Quantity<L1 + L2, T1 + T2, A1 + A2, V1 + V2, C1 + C2>::fromBase(lhs.raw() * rhs.raw());
}

/// @brief Quotient of two quantities; exponents subtract.
template <int L1, int T1, int A1, int V1, int C1, int L2, int T2, int A2, int V2, int C2>
constexpr Quantity<L1 - L2, T1 - T2, A1 - A2, V1 - V2, C1 - C2> operator/(
    Quantity<L1, T1, A1, V1, C1> lhs, Quantity<L2, T2, A2, V2, C2> rhs) {
  return Quantity<L1 - L2, T1 - T2, A1 - A2, V1 - V2, C1 - C2>::fromBase(lhs.raw() / rhs.raw());
}

/// @brief Reciprocal of a quantity, e.g. `1.0 / a_time` is a frequency.
template <int L, int T, int A, int V, int C>
  requires(!Quantity<L, T, A, V, C>::is_dimensionless)
constexpr Quantity<-L, -T, -A, -V, -C> operator/(double lhs, Quantity<L, T, A, V, C> rhs) {
  return Quantity<-L, -T, -A, -V, -C>::fromBase(lhs / rhs.raw());
}

// ---------------------------------------------------------------------------
// Named dimensions
// ---------------------------------------------------------------------------

/// A plain number that still participates in dimensional analysis.
using QNumber = Quantity<0, 0, 0, 0, 0>;
using QLength = Quantity<1, 0, 0, 0, 0>;
using QArea = Quantity<2, 0, 0, 0, 0>;
using QTime = Quantity<0, 1, 0, 0, 0>;
using QAngle = Quantity<0, 0, 1, 0, 0>;
using QVoltage = Quantity<0, 0, 0, 1, 0>;
using QCurrent = Quantity<0, 0, 0, 0, 1>;

using QVelocity = Quantity<1, -1, 0, 0, 0>;
using QAcceleration = Quantity<1, -2, 0, 0, 0>;
using QJerk = Quantity<1, -3, 0, 0, 0>;
using QAngularVelocity = Quantity<0, -1, 1, 0, 0>;
using QAngularAcceleration = Quantity<0, -2, 1, 0, 0>;
/// 1 / length - what a pure-pursuit or boomerang controller tracks.
using QCurvature = Quantity<-1, 0, 0, 0, 0>;
using QFrequency = Quantity<0, -1, 0, 0, 0>;

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

inline constexpr QNumber number{1.0};
inline constexpr QNumber percent{0.01};

inline constexpr QLength metre = QLength::fromBase(1.0);
inline constexpr QLength meter = metre;
inline constexpr QLength centimetre = metre / 100.0;
inline constexpr QLength centimeter = centimetre;
inline constexpr QLength millimetre = metre / 1000.0;
inline constexpr QLength millimeter = millimetre;
// Written as exact base-unit values rather than `millimetre * 25.4` so that
// `inch.in()` is exactly 1.0 and `(24_in).in()` is exactly 24.0. Going through
// millimetre first costs a rounding step and leaves 24 in reading 23.999999999999996.
inline constexpr QLength inch = QLength::fromBase(0.0254);
inline constexpr QLength foot = QLength::fromBase(0.0254 * 12.0);
/// One VEX field tile, 24 inches.
inline constexpr QLength tile = QLength::fromBase(0.0254 * 24.0);

inline constexpr QTime second = QTime::fromBase(1.0);
inline constexpr QTime millisecond = second / 1000.0;
inline constexpr QTime minute = second * 60.0;

inline constexpr QAngle radian = QAngle::fromBase(1.0);
inline constexpr QAngle degree = radian * (pi / 180.0);
inline constexpr QAngle rotation = radian * (2.0 * pi);

inline constexpr QVoltage volt = QVoltage::fromBase(1.0);
inline constexpr QVoltage millivolt = volt / 1000.0;

inline constexpr QCurrent ampere = QCurrent::fromBase(1.0);
inline constexpr QCurrent milliampere = ampere / 1000.0;

inline constexpr QVelocity mps = metre / second;
inline constexpr QVelocity inps = inch / second;

inline constexpr QAcceleration mps2 = metre / (second * second);

inline constexpr QAngularVelocity radps = radian / second;
inline constexpr QAngularVelocity degps = degree / second;
inline constexpr QAngularVelocity rpm = rotation / minute;

// ---------------------------------------------------------------------------
// Named accessors
//
// Free-function spellings of the member accessors above, for call sites where
// `milliseconds(t)` reads better than `t.ms()`.
// ---------------------------------------------------------------------------

#define MCLIB_UNIT_ACCESSOR(qtype, name, unit)                       \
  constexpr double name(qtype value) { return value.convert(unit); } \
  static_assert(true, "")

MCLIB_UNIT_ACCESSOR(QLength, meters, metre);
MCLIB_UNIT_ACCESSOR(QLength, centimeters, centimetre);
MCLIB_UNIT_ACCESSOR(QLength, millimeters, millimetre);
MCLIB_UNIT_ACCESSOR(QLength, inches, inch);
MCLIB_UNIT_ACCESSOR(QLength, feet, foot);
MCLIB_UNIT_ACCESSOR(QTime, seconds, second);
MCLIB_UNIT_ACCESSOR(QTime, milliseconds, millisecond);
MCLIB_UNIT_ACCESSOR(QAngle, radians, radian);
MCLIB_UNIT_ACCESSOR(QAngle, degrees, degree);
MCLIB_UNIT_ACCESSOR(QVoltage, volts, volt);
MCLIB_UNIT_ACCESSOR(QVoltage, millivolts, millivolt);
MCLIB_UNIT_ACCESSOR(QCurrent, amperes, ampere);
MCLIB_UNIT_ACCESSOR(QCurrent, milliamperes, milliampere);

#undef MCLIB_UNIT_ACCESSOR

// ---------------------------------------------------------------------------
// Math on quantities
// ---------------------------------------------------------------------------

/// @brief Absolute value, dimension preserved.
template <int L, int T, int A, int V, int C>
constexpr Quantity<L, T, A, V, C> abs(Quantity<L, T, A, V, C> value) {
  return value.raw() < 0.0 ? -value : value;
}

/// @brief Smaller of two same-dimension quantities.
template <int L, int T, int A, int V, int C>
constexpr Quantity<L, T, A, V, C> min(Quantity<L, T, A, V, C> a, Quantity<L, T, A, V, C> b) {
  return a.raw() < b.raw() ? a : b;
}

/// @brief Larger of two same-dimension quantities.
template <int L, int T, int A, int V, int C>
constexpr Quantity<L, T, A, V, C> max(Quantity<L, T, A, V, C> a, Quantity<L, T, A, V, C> b) {
  return a.raw() > b.raw() ? a : b;
}

/// @brief @p value clamped into [@p lo, @p hi].
template <int L, int T, int A, int V, int C>
constexpr Quantity<L, T, A, V, C> clamp(Quantity<L, T, A, V, C> value, Quantity<L, T, A, V, C> lo,
                                        Quantity<L, T, A, V, C> hi) {
  return min(max(value, lo), hi);
}

/// @brief -1, 0 or +1 as a plain double.
template <int L, int T, int A, int V, int C>
constexpr double sign(Quantity<L, T, A, V, C> value) {
  return value.raw() > 0.0 ? 1.0 : (value.raw() < 0.0 ? -1.0 : 0.0);
}

/// @brief Square of a quantity; exponents double.
template <int L, int T, int A, int V, int C>
constexpr Quantity<2 * L, 2 * T, 2 * A, 2 * V, 2 * C> square(Quantity<L, T, A, V, C> value) {
  return value * value;
}

inline double sin(QAngle angle) { return std::sin(angle.raw()); }
inline double cos(QAngle angle) { return std::cos(angle.raw()); }
inline double tan(QAngle angle) { return std::tan(angle.raw()); }
inline QAngle asin(double value) { return QAngle::fromBase(std::asin(value)); }
inline QAngle acos(double value) { return QAngle::fromBase(std::acos(value)); }
inline QAngle atan(double value) { return QAngle::fromBase(std::atan(value)); }

/// @brief Heading of the vector (@p x, @p y), in the range (-pi, pi].
inline QAngle atan2(QLength y, QLength x) { return QAngle::fromBase(std::atan2(y.raw(), x.raw())); }

/// @brief Length of the vector (@p x, @p y).
inline QLength hypot(QLength x, QLength y) { return QLength::fromBase(std::hypot(x.raw(), y.raw())); }

/**
 * @brief Wrap an angle into (-180 deg, +180 deg].
 */
inline QAngle wrap(QAngle angle) {
  double value = std::fmod(angle.raw() + pi, 2.0 * pi);
  if (value <= 0.0) {
    value += 2.0 * pi;
  }
  return QAngle::fromBase(value - pi);
}

/**
 * @brief Arc length swept by @p angle at radius @p radius.
 *
 * Angle is a real dimension here, so `radius * angle` does not type-check on
 * its own. This is the sanctioned crossing: it drops the angle exponent the way
 * the physics does.
 */
constexpr QLength arcLength(QLength radius, QAngle angle) {
  return QLength::fromBase(radius.raw() * angle.raw());
}

/// @brief Inverse of arcLength(): the angle an arc of @p arc subtends.
constexpr QAngle arcAngle(QLength arc, QLength radius) {
  return QAngle::fromBase(arc.raw() / radius.raw());
}

/// @brief Linear speed at the rim of a wheel of radius @p radius.
constexpr QVelocity rimVelocity(QLength radius, QAngularVelocity omega) {
  return QVelocity::fromBase(radius.raw() * omega.raw());
}

// ---------------------------------------------------------------------------
// Literals
// ---------------------------------------------------------------------------

inline namespace literals {

#define MCLIB_UNIT_LITERAL(suffix, unit)                                        \
  constexpr auto operator""_##suffix(long double value) {                       \
    return static_cast<double>(value) * (unit);                                 \
  }                                                                             \
  constexpr auto operator""_##suffix(unsigned long long value) {                \
    return static_cast<double>(value) * (unit);                                 \
  }                                                                             \
  static_assert(true, "")

MCLIB_UNIT_LITERAL(m, metre);
MCLIB_UNIT_LITERAL(cm, centimetre);
MCLIB_UNIT_LITERAL(mm, millimetre);
MCLIB_UNIT_LITERAL(in, inch);
MCLIB_UNIT_LITERAL(ft, foot);
MCLIB_UNIT_LITERAL(tile, tile);

MCLIB_UNIT_LITERAL(s, second);
MCLIB_UNIT_LITERAL(ms, millisecond);
MCLIB_UNIT_LITERAL(min, minute);

MCLIB_UNIT_LITERAL(rad, radian);
MCLIB_UNIT_LITERAL(deg, degree);
MCLIB_UNIT_LITERAL(rot, rotation);

MCLIB_UNIT_LITERAL(V, volt);
MCLIB_UNIT_LITERAL(mV, millivolt);
MCLIB_UNIT_LITERAL(A, ampere);
MCLIB_UNIT_LITERAL(mA, milliampere);

MCLIB_UNIT_LITERAL(mps, mps);
MCLIB_UNIT_LITERAL(inps, inps);
MCLIB_UNIT_LITERAL(mps2, mps2);
MCLIB_UNIT_LITERAL(radps, radps);
MCLIB_UNIT_LITERAL(degps, degps);
MCLIB_UNIT_LITERAL(rpm, rpm);

#undef MCLIB_UNIT_LITERAL

}  // namespace literals

}  // namespace mclib::units

// ---------------------------------------------------------------------------
// Global back-compat surface
//
// QTime, millisecond and second lived in the global namespace before this file
// grew a type system, and roughly fifteen call sites across the mechanism,
// command and auton headers spell `pros::millis() * millisecond` unqualified.
// Those names stay global, and there is deliberately no opt-out macro: mclib's
// own headers depend on these aliases, so making them conditional would just
// mean the library stops compiling. `Quantity` itself is *not* exported - it is
// a common enough identifier that a downstream global `class Quantity` would
// become ambiguous - so spell it `mclib::units::Quantity` when you need the
// template by name.
// ---------------------------------------------------------------------------

using mclib::units::QAcceleration;
using mclib::units::QAngle;
using mclib::units::QAngularAcceleration;
using mclib::units::QAngularVelocity;
using mclib::units::QArea;
using mclib::units::QCurrent;
using mclib::units::QCurvature;
using mclib::units::QFrequency;
using mclib::units::QJerk;
using mclib::units::QLength;
using mclib::units::QNumber;
using mclib::units::QTime;
using mclib::units::QVelocity;
using mclib::units::QVoltage;

using mclib::units::millisecond;
using mclib::units::second;

using namespace mclib::units::literals;
