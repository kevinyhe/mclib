// mclib
#pragma once

#include "mclib/device/types.hpp"

#include "mclib/units/units.hpp"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

/**
 * @brief One V5 smart motor.
 *
 * Every getter comes in two spellings: the original `double` one, whose unit
 * lives only in its name, and a units-typed sibling that carries it in the
 * type. Both read the same hardware; the typed one is the one to write new
 * code against. Two members have no typed sibling on purpose:
 * getTemperature() (the unit system has no temperature dimension) and
 * setPercent(), whose argument is a -1.0..1.0 FRACTION, not 0..100, and which
 * would become ambiguous against a dimensionless Quantity overload.
 */
class Motor {
public:
  explicit Motor(std::int8_t port, Gearset gearset = Gearset::Blue);

  void setVoltage(double volts);
  void setPercent(double percent);
  void stop();
  void setBrakeMode(BrakeMode mode);
  void tarePosition();

  double getPositionDeg() const;
  double getActualVelocity() const;
  double getCurrentDraw() const;
  double getTemperature() const;

  /**
   * @brief Commanded voltage, typed. Sibling of setVoltage(double), clamped to
   *        the same +/-12 V rails.
   *
   * Overloads rather than replaces: `setVoltage(6.0)` still resolves to the
   * double version, because QVoltage's constructor is explicit.
   */
  void setVoltage(units::QVoltage voltage);

  /**
   * @brief Motor position, typed, or nullopt when the motor is not reporting.
   *
   * Sibling of getPositionDeg(), which returns PROS_ERR_F - positive infinity
   * - on a dead or misconfigured port.
   */
  std::optional<units::QAngle> position() const;

  /**
   * @brief Shaft speed, typed, or nullopt when the motor is not reporting.
   *
   * Sibling of getActualVelocity(), which returns gearset-dependent RPM, and
   * positive infinity on a dead port.
   */
  std::optional<units::QAngularVelocity> velocity() const;

  /**
   * @brief Current draw, typed, or nullopt when the motor is not reporting.
   *
   * Sibling of getCurrentDraw(), which returns MILLIAMPS - the 1000x trap this
   * accessor exists to close. Compare against `2.5_A`, not against 2500.
   *
   * The optional closes the other half of that trap: a dead port reads
   * PROS_ERR = 2 147 483 647 mA, so `getCurrentDraw() > 2500` is true for an
   * unplugged motor and a stall check would fire on nothing.
   */
  std::optional<units::QCurrent> current() const;

private:
  static std::int32_t voltsToMillivolts(double volts);
  static std::int32_t percentToPower(double percent);

  pros::Motor m_motor;
};

}  // namespace device
}  // namespace mclib
