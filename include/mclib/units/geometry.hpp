// mclib
#pragma once

#include "mclib/units/units.hpp"

/**
 * @file geometry.hpp
 * @brief Drivetrain geometry as a type, so diameter and circumference cannot
 *        be swapped.
 *
 * The robot's wheel size used to be described three times, in three places,
 * with three different numbers and one misleading name:
 *
 * - `config.cpp`: `wheel_distance_in = 9.06`. Despite the name this was a
 *   *circumference*: every use site was `deg * wheel_distance_in / 360.0`.
 *   9.06 in of circumference is a 2.884 in diameter.
 * - `config.cpp`: `vertical_tracker_diameter = 2`. This one really was a
 *   diameter: its use site was `deg * d * M_PI / 360.0`.
 * - `chassis.hpp`: `ChassisDimensions::wheel_diameter_in = 2.75`, a third
 *   number for the same robot, 4.87% off the first.
 *
 * Three `double`s, three conventions, nothing to stop you feeding one into a
 * formula written for another. Wheel fixes that by having no constructor you
 * can call with a bare number: you must say `Wheel::fromDiameter(...)` or
 * `Wheel::fromCircumference(...)`, and both take a QLength, so the unit is
 * carried too. Once built, all three of diameter(), radius() and
 * circumference() are available and consistent by construction.
 *
 * There is now one value of each: `mclib::config::robot_drive_geometry` and
 * `mclib::config::vertical_tracking_wheel`, which the user states for their
 * own robot. `ChassisDimensions` is an alias for DriveGeometry and the four
 * `double` globals are gone.
 *
 * Header-only and entirely constexpr; it pulls in no PROS header, so it is
 * host-testable (see tests/geometry_test.cpp).
 */

namespace mclib::units {

/**
 * @brief A round wheel, stored as its diameter.
 *
 * Not constructible from a raw number and not default-constructible. The only
 * ways in are the two named factories, which state which measurement you have.
 */
class Wheel {
 public:
  Wheel() = delete;

  /// @brief A wheel of the given diameter, e.g. `Wheel::fromDiameter(2.75_in)`.
  static constexpr Wheel fromDiameter(QLength diameter) { return Wheel(diameter); }

  /**
   * @brief A wheel of the given rolling circumference.
   *
   * Use this when what you measured is how far the robot moves per wheel
   * revolution - which is what a tape measure around a compressed tread gives
   * you, and what the old `wheel_distance_in = 9.06` actually was.
   */
  static constexpr Wheel fromCircumference(QLength circumference) {
    return Wheel(circumference / pi);
  }

  /// @brief Wheel diameter.
  constexpr QLength diameter() const { return m_diameter; }
  /// @brief Wheel radius, half the diameter.
  constexpr QLength radius() const { return m_diameter / 2.0; }
  /// @brief Distance travelled per full wheel revolution, pi * diameter.
  constexpr QLength circumference() const { return m_diameter * pi; }

  // Deliberately no operator==. Comparing two wheels means comparing two
  // doubles that have been through a metre round trip, and units.hpp is
  // explicit that lengths must be compared with a tolerance: over random
  // diameters, `Wheel::fromCircumference(w.circumference()) == w` is false
  // about 15% of the time. Compare `a.diameter() - b.diameter()` against a
  // tolerance you choose instead.

 private:
  explicit constexpr Wheel(QLength diameter) : m_diameter(diameter) {}

  QLength m_diameter;
};

/**
 * @brief Everything the odometry and motion math needs to know about the
 *        drivetrain's shape.
 *
 * One value replaces the `wheel_distance_in` / `distance_between_wheels` /
 * `ChassisDimensions` triple. The encoder-degrees-to-inches conversion lives
 * here as a method rather than being open-coded, as it was at ten-odd sites
 * across motion.cpp and odometry.cpp, each of which had to remember the
 * `/ 360.0` for itself.
 */
struct DriveGeometry {
  /// The wheel the encoder is geared to.
  Wheel wheel;
  /// Distance between the left and right wheel contact patches.
  QLength track_width;
  /**
   * Wheel revolutions per encoder revolution.
   *
   * 1.0 for a rotation sensor on the wheel shaft, or for a direct-drive motor
   * encoder. Under 1.0 when the wheel turns slower than the sensor.
   */
  double gear_ratio = 1.0;

  /**
   * @brief Distance rolled for a given encoder rotation.
   *
   * Replaced `deg * wheel_distance_in / 360.0` and
   * `deg * tracker_diameter * M_PI / 360.0` - both are this same formula
   * written from a different starting measurement. It computes in metres, so
   * it can differ from the inches-only spelling by up to 2 ulp; see
   * tests/geometry_test.cpp.
   */
  constexpr QLength encoderToDistance(QAngle encoder_angle) const {
    return arcLength(wheel.radius() * gear_ratio, encoder_angle);
  }

  /// @brief Inverse of encoderToDistance(): encoder rotation to roll @p distance.
  constexpr QAngle distanceToEncoder(QLength distance) const {
    return arcAngle(distance, wheel.radius() * gear_ratio);
  }

  /// @brief Half the track width - the radius each wheel turns about when the
  ///        robot spins in place.
  constexpr QLength turnRadius() const { return track_width / 2.0; }

  /// @brief Distance one side must roll for the robot to spin in place by
  ///        @p robot_turn.
  constexpr QLength spinArc(QAngle robot_turn) const {
    return arcLength(turnRadius(), robot_turn);
  }
};

/**
 * @brief A free-spinning tracking wheel: a Wheel plus its offset from the
 *        tracking centre.
 *
 * Same conversion as DriveGeometry, without a track width. `offset` is signed
 * and follows the old `vertical_tracker_dist_from_center` convention.
 */
struct TrackingWheel {
  Wheel wheel;
  QLength offset;
  double gear_ratio = 1.0;

  /// @brief Distance rolled for a given encoder rotation.
  constexpr QLength encoderToDistance(QAngle encoder_angle) const {
    return arcLength(wheel.radius() * gear_ratio, encoder_angle);
  }

  /// @brief Inverse of encoderToDistance().
  constexpr QAngle distanceToEncoder(QLength distance) const {
    return arcAngle(distance, wheel.radius() * gear_ratio);
  }
};

}  // namespace mclib::units
