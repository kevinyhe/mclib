// mclib
#pragma once

/**
 * @file pure_pursuit.hpp
 * @brief Pure-pursuit path following with curvature-based speed limiting.
 *
 * @details Give the follower a baked `Path` and call `update()` once per
 * control tick with the current field pose. It returns the goal point it is
 * chasing, the signed curvature of the arc that reaches it, a speed, and the
 * left/right wheel speeds that produce that arc. It owns no motors, no tasks
 * and no clock, so it is fully host-testable and can be driven from anywhere.
 *
 * ## Frame
 *
 * Field frame, compass convention (`math.hpp`): heading 0 is +Y, clockwise-
 * positive. Curvature is signed like `mclib::arcRadius()`: **positive curves
 * to the robot's right.** A robot sitting on a straight path aimed along it
 * gets curvature exactly zero.
 *
 * ## The four cases that break naive implementations
 *
 * - **Multiple intersections.** The lookahead circle can cut the path in
 *   several places. The follower takes the **first intersection at or after
 *   the monotone lookahead cursor**, walking segments forward from where it
 *   left off last tick. "First ahead" and not "furthest along" is the whole
 *   point: on a hairpin the far branch is also inside the circle, and chasing
 *   it would cut the corner and skip the rest of the path. The cursor never
 *   moves backwards, so a path that passes near itself cannot drag the goal
 *   back to an earlier lap.
 * - **Off the path.** When the closest point on the path is further away than
 *   the lookahead, no intersection exists at all. The follower then aims at
 *   the path point one lookahead **beyond the closest point**, which is a
 *   smooth rejoin rather than a lunge at the endpoint, and reports
 *   `off_path = true`.
 * - **End of path.** When the search runs off the end without an
 *   intersection, the goal is the final path point and `at_end` is true. The
 *   effective lookahead then shrinks as the robot arrives, so curvature is
 *   clamped to `PurePursuitConfig::max_curvature` to stop the gain running
 *   away. Once the remaining arc length is inside `finish_tolerance`,
 *   `finished` is true and the speeds are zero.
 * - **Doubling back.** Both the closest-point search and the lookahead search
 *   are forward-only from their cursors, and the closest-point search is also
 *   bounded to `search_window` of arc length ahead. A path that returns near
 *   its own start therefore cannot capture the cursor.
 *
 * ## Speed
 *
 * Three limits are applied and the smallest wins: the configured maximum, the
 * lateral-acceleration limit `sqrt(a_lat / |k|)` evaluated over the tightest
 * curvature in the next lookahead of path (so the robot brakes *before* the
 * corner, not in it), and the endpoint ramp `sqrt(2 a_decel * remaining)`.
 * `min_velocity` puts a floor under the result so the robot does not stall in
 * a corner.
 *
 * These are deliberately kinematic one-liners with no state. A trapezoidal or
 * S-curve profile can be layered on top later by ignoring
 * `PurePursuitOutput::velocity` and feeding the profile's speed through
 * `wheelSpeeds()` with the reported curvature.
 */

#include "mclib/math.hpp"
#include "mclib/path/path.hpp"
#include "mclib/telemetry/logger.hpp"
#include "mclib/units/units.hpp"

#include <cstddef>

namespace mclib {
namespace path {

using units::QAcceleration;
using units::QAngularVelocity;
using units::QVelocity;

/**
 * @brief Left and right wheel speeds for a differential drive.
 */
struct WheelSpeeds {
  QVelocity left{};
  QVelocity right{};
};

/**
 * @brief Tuning for `PurePursuit`.
 */
struct PurePursuitConfig {
  /**
   * @brief Lookahead distance: how far ahead on the path to aim.
   *
   * @details The one knob that matters. Too small oscillates, too large cuts
   * corners. Roughly one to two robot lengths is a sane start.
   */
  QLength lookahead = 12.0 * units::inch;

  /// @brief Distance between the left and right wheels, for `wheelSpeeds()`.
  QLength track_width = 12.0 * units::inch;

  /// @brief Top speed the follower will ever ask for.
  QVelocity max_velocity = 48.0 * units::inps;

  /**
   * @brief Floor under the commanded speed while the path is unfinished.
   *
   * @details Stops a tight corner or a long endpoint ramp from asking for a
   * speed the drivetrain cannot actually move at. Never applied once
   * `finished` is true.
   */
  QVelocity min_velocity = 6.0 * units::inps;

  /**
   * @brief Lateral acceleration budget for the curvature speed limit.
   *
   * @details Effectively "how hard may the robot corner before it skids".
   * Lower it if the robot pushes wide on tight turns.
   */
  QAcceleration max_lateral_accel = 60.0 * units::inps / units::second;

  /// @brief Deceleration used for the ramp into the end of the path.
  QAcceleration max_decel = 40.0 * units::inps / units::second;

  /**
   * @brief Hard cap on commanded curvature magnitude.
   *
   * @details Only bites near the endpoint, where the effective lookahead
   * shrinks and the pure-pursuit gain diverges. The default is a 6 inch turn
   * radius, tighter than any real drivetrain will track. Zero disables the
   * clamp.
   */
  units::QCurvature max_curvature = 1.0 / (6.0 * units::inch);

  /// @brief Remaining arc length at which the path counts as finished.
  QLength finish_tolerance = 2.0 * units::inch;

  /**
   * @brief How far ahead of the current closest point to keep searching for a
   *        closer one.
   *
   * @details Bounds the forward jump the closest-point cursor may take in one
   * tick. Large enough to recover from a skid, small enough that a path
   * doubling back on itself cannot steal the cursor.
   */
  QLength search_window = 24.0 * units::inch;
};

/**
 * @brief Everything one `update()` decided, and why.
 *
 * @details Log `lookahead_point`, `cross_track_error` and `curvature`. A path
 * follower you cannot plot is a path follower you cannot tune.
 */
struct PurePursuitOutput {
  /// @brief The goal point being chased, field inches.
  Vec2 lookahead_point{0.0, 0.0};

  /// @brief Signed curvature of the arc from the pose to the goal; + is right.
  units::QCurvature curvature{};

  /// @brief Commanded chassis speed after every limit.
  QVelocity velocity{};

  /// @brief Commanded turn rate, `turnRate(velocity, curvature)`.
  QAngularVelocity turn_rate{};

  /// @brief Left/right wheel speeds for `velocity` on that arc.
  WheelSpeeds wheels{};

  /**
   * @brief Signed lateral offset from the path at the closest point.
   * @details **Positive means the robot is to the right of the path**, so a
   *          correct follower answers with a negative (left) curvature.
   */
  QLength cross_track_error{};

  /// @brief Arc length of the closest point from the start of the path.
  QLength distance_along{};

  /// @brief Arc length still to go, `path.length() - distance_along`.
  QLength remaining{};

  /// @brief Straight-line distance from the robot to the closest path point.
  QLength path_error{};

  /// @brief True when the closest point is further away than the lookahead.
  bool off_path = false;

  /// @brief True when the lookahead ran off the end and the goal is the endpoint.
  bool at_end = false;

  /// @brief True when `remaining` is inside `finish_tolerance`; speeds are zero.
  bool finished = false;
};

/**
 * @brief Speed cap from a lateral acceleration budget: `sqrt(a / |k|)`.
 * @param curvature Signed or unsigned; only the magnitude is used.
 * @param max_velocity Returned unchanged when the curvature is ~straight.
 * @param max_lateral_accel Cornering acceleration budget.
 */
QVelocity curvatureSpeedLimit(units::QCurvature curvature, QVelocity max_velocity,
                              QAcceleration max_lateral_accel);

/**
 * @brief Speed cap that still stops in @p remaining: `sqrt(2 a d)`.
 * @param remaining Arc length left. Negative is treated as zero.
 * @param max_decel Deceleration budget.
 */
QVelocity approachSpeedLimit(QLength remaining, QAcceleration max_decel);

/**
 * @brief Split a chassis speed onto an arc of the given curvature.
 * @param velocity Speed of the chassis centre.
 * @param curvature Signed; positive turns right, so the right wheel is slower.
 * @param track_width Distance between the wheels.
 */
WheelSpeeds wheelSpeeds(QVelocity velocity, units::QCurvature curvature,
                        QLength track_width);

/**
 * @brief A pure-pursuit follower over one baked `Path`.
 *
 * @details Stateful only in its two search cursors. Construct it, call
 * `update()` each tick, drive the motors with what it returns, and stop when
 * `PurePursuitOutput::finished`. `reset()` puts the cursors back to the start.
 */
class PurePursuit {
 public:
  PurePursuit() = default;

  /**
   * @brief Follow @p path with @p config.
   * @param path Baked path, copied in. An invalid path makes every `update()`
   *        return a zeroed, `finished` output.
   */
  explicit PurePursuit(Path path, const PurePursuitConfig& config = {});

  /// @brief Compute the command for @p pose. Advances the search cursors.
  PurePursuitOutput update(const Pose2D& pose);

  /// @brief Rewind both search cursors to the start of the path.
  void reset();

  /// @brief Replace the path and rewind the cursors.
  void setPath(Path path);

  /// @brief Replace the tuning. Does not move the cursors.
  void setConfig(const PurePursuitConfig& config) { m_config = config; }

  /// @brief The path being followed.
  const Path& path() const { return m_path; }

  /// @brief The current tuning.
  const PurePursuitConfig& config() const { return m_config; }

  /// @brief Arc length of the closest point as of the last `update()`.
  QLength progress() const { return m_progress; }

  /**
   * @brief Register the four channels worth plotting on @p logger.
   *
   * @details Optional. After this, every `update()` writes the goal point,
   * cross-track error and curvature into the logger; the caller still owns
   * calling `sample()`. Registration must happen before the logger's first
   * sample, as with any channel.
   */
  void attachLogger(telemetry::Logger& logger, const char* prefix = "pp");

  /// @brief Stop writing to the logger passed to `attachLogger()`.
  void detachLogger() { m_logger = nullptr; }

 private:
  /// @brief Where on the path a point sits: segment index plus interpolation.
  struct Projection {
    std::size_t index = 0;   ///< Index of the segment's first sample.
    double t = 0.0;          ///< Position within that segment, 0 to 1.
    QLength distance{};      ///< Arc length from the start of the path.
    Vec2 point{0.0, 0.0};    ///< The projected point, field inches.
    QLength error{};         ///< Distance from the queried point to `point`.
  };

  /// @brief Forward-only, window-bounded closest-point search.
  Projection closestPoint(const Vec2& position) const;

  /**
   * @brief First lookahead-circle intersection at or after the cursor.
   * @param found Set false when the search ran off the end of the path.
   */
  Vec2 findLookaheadPoint(const Vec2& position, const Projection& closest, bool& found);

  void publish(const PurePursuitOutput& output) const;

  Path m_path;
  PurePursuitConfig m_config;

  // Search cursors. Both only ever move forward; `reset()` is the only way
  // back. This is what stops a path that doubles back from dragging the goal
  // point onto an earlier part of the route.
  std::size_t m_closest_index = 0;
  double m_closest_t = 0.0;
  std::size_t m_lookahead_index = 0;
  double m_lookahead_t = 0.0;
  QLength m_progress{};

  telemetry::Logger* m_logger = nullptr;
  telemetry::Channel<QLength> m_channel_goal_x;
  telemetry::Channel<QLength> m_channel_goal_y;
  telemetry::Channel<QLength> m_channel_cross_track;
  telemetry::Channel<double> m_channel_curvature;
  telemetry::Channel<units::QVelocity> m_channel_velocity;
};

}  // namespace path
}  // namespace mclib
