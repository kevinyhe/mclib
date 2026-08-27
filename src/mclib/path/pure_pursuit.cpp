// mclib
#include "mclib/path/pure_pursuit.hpp"

#include <cmath>
#include <cstdio>

namespace mclib {
namespace path {

namespace {

/// @brief Wrap a plain inch count as a QLength.
inline QLength inches(double value) { return units::inch * value; }

/// @brief Curvature magnitudes below this (1/m) are a straight line.
constexpr double kStraightCurvature = 1e-9;

/**
 * @brief First intersection of a circle with a segment, at or after @p min_t.
 *
 * @param a Segment start, inches. @param b Segment end, inches.
 * @param centre Circle centre, inches. @param radius Circle radius, inches.
 * @param min_t Lower bound on the segment parameter, 0 to 1.
 * @param t Set to the accepted parameter when this returns true.
 */
bool segmentCircleIntersection(const Vec2& a, const Vec2& b, const Vec2& centre,
                               double radius, double min_t, double& t) {
  const Vec2 d = b - a;
  const Vec2 f = a - centre;
  const double qa = d.squaredNorm();
  if (qa <= 0.0) {
    return false;
  }
  const double qb = 2.0 * f.dot(d);
  const double qc = f.squaredNorm() - radius * radius;
  const double discriminant = qb * qb - 4.0 * qa * qc;
  if (discriminant < 0.0) {
    return false;
  }

  const double root = std::sqrt(discriminant);
  // Smaller root first: "first intersection walking forward along the path".
  const double candidates[2] = {(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)};
  for (const double candidate : candidates) {
    if (candidate >= min_t && candidate <= 1.0) {
      t = candidate;
      return true;
    }
  }
  return false;
}

}  // namespace

QVelocity curvatureSpeedLimit(units::QCurvature curvature, QVelocity max_velocity,
                              QAcceleration max_lateral_accel) {
  const double magnitude = std::fabs(curvature.raw());
  if (magnitude <= kStraightCurvature || max_lateral_accel.raw() <= 0.0) {
    return max_velocity;
  }
  const QVelocity limit = QVelocity::fromBase(std::sqrt(max_lateral_accel.raw() / magnitude));
  return limit < max_velocity ? limit : max_velocity;
}

QVelocity approachSpeedLimit(QLength remaining, QAcceleration max_decel) {
  const double distance = std::fmax(remaining.raw(), 0.0);
  if (max_decel.raw() <= 0.0) {
    return QVelocity::fromBase(0.0);
  }
  return QVelocity::fromBase(std::sqrt(2.0 * max_decel.raw() * distance));
}

WheelSpeeds wheelSpeeds(QVelocity velocity, units::QCurvature curvature, QLength track_width) {
  // curvature * track_width is dimensionless: 1/length * length.
  const double spread = curvature.raw() * track_width.raw() / 2.0;
  // Positive curvature turns right, so the right wheel is the slow one.
  return WheelSpeeds{velocity * (1.0 + spread), velocity * (1.0 - spread)};
}

PurePursuit::PurePursuit(Path path, const PurePursuitConfig& config)
    : m_path(std::move(path)), m_config(config) {}

void PurePursuit::reset() {
  m_closest_index = 0;
  m_closest_t = 0.0;
  m_lookahead_index = 0;
  m_lookahead_t = 0.0;
  m_progress = QLength{};
}

void PurePursuit::setPath(Path path) {
  m_path = std::move(path);
  reset();
}

void PurePursuit::attachLogger(telemetry::Logger& logger, const char* prefix) {
  char name[48];
  m_logger = &logger;
  std::snprintf(name, sizeof(name), "%s_goal_x", prefix);
  m_channel_goal_x = logger.addLength(name);
  std::snprintf(name, sizeof(name), "%s_goal_y", prefix);
  m_channel_goal_y = logger.addLength(name);
  std::snprintf(name, sizeof(name), "%s_cross_track", prefix);
  m_channel_cross_track = logger.addLength(name);
  std::snprintf(name, sizeof(name), "%s_curvature", prefix);
  m_channel_curvature = logger.addNumber(name, "1/in");
  std::snprintf(name, sizeof(name), "%s_velocity", prefix);
  m_channel_velocity = logger.addVelocity(name);
}

void PurePursuit::publish(const PurePursuitOutput& output) const {
  if (m_logger == nullptr) {
    return;
  }
  m_logger->set(m_channel_goal_x, inches(output.lookahead_point.x()));
  m_logger->set(m_channel_goal_y, inches(output.lookahead_point.y()));
  m_logger->set(m_channel_cross_track, output.cross_track_error);
  // Curvature has no add*() helper; record it in 1/inch to match the header.
  m_logger->set(m_channel_curvature, output.curvature.raw() * 0.0254);
  m_logger->set(m_channel_velocity, output.velocity);
}

PurePursuit::Projection PurePursuit::closestPoint(const Vec2& position) const {
  Projection best;
  best.index = m_closest_index;
  best.t = m_closest_t;
  best.distance = m_path.at(m_closest_index).distance;
  best.point = m_path.at(m_closest_index).point();

  if (m_path.size() < 2) {
    best.error = inches((position - best.point).norm());
    return best;
  }

  if (m_closest_index + 1 < m_path.size()) {
    const Vec2 a = m_path[m_closest_index].point();
    const Vec2 b = m_path[m_closest_index + 1].point();
    best.point = a + m_closest_t * (b - a);
    best.distance = m_path[m_closest_index].distance +
                    (m_path[m_closest_index + 1].distance -
                     m_path[m_closest_index].distance) *
                        m_closest_t;
  }
  best.error = inches((position - best.point).norm());

  // Forward-only, and never more than search_window of arc length ahead. Both
  // halves matter: forward-only stops the cursor sliding back onto a part of
  // the path already driven, and the window stops a path that doubles back
  // from capturing the cursor on its return leg.
  const QLength limit = best.distance + m_config.search_window;
  for (std::size_t i = m_closest_index; i + 1 < m_path.size(); ++i) {
    if (m_path[i].distance > limit) {
      break;
    }
    const Vec2 a = m_path[i].point();
    const Vec2 b = m_path[i + 1].point();
    const Vec2 d = b - a;
    const double denominator = d.squaredNorm();
    // Within the segment the cursor is already on, the projection may not slide
    // backwards either.
    const double floor_t = (i == m_closest_index) ? m_closest_t : 0.0;
    double t = floor_t;
    if (denominator > 0.0) {
      t = clamp((position - a).dot(d) / denominator, floor_t, 1.0);
    }
    const Vec2 projected = a + t * d;
    const QLength error = inches((position - projected).norm());
    if (error < best.error) {
      best.index = i;
      best.t = t;
      best.point = projected;
      best.error = error;
      best.distance = m_path[i].distance + (m_path[i + 1].distance - m_path[i].distance) * t;
    }
  }
  return best;
}

Vec2 PurePursuit::findLookaheadPoint(const Vec2& position, const Projection& closest,
                                     bool& found) {
  found = false;

  // The lookahead cursor never trails the closest-point cursor. Without this,
  // a robot that skips ahead - a skid, or a re-plan - would chase a goal point
  // behind itself.
  if (closest.index > m_lookahead_index ||
      (closest.index == m_lookahead_index && closest.t > m_lookahead_t)) {
    m_lookahead_index = closest.index;
    m_lookahead_t = closest.t;
  }

  const double radius = m_config.lookahead.in();
  for (std::size_t i = m_lookahead_index; i + 1 < m_path.size(); ++i) {
    const double min_t = (i == m_lookahead_index) ? m_lookahead_t : 0.0;
    const Vec2 a = m_path[i].point();
    const Vec2 b = m_path[i + 1].point();
    double t = 0.0;
    if (segmentCircleIntersection(a, b, position, radius, min_t, t)) {
      m_lookahead_index = i;
      m_lookahead_t = t;
      found = true;
      return a + t * (b - a);
    }
  }
  return m_path.back().point();
}

PurePursuitOutput PurePursuit::update(const Pose2D& pose) {
  PurePursuitOutput output;
  if (!m_path.valid()) {
    output.finished = true;
    output.lookahead_point = Vec2{pose.x, pose.y};
    publish(output);
    return output;
  }

  const Vec2 position{pose.x, pose.y};

  const Projection closest = closestPoint(position);
  m_closest_index = closest.index;
  m_closest_t = closest.t;
  m_progress = closest.distance;

  output.distance_along = closest.distance;
  output.remaining = m_path.length() - closest.distance;
  output.path_error = closest.error;
  output.off_path = closest.error > m_config.lookahead;

  // Signed cross-track error in the *path's* frame: rotate the offset from the
  // path to the robot by the path's heading, and read the "right" component.
  // Positive therefore means the robot sits to the right of the path.
  const QAngle path_heading = m_path.atDistance(closest.distance).heading;
  const Vec2 offset = fieldToRobot(position - closest.point, path_heading.rad());
  output.cross_track_error = inches(offset.x());

  bool found = false;
  Vec2 goal = findLookaheadPoint(position, closest, found);
  if (!found) {
    if (output.off_path) {
      // No intersection because the robot is nowhere near the path. Aim one
      // lookahead further along than the closest point: a rejoin, not a lunge
      // at the endpoint.
      goal = m_path.atDistance(closest.distance + m_config.lookahead).point();
    } else {
      // No intersection because the circle hangs off the end of the path.
      output.at_end = true;
    }
  }
  output.lookahead_point = goal;

  // arcRadius() works in the same inches Pose2D and Vec2 use, so the reciprocal
  // has to be re-attached to an inch before it is a QCurvature.
  const double radius_in = arcRadius(pose, goal);
  units::QCurvature curvature{};
  if (std::isfinite(radius_in) && radius_in != 0.0) {
    curvature = 1.0 / inches(radius_in);
  }
  const double cap = std::fabs(m_config.max_curvature.raw());
  if (cap > 0.0 && std::fabs(curvature.raw()) > cap) {
    curvature = units::QCurvature::fromBase(std::copysign(cap, curvature.raw()));
  }
  output.curvature = curvature;

  // Finished means "arrived", not "ran out of path": a robot that has been
  // shoved off the route still has work to do even when its projection is at
  // the end, so off_path vetoes the finish.
  output.finished = !output.off_path && output.remaining <= m_config.finish_tolerance;

  // Speed: the tightest of the three limits. The curvature limit reads the
  // whole next lookahead of path, not just the point underfoot, so the robot
  // brakes before the corner instead of in it.
  const units::QCurvature window_curvature = m_path.maxAbsCurvature(
      closest.distance, closest.distance + m_config.lookahead);
  const double worst =
      std::fmax(std::fabs(curvature.raw()), std::fabs(window_curvature.raw()));
  QVelocity velocity = m_config.max_velocity;
  velocity = units::min(velocity, curvatureSpeedLimit(units::QCurvature::fromBase(worst),
                                                      m_config.max_velocity,
                                                      m_config.max_lateral_accel));
  velocity = units::min(velocity, approachSpeedLimit(output.remaining, m_config.max_decel));

  if (output.finished) {
    velocity = QVelocity{};
  } else {
    velocity = units::max(velocity, units::min(m_config.min_velocity, m_config.max_velocity));
  }

  WheelSpeeds wheels = wheelSpeeds(velocity, curvature, m_config.track_width);
  const double peak = std::fmax(std::fabs(wheels.left.raw()), std::fabs(wheels.right.raw()));
  if (peak > m_config.max_velocity.raw() && peak > 0.0) {
    const double scale = m_config.max_velocity.raw() / peak;
    wheels.left *= scale;
    wheels.right *= scale;
    velocity *= scale;
  }

  output.velocity = velocity;
  output.wheels = wheels;
  output.turn_rate = units::turnRate(velocity, curvature);

  publish(output);
  return output;
}

}  // namespace path
}  // namespace mclib
