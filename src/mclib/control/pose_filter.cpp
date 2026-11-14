// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/pose_filter.hpp"

#include "mclib/snapshot/solver.hpp"

#include "Eigen/Dense"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mclib {
namespace control {

namespace {

constexpr double kDeg = kPi / 180.0;

// What the sensor should read from @p pose, or NaN when the ray hits nothing
// in range.
double expectedRange(const snapshot::SensorGeometry& geometry, const Pose2D& pose,
                     std::uint32_t default_mask) {
  const snapshot::RayPrediction prediction = snapshot::predict_range(
      geometry, static_cast<float>(pose.x), static_cast<float>(pose.y),
      static_cast<float>(pose.theta / kDeg), default_mask);
  return prediction.valid ? static_cast<double>(prediction.expected_in)
                          : std::nan("");
}

// Joseph form: stays symmetric and positive with rounding error, where the
// short (I - KH) P form can drift.
Mat3 josephUpdate(const Mat3& P, const Eigen::MatrixXd& K, const Eigen::MatrixXd& H,
                  const Eigen::MatrixXd& R) {
  const Mat3 I_KH = Mat3::Identity() - K * H;
  return I_KH * P * I_KH.transpose() + K * R * K.transpose();
}

}  // namespace

PoseFilter::PoseFilter(PoseFilterConfig config) : m_config(config) { reset(Pose2D{}); }

void PoseFilter::reset(const Pose2D& pose, double position_sigma_in,
                       double heading_sigma_deg) {
  m_pose = Pose2D{pose.x, pose.y, wrapAngle(pose.theta)};
  m_cov = Mat3::Zero();
  m_cov(0, 0) = m_cov(1, 1) = position_sigma_in * position_sigma_in;
  m_cov(2, 2) = std::pow(heading_sigma_deg * kDeg, 2);
  m_accepted = m_rejected = m_too_old = 0;
  m_history.clear();
}

namespace {
// The odometry step in the robot frame it started from: forward along the
// heading, right across it (compass frame: forward is (sin, cos), right is
// (cos, -sin)).
template <class Step>
Step odometryStep(const Pose2D& previous_odom, const Pose2D& current_odom) {
  const double dx = current_odom.x - previous_odom.x;
  const double dy = current_odom.y - previous_odom.y;
  const double s0 = std::sin(previous_odom.theta), c0 = std::cos(previous_odom.theta);
  return {dx * s0 + dy * c0, dx * c0 - dy * s0,
          wrapAngle(current_odom.theta - previous_odom.theta)};
}

bool notAfter(std::uint32_t a_ms, std::uint32_t b_ms) {
  return static_cast<std::int32_t>(a_ms - b_ms) <= 0;
}
}  // namespace

void PoseFilter::predict(const Pose2D& previous_odom, const Pose2D& current_odom) {
  m_history.clear();
  applyStep(odometryStep<Step>(previous_odom, current_odom));
}

void PoseFilter::predict(const Pose2D& previous_odom, const Pose2D& current_odom,
                         std::uint32_t now_ms) {
  const Step step = odometryStep<Step>(previous_odom, current_odom);
  applyStep(step);
  m_history.push_back(Tick{now_ms, step, {}, m_pose, m_cov});
  // Keep the newest step that is history_ms old or older: a reading taken
  // history_ms ago lands on it.
  while (m_history.size() > 1 &&
         static_cast<std::uint32_t>(now_ms - m_history[1].time_ms) >= m_config.history_ms) {
    m_history.pop_front();
  }
}

void PoseFilter::applyStep(const Step& step) {
  const double forward = step.forward, right = step.right, turn = step.turn;

  // Apply the same step from the filter's own pose.
  const double s = std::sin(m_pose.theta), c = std::cos(m_pose.theta);
  m_pose.x += forward * s + right * c;
  m_pose.y += forward * c - right * s;
  m_pose.theta = wrapAngle(m_pose.theta + turn);

  Mat3 F = Mat3::Identity();
  F(0, 2) = forward * c - right * s;
  F(1, 2) = -forward * s - right * c;

  // Noise grows with how far the robot went and how much it turned, split
  // along and across the heading, then rotated into the field frame.
  const double travelled = std::hypot(forward, right);
  const double along_var = m_config.odom_along_var_per_in * travelled;
  const double side_var = m_config.odom_side_var_per_in * travelled;
  Mat2 rotate;
  rotate << s, c, c, -s;  // columns: forward, right in field (x, y)
  Mat2 local = Mat2::Zero();
  local(0, 0) = along_var;
  local(1, 1) = side_var;
  Mat3 Q = Mat3::Zero();
  Q.topLeftCorner<2, 2>() = rotate * local * rotate.transpose();
  Q(2, 2) = (m_config.odom_heading_var_per_deg * std::fabs(turn / kDeg) +
             m_config.odom_heading_var_per_in * travelled) * kDeg * kDeg;

  m_cov = F * m_cov * F.transpose() + Q;
}

bool PoseFilter::updateDistance(const snapshot::SensorGeometry& geometry, double measured_in) {
  return updateNow(DistanceReading{geometry, measured_in});
}

bool PoseFilter::updateDistance(const snapshot::SensorGeometry& geometry, double measured_in,
                                std::uint32_t taken_ms) {
  return updateAt(DistanceReading{geometry, measured_in}, taken_ms);
}

bool PoseFilter::updateGps(const Pose2D& measured, double position_sigma_in,
                           double heading_sigma_deg) {
  return updateNow(GpsReading{measured, position_sigma_in, heading_sigma_deg});
}

bool PoseFilter::updateGps(const Pose2D& measured, double position_sigma_in,
                           double heading_sigma_deg, std::uint32_t taken_ms) {
  return updateAt(GpsReading{measured, position_sigma_in, heading_sigma_deg}, taken_ms);
}

bool PoseFilter::updateNow(const Reading& reading) {
  const Outcome outcome = applyReading(reading);
  if (outcome == Outcome::Rejected) ++m_rejected;
  if (outcome != Outcome::Used) return false;
  ++m_accepted;
  if (!m_history.empty()) {
    Tick& latest = m_history.back();
    latest.readings.push_back(reading);
    latest.pose = m_pose;
    latest.cov = m_cov;
  }
  return true;
}

bool PoseFilter::updateAt(const Reading& reading, std::uint32_t taken_ms) {
  if (m_history.empty() || notAfter(m_history.back().time_ms, taken_ms)) {
    return updateNow(reading);
  }
  if (static_cast<std::uint32_t>(m_history.back().time_ms - taken_ms) > m_config.history_ms) {
    ++m_too_old;
    return false;
  }
  std::size_t at = m_history.size();
  for (std::size_t i = m_history.size(); i-- > 0;) {
    if (notAfter(m_history[i].time_ms, taken_ms)) {
      at = i;
      break;
    }
  }
  if (at == m_history.size()) {
    ++m_too_old;
    return false;
  }

  const Pose2D pose_now = m_pose;
  const Mat3 cov_now = m_cov;
  m_pose = m_history[at].pose;
  m_cov = m_history[at].cov;
  const Outcome outcome = applyReading(reading);
  if (outcome != Outcome::Used) {
    m_pose = pose_now;
    m_cov = cov_now;
    if (outcome == Outcome::Rejected) ++m_rejected;
    return false;
  }
  ++m_accepted;
  m_history[at].readings.push_back(reading);
  m_history[at].pose = m_pose;
  m_history[at].cov = m_cov;
  for (std::size_t i = at + 1; i < m_history.size(); ++i) {
    Tick& tick = m_history[i];
    applyStep(tick.step);
    for (const Reading& later : tick.readings) applyReading(later);
    tick.pose = m_pose;
    tick.cov = m_cov;
  }
  return true;
}

PoseFilter::Outcome PoseFilter::applyReading(const Reading& reading) {
  if (const auto* distance = std::get_if<DistanceReading>(&reading)) {
    return applyDistance(*distance);
  }
  return applyGps(std::get<GpsReading>(reading));
}

PoseFilter::Outcome PoseFilter::applyDistance(const DistanceReading& reading) {
  const snapshot::SensorGeometry& geometry = reading.geometry;
  const double measured_in = reading.measured_in;
  if (!(measured_in > 0.0) || !std::isfinite(measured_in)) return Outcome::Ignored;
  const double expected = expectedRange(geometry, m_pose, m_config.field_mask);
  if (!std::isfinite(expected)) return Outcome::Ignored;

  // Jacobian by central differences through the same raycast, so there is
  // one copy of the sensor geometry, not two.
  Eigen::RowVector3d H;
  const double steps[3] = {0.05, 0.05, 0.1 * kDeg};
  for (int i = 0; i < 3; ++i) {
    Pose2D plus = m_pose, minus = m_pose;
    (i == 0 ? plus.x : i == 1 ? plus.y : plus.theta) += steps[i];
    (i == 0 ? minus.x : i == 1 ? minus.y : minus.theta) -= steps[i];
    const double hp = expectedRange(geometry, plus, m_config.field_mask);
    const double hm = expectedRange(geometry, minus, m_config.field_mask);
    // Near a corner the two sides can hit different walls or none; the
    // derivative there means nothing.
    if (!std::isfinite(hp) || !std::isfinite(hm)) return Outcome::Ignored;
    H(i) = (hp - hm) / (2 * steps[i]);
  }

  const double sigma = std::max(m_config.distance_fraction * measured_in, m_config.distance_min_in);
  const double R = sigma * sigma;
  const double innovation = measured_in - expected;
  const double S = (H * m_cov * H.transpose())(0, 0) + R;
  if (innovation * innovation > m_config.gate * m_config.gate * S) {
    // Most likely something other than a wall is in front of the sensor.
    return Outcome::Rejected;
  }

  const Eigen::Vector3d K = m_cov * H.transpose() / S;
  m_pose.x += K(0) * innovation;
  m_pose.y += K(1) * innovation;
  m_pose.theta = wrapAngle(m_pose.theta + K(2) * innovation);
  m_cov = josephUpdate(m_cov, K, H, Eigen::MatrixXd::Constant(1, 1, R));
  return Outcome::Used;
}

PoseFilter::Outcome PoseFilter::applyGps(const GpsReading& reading) {
  const Pose2D& measured = reading.measured;
  const double position_sigma_in = reading.position_sigma_in;
  const double heading_sigma_deg = reading.heading_sigma_deg;
  if (!std::isfinite(measured.x) || !std::isfinite(measured.y) ||
      !(position_sigma_in > 0.0) || !std::isfinite(position_sigma_in)) {
    return Outcome::Ignored;
  }
  const bool use_heading = heading_sigma_deg > 0.0 && std::isfinite(heading_sigma_deg) &&
                           std::isfinite(measured.theta);
  const int n = use_heading ? 3 : 2;
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n, 3);
  Eigen::VectorXd innovation(n);
  innovation(0) = measured.x - m_pose.x;
  innovation(1) = measured.y - m_pose.y;
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
  R(0, 0) = R(1, 1) = position_sigma_in * position_sigma_in;
  if (use_heading) {
    innovation(2) = wrapAngle(measured.theta - m_pose.theta);
    R(2, 2) = std::pow(heading_sigma_deg * kDeg, 2);
  }

  const Eigen::MatrixXd S = H * m_cov * H.transpose() + R;
  const Eigen::MatrixXd S_inv = S.inverse();
  const double distance2 = innovation.dot(S_inv * innovation);
  if (distance2 > m_config.gate * m_config.gate * n) {
    return Outcome::Rejected;
  }

  const Eigen::MatrixXd K = m_cov * H.transpose() * S_inv;
  const Eigen::Vector3d step = K * innovation;
  m_pose.x += step(0);
  m_pose.y += step(1);
  m_pose.theta = wrapAngle(m_pose.theta + step(2));
  m_cov = josephUpdate(m_cov, K, H, R);
  return Outcome::Used;
}

PoseFusion::PoseFusion(PoseFusionConfig config, std::vector<DistanceSensorInput> sensors)
    : m_config(config),
      m_sensors(std::move(sensors)),
      m_last_read_ms(m_sensors.size(), 0),
      m_filter(config.filter),
      m_config_error(std::any_of(m_sensors.begin(), m_sensors.end(),
                                 [&config](const DistanceSensorInput& sensor) {
                                   return sensor.latency_ms > config.filter.history_ms;
                                 })) {}

bool PoseFusion::setGps(GpsInput gps) {
  if (gps.latency_ms > m_config.filter.history_ms) return false;
  sync::LockGuard lock(m_mutex);
  m_gps = std::move(gps);
  m_last_gps_ms = 0;
  return true;
}

Pose2D PoseFusion::toField(const Pose2D& odometry) const {
  return {odometry.x + m_config.field_offset_x_in, odometry.y + m_config.field_offset_y_in,
          odometry.theta};
}

void PoseFusion::reset(const Pose2D& odometry_pose, double position_sigma_in,
                       double heading_sigma_deg) {
  sync::LockGuard lock(m_mutex);
  m_previous = toField(odometry_pose);
  m_filter.reset(m_previous, position_sigma_in, heading_sigma_deg);
  std::fill(m_last_read_ms.begin(), m_last_read_ms.end(), 0);
  m_last_gps_ms = 0;
  m_started = true;
}

std::optional<Vec2> PoseFusion::step(const Pose2D& odometry_pose, std::uint32_t now_ms) {
  if (m_config_error) return std::nullopt;
  if (!std::isfinite(odometry_pose.x) || !std::isfinite(odometry_pose.y) ||
      !std::isfinite(odometry_pose.theta)) {
    return std::nullopt;
  }
  if (!m_started) reset(odometry_pose);
  sync::LockGuard lock(m_mutex);
  const Pose2D now = toField(odometry_pose);
  // How far odometry moved this tick, from where the last correction left it.
  const double moved = std::hypot(now.x - m_previous.x, now.y - m_previous.y);
  m_filter.predict(m_previous, now, now_ms);
  for (std::size_t i = 0; i < m_sensors.size(); ++i) {
    const DistanceSensorInput& sensor = m_sensors[i];
    if (!sensor.read_in) continue;
    if (static_cast<std::uint32_t>(now_ms - m_last_read_ms[i]) < sensor.period_ms) continue;
    m_last_read_ms[i] = now_ms;
    m_filter.updateDistance(sensor.geometry, sensor.read_in(), now_ms - sensor.latency_ms);
  }
  if (m_gps && m_gps->read &&
      static_cast<std::uint32_t>(now_ms - m_last_gps_ms) >= m_gps->period_ms) {
    m_last_gps_ms = now_ms;
    if (const std::optional<Pose2D> gps = m_gps->read()) {
      double sigma = m_gps->position_sigma_in;
      if (m_gps->error_in) {
        const double reported = m_gps->error_in();
        if (std::isfinite(reported) && reported > sigma) sigma = reported;
      }
      m_filter.updateGps({gps->x + m_gps->origin_x_in, gps->y + m_gps->origin_y_in, gps->theta},
                         sigma, -1.0, now_ms - m_gps->latency_ms);
    }
  }

  const Pose2D fused = m_filter.pose();
  const double ex = fused.x - now.x, ey = fused.y - now.y;
  const double gap = std::hypot(ex, ey);
  m_previous = now;
  if (gap < 1e-9) return std::nullopt;
  const double limit = m_config.correction_per_in * moved + m_config.correction_floor_in;
  const double k = gap > limit ? limit / gap : 1.0;
  m_previous.x += k * ex;
  m_previous.y += k * ey;
  return Vec2{m_previous.x - m_config.field_offset_x_in,
              m_previous.y - m_config.field_offset_y_in};
}

Pose2D PoseFusion::pose() const {
  sync::LockGuard lock(m_mutex);
  const Pose2D p = m_filter.pose();
  return {p.x - m_config.field_offset_x_in, p.y - m_config.field_offset_y_in, p.theta};
}

int PoseFusion::accepted() const {
  sync::LockGuard lock(m_mutex);
  return m_filter.accepted();
}

int PoseFusion::rejected() const {
  sync::LockGuard lock(m_mutex);
  return m_filter.rejected();
}

int PoseFusion::tooOld() const {
  sync::LockGuard lock(m_mutex);
  return m_filter.tooOld();
}

double PoseFusion::positionSigmaIn() const {
  sync::LockGuard lock(m_mutex);
  return m_filter.positionSigmaIn();
}

double PoseFilter::positionSigmaIn() const {
  const Eigen::SelfAdjointEigenSolver<Mat2> solver(m_cov.topLeftCorner<2, 2>());
  return std::sqrt(std::max(0.0, solver.eigenvalues().maxCoeff()));
}

}  // namespace control
}  // namespace mclib
