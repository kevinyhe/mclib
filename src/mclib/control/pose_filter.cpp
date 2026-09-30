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
  m_accepted = m_rejected = 0;
}

void PoseFilter::predict(const Pose2D& previous_odom, const Pose2D& current_odom) {
  // The odometry step in the robot frame it started from: forward along the
  // heading, right across it (compass frame: forward is (sin, cos), right
  // is (cos, -sin)).
  const double dx = current_odom.x - previous_odom.x;
  const double dy = current_odom.y - previous_odom.y;
  const double s0 = std::sin(previous_odom.theta), c0 = std::cos(previous_odom.theta);
  const double forward = dx * s0 + dy * c0;
  const double right = dx * c0 - dy * s0;
  const double turn = wrapAngle(current_odom.theta - previous_odom.theta);

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
  if (!(measured_in > 0.0) || !std::isfinite(measured_in)) return false;
  const double expected = expectedRange(geometry, m_pose, m_config.field_mask);
  if (!std::isfinite(expected)) return false;

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
    if (!std::isfinite(hp) || !std::isfinite(hm)) return false;
    H(i) = (hp - hm) / (2 * steps[i]);
  }

  const double sigma = std::max(m_config.distance_fraction * measured_in, m_config.distance_min_in);
  const double R = sigma * sigma;
  const double innovation = measured_in - expected;
  const double S = (H * m_cov * H.transpose())(0, 0) + R;
  if (innovation * innovation > m_config.gate * m_config.gate * S) {
    // Most likely something other than a wall is in front of the sensor.
    ++m_rejected;
    return false;
  }

  const Eigen::Vector3d K = m_cov * H.transpose() / S;
  m_pose.x += K(0) * innovation;
  m_pose.y += K(1) * innovation;
  m_pose.theta = wrapAngle(m_pose.theta + K(2) * innovation);
  m_cov = josephUpdate(m_cov, K, H, Eigen::MatrixXd::Constant(1, 1, R));
  ++m_accepted;
  return true;
}

bool PoseFilter::updateGps(const Pose2D& measured, double position_sigma_in,
                           double heading_sigma_deg) {
  if (!std::isfinite(measured.x) || !std::isfinite(measured.y) ||
      !(position_sigma_in > 0.0) || !std::isfinite(position_sigma_in)) {
    return false;
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
    ++m_rejected;
    return false;
  }

  const Eigen::MatrixXd K = m_cov * H.transpose() * S_inv;
  const Eigen::Vector3d step = K * innovation;
  m_pose.x += step(0);
  m_pose.y += step(1);
  m_pose.theta = wrapAngle(m_pose.theta + step(2));
  m_cov = josephUpdate(m_cov, K, H, R);
  ++m_accepted;
  return true;
}

double PoseFilter::positionSigmaIn() const {
  const Eigen::SelfAdjointEigenSolver<Mat2> solver(m_cov.topLeftCorner<2, 2>());
  return std::sqrt(std::max(0.0, solver.eigenvalues().maxCoeff()));
}

}  // namespace control
}  // namespace mclib
