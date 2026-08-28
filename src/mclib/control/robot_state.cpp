// mclib
#include "mclib/control/robot_state.hpp"

#include <atomic>

namespace mclib {
namespace control {

Pose2D RobotState::pose() const {
  sync::LockGuard lock(m_mutex);
  return m_pose;
}

void RobotState::setPose(const Pose2D& pose) {
  sync::LockGuard lock(m_mutex);
  m_pose = Pose2D{pose.x, pose.y, wrapAngle(pose.theta)};
}

void RobotState::setPosition(double x_in, double y_in) {
  sync::LockGuard lock(m_mutex);
  m_pose.x = x_in;
  m_pose.y = y_in;
}



double RobotState::headingRad() const {
  sync::LockGuard lock(m_mutex);
  return m_pose.theta;
}



QAngle RobotState::heading() const {
  return headingRad() * units::radian;
}

bool RobotState::isTurning() const {
  sync::LockGuard lock(m_mutex);
  return m_is_turning;
}

void RobotState::setTurning(bool turning) {
  sync::LockGuard lock(m_mutex);
  m_is_turning = turning;
}

double RobotState::correctAngleDeg() const {
  sync::LockGuard lock(m_mutex);
  return m_correct_angle_deg;
}

void RobotState::setCorrectAngleDeg(double angle_deg) {
  sync::LockGuard lock(m_mutex);
  m_correct_angle_deg = angle_deg;
}

double RobotState::prevLeftOutput() const {
  sync::LockGuard lock(m_mutex);
  return m_prev_left_output;
}

double RobotState::prevRightOutput() const {
  sync::LockGuard lock(m_mutex);
  return m_prev_right_output;
}

void RobotState::setPrevOutputs(double left, double right) {
  sync::LockGuard lock(m_mutex);
  m_prev_left_output = left;
  m_prev_right_output = right;
}

void RobotState::clearMotionOutputs() {
  sync::LockGuard lock(m_mutex);
  m_is_turning = false;
  m_prev_left_output = 0.0;
  m_prev_right_output = 0.0;
}

RobotState& robotState() {
  static RobotState state;
  return state;
}

namespace {
/**
 * @brief One cancel flag per CancelToken.
 *
 * Plain bools would be a data race: they are set from the scheduler task and
 * read from the routine's own task. Constant-initialised, so there is no
 * static-initialisation order to get wrong.
 */
std::atomic_bool g_cancel_requested[2] = {{false}, {false}};

std::atomic_bool& cancelFlag(CancelToken token) {
  return g_cancel_requested[static_cast<int>(token)];
}
}  // namespace

bool cancelRequested(CancelToken token) {
  return cancelFlag(token).load(std::memory_order_acquire);
}

void requestCancel(CancelToken token) {
  cancelFlag(token).store(true, std::memory_order_release);
}

void clearCancel(CancelToken token) {
  cancelFlag(token).store(false, std::memory_order_release);
}

}  // namespace control
}  // namespace mclib
