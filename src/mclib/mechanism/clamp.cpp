// mclib
#include "mclib/mechanism/clamp.hpp"

#include "pros/rtos.hpp"

namespace mclib {
namespace mechanism {

namespace {
/** Highest confidence a V5 distance sensor reports. Anything above is an error
 * code, not a reading. */
constexpr std::int32_t kMaxConfidence = 63;
/** Reading PROS returns when the sensor cannot see any object. */
constexpr std::int32_t kNoObjectMm = 9999;
}  // namespace

Clamp::Clamp(const ClampConfig& config)
    : StateMechanism<bool>(config.default_state),
      m_config(config),
      // The solenoid is driven as a raw pass-through (extended_state = true on
      // the device) and this class owns the polarity, so the logical state the
      // mechanism caches is never the inverted pin value.
      m_pneumatic(std::make_unique<device::Pneumatic>(
          config.adi_port,
          config.default_state == config.extended_state,
          true)),
      m_distance(config.distance_port > 0
                     ? std::make_unique<device::Distance>(
                           static_cast<std::uint8_t>(config.distance_port))
                     : nullptr),
      m_auto_clamp_enabled(config.auto_clamp_enabled) {}

void Clamp::clamp() {
  m_release_latched = false;
  setState(true);
}

void Clamp::release() {
  // A manual release latches auto-clamp off, otherwise the next periodic()
  // would see the same goal still in range and grab it right back.
  m_release_latched = true;
  setState(false);
}

void Clamp::toggle() {
  if (isClamped()) {
    release();
  } else {
    clamp();
  }
}

bool Clamp::isClamped() const {
  return getState();
}

void Clamp::setAutoClamp(bool enabled) {
  m_auto_clamp_enabled = enabled;
  m_release_latched = false;
}

bool Clamp::isAutoClampEnabled() const {
  return m_auto_clamp_enabled;
}

double Clamp::distanceMm() const {
  if (!m_distance) {
    return -1.0;
  }

  const std::int32_t reading = m_distance->getDistanceMm();
  if (reading < 0) {
    // PROS_ERR: unplugged or misconfigured port.
    return -1.0;
  }
  if (reading >= kNoObjectMm) {
    // Nothing in range at all. The confidence gate says nothing useful here,
    // and this reading has to get through so a released clamp can re-arm.
    return static_cast<double>(kNoObjectMm);
  }

  // PROS pins confidence at 10 for anything closer than 200mm, so this gate
  // only ever rejects a genuinely noisy mid-range reading.
  const std::int32_t confidence = m_distance->getConfidence();
  if (confidence < m_config.min_confidence || confidence > kMaxConfidence) {
    return -1.0;
  }

  // 0 is a real reading: the object is inside the sensor's minimum range,
  // which is exactly where a goal sits as the clamp closes.
  return static_cast<double>(reading);
}

void Clamp::periodic() {
  updateAutoClamp();
  StateMechanism<bool>::periodic();
}

std::unique_ptr<Command> Clamp::makeClampCommand() {
  return runOnce([this]() { clamp(); });
}

std::unique_ptr<Command> Clamp::makeReleaseCommand() {
  return runOnce([this]() { release(); });
}

std::unique_ptr<Command> Clamp::makeToggleCommand() {
  return runOnce([this]() { toggle(); });
}

std::unique_ptr<Command> Clamp::makeAutoClampCommand(double timeout_ms) {
  auto start_time = std::make_shared<double>(0.0);
  auto prior_enabled = std::make_shared<bool>(false);

  return std::make_unique<FunctionalCommand>(
      [this, start_time, prior_enabled]() {
        *prior_enabled = m_auto_clamp_enabled;
        setAutoClamp(true);
        *start_time = static_cast<double>(pros::millis());
      },
      [this]() { updateAutoClamp(); },
      [this, prior_enabled](bool) {
        // Restore the arming the caller had, but only if nothing else changed
        // it while the command ran, and without touching the release latch.
        if (m_auto_clamp_enabled) {
          m_auto_clamp_enabled = *prior_enabled;
        }
      },
      [this, start_time, timeout_ms]() {
        const bool timed_out =
            timeout_ms > 0.0 &&
            static_cast<double>(pros::millis()) - *start_time >= timeout_ms;
        return isClamped() || timed_out;
      },
      std::initializer_list<Subsystem*>{this});
}

void Clamp::applyState(const bool& clamped) {
  m_pneumatic->set_value(solenoidValueFor(clamped));
}

bool Clamp::solenoidValueFor(bool clamped) const {
  return clamped == m_config.extended_state;
}

void Clamp::updateAutoClamp() {
  if (!m_distance) {
    return;
  }

  const double reading = distanceMm();
  if (reading < 0.0) {
    return;
  }

  if (reading >= m_config.auto_clamp_threshold_mm) {
    // Nothing in range any more, so the goal the driver released is gone.
    m_release_latched = false;
    return;
  }

  if (!m_auto_clamp_enabled || m_release_latched || isClamped()) {
    return;
  }

  setState(true);
}

}  // namespace mechanism
}  // namespace mclib
