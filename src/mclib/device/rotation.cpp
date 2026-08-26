// mclib
#include "mclib/device/rotation.hpp"

namespace mclib {
namespace device {

namespace {
std::int8_t signedPort(std::int8_t port, bool reversed) {
  const std::int8_t absolute = port < 0 ? -port : port;
  return reversed ? -absolute : absolute;
}
}  // namespace

Rotation::Rotation(std::int8_t port, bool reversed)
    : m_rotation(signedPort(port, reversed)) {}

double Rotation::getPositionDeg() const {
  return m_rotation.get_position() / 100.0;
}

void Rotation::resetPosition() {
  m_rotation.reset_position();
}

bool Rotation::isInstalled() const {
  return m_rotation.is_installed();
}

}  // namespace device
}  // namespace mclib
