// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/units.hpp"

#include "api.h"

#include <cstdint>
#include <optional>

namespace mclib {
namespace device {

class Distance {
public:
  explicit Distance(std::uint8_t port);

  std::int32_t getDistanceMm() const;
  double getDistanceIn() const;
  std::int32_t getConfidence() const;

  /**
   * @brief Measured distance as a typed length, or nullopt when the port is
   *        not reporting a distance sensor.
   *
   * Units-typed sibling of getDistanceMm() / getDistanceIn(), built from the
   * sensor's native millimetres.
   *
   * The optional is not decoration. PROS returns PROS_ERR (INT32_MAX) for a
   * bad port, which getDistanceMm() hands straight back and getDistanceIn()
   * turns into 84.5 million inches - a plausible-looking number that no range
   * check catches, and get_confidence() fails the same way, so gating on
   * confidence does not save you. nullopt makes the caller face it.
   *
   * A live sensor with nothing in front of it is NOT an error: it reports
   * 9999 mm, and that comes back as a real length.
   */
  std::optional<units::QLength> distance() const;

private:
  mutable pros::Distance m_distance;
};

}  // namespace device
}  // namespace mclib
