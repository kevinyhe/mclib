// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/time.hpp"

#include "pros/rtos.hpp"

#include <cstdint>

namespace mclib {
namespace time {

std::uint32_t systemMillis() { return pros::millis(); }

}  // namespace time
}  // namespace mclib
