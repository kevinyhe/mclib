// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/control/motion_config.hpp"

namespace mclib {
namespace control {

namespace {
// Function-local so there is no static-initialisation-order question between
// this and a ChassisController constructed at namespace scope, which writes
// it from its constructor.
MotionConfig& storage() {
  static MotionConfig config{};
  return config;
}
}  // namespace

const MotionConfig& motionConfig() {
  return storage();
}

void setMotionConfig(const MotionConfig& config) {
  storage() = config;
}

}  // namespace control
}  // namespace mclib
