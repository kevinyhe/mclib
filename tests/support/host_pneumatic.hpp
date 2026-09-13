// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <map>

/**
 * @brief The pin levels the host stand-in Pneumatic drives. See
 *        tests/support/host_pneumatic.cpp.
 */
namespace mclib {
namespace test {
namespace host_pneumatic {
/// @brief Solenoid pin level per ADI port, as the stand-in Pneumatic drives it.
extern std::map<char, bool> pin;
/// @brief How many times each port's pin was written, construction included.
extern std::map<char, int> pin_writes;
/// @brief Forget every pin. Call at the start of each test case.
void reset();
}  // namespace host_pneumatic
}  // namespace test
}  // namespace mclib
