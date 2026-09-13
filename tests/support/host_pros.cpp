// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Host-side stand-ins for the PROS competition-status calls that
// CommandScheduler makes. On the robot these come out of libpros; a host test
// links this instead.
//
// Defaults to enabled teleop with no field connection. Tests may change the
// status through setCompetitionStatus() to exercise mode transitions.
//
// It lives under tests/support/ rather than tests/ because `make test` globs
// tests/*.cpp into test binaries and this file has no main(). It is listed in
// HOST_TEST_SRC, so every test binary gets it.

#include "pros/misc.hpp"

#include <cstdint>
#include "support/host_pros.hpp"

namespace {
std::uint8_t g_host_competition_flags = 0;
}
namespace mclib::test {
void setCompetitionStatus(std::uint8_t status) { g_host_competition_flags = status; }
}

namespace pros {
namespace competition {

std::uint8_t get_status(void) { return g_host_competition_flags; }
std::uint8_t is_autonomous(void) { return (g_host_competition_flags & 2) != 0; }
std::uint8_t is_disabled(void) { return (g_host_competition_flags & 1) != 0; }

}  // namespace competition
}  // namespace pros
