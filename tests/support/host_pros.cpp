// mclib
//
// Host-side stand-ins for the PROS competition-status calls that
// CommandScheduler makes. On the robot these come out of libpros; a host test
// links this instead.
//
// Both report "connected to nothing, not disabled, not autonomous", which is
// the state a host test wants: schedule() refuses to schedule anything at all
// while is_disabled() is true, and run() polls the teleop event loop.
//
// It lives under tests/support/ rather than tests/ because `make test` globs
// tests/*.cpp into test binaries and this file has no main(). It is listed in
// HOST_TEST_SRC, so every test binary gets it.

#include "pros/misc.hpp"

#include <cstdint>

namespace pros {
namespace competition {

std::uint8_t get_status(void) { return 0; }
std::uint8_t is_autonomous(void) { return 0; }
std::uint8_t is_disabled(void) { return 0; }

}  // namespace competition
}  // namespace pros
