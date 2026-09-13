// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// The host-side definition of mclib::time::systemMillis(). On the robot this
// symbol comes from src/mclib/time.cpp, the one translation unit that includes
// a PROS header; a host test cannot link that, so it links this instead.
//
// It lives under tests/support/ rather than tests/ because `make test` globs
// tests/*.cpp and turns every match into its own test binary, and this file has
// no main(). It is listed in HOST_TEST_SRC, so every test binary gets it.
//
// The value is 0: any test that cares about time installs its own clock with
// mclib::time::ScopedClock, and this is only what the seam reads before that
// happens.

#include "mclib/time.hpp"

#include <cstdint>

namespace mclib {
namespace time {

std::uint32_t systemMillis() { return 0; }

}  // namespace time
}  // namespace mclib
