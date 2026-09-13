// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/**
 * @file sync.hpp
 * @brief The one mutex type mclib uses, and the only place that chooses it.
 *
 * On the V5 brain this is `pros::Mutex` - a FreeRTOS mutex with priority
 * inheritance. In a host build (`make test`, which defines
 * `MCLIB_HOST_BUILD`) it is `std::mutex`, so everything guarded by it stays
 * compilable and testable without the ARM toolchain or a PROS runtime.
 *
 * `pros::Mutex` satisfies the standard Lockable requirements, so
 * `std::lock_guard` and `std::unique_lock` work on both.
 *
 * The switch is a macro rather than `__has_include("pros/rtos.hpp")` on
 * purpose: the host test build puts `include/` on the search path, so the PROS
 * headers are always findable. Only the build system knows which target this
 * is.
 */

#include <mutex>

#if !defined(MCLIB_HOST_BUILD)
#include "pros/rtos.hpp"
#endif

namespace mclib {
namespace sync {

#if defined(MCLIB_HOST_BUILD)
/// @brief The library's mutex: std::mutex on a host build.
using Mutex = std::mutex;
#else
/// @brief The library's mutex: pros::Mutex on the V5 brain.
using Mutex = pros::Mutex;
#endif

/// @brief Scoped lock over mclib::sync::Mutex.
using LockGuard = std::lock_guard<Mutex>;

}  // namespace sync
}  // namespace mclib
