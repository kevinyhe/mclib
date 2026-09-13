// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "mclib/units/units.hpp"

#include <cstdint>

namespace mclib {
namespace time {

/**
 * @brief Signature of a clock source: milliseconds since the program started.
 *
 * @details A plain function pointer, not std::function, because millis() runs
 * on every control tick and an indirect call through a raw pointer is as cheap
 * as the seam can be.
 */
using ClockFn = std::uint32_t (*)();

/**
 * @brief The platform clock, and the only place in the library that reads
 * pros::millis().
 *
 * @details Defined in src/mclib/time.cpp, which is the single translation unit
 * that includes a PROS header. It is deliberately a strong out-of-line symbol
 * rather than a pointer installed at static-initialisation time. That buys two
 * things:
 *
 * - Every translation unit that calls millis() has an undefined reference to
 *   this symbol, so a plain archive link is forced to pull time.cpp.o out of
 *   libmclib.a. A version that only registered itself from a global
 *   constructor could be dropped by the linker, leaving millis() stuck at 0
 *   and every timeout, settle window and PID arrival check dead.
 * - There is no static-initialisation order to get wrong. A mechanism
 *   constructed at namespace scope reads the real clock, exactly as it did
 *   when it called pros::millis() directly.
 *
 * A host build links its own definition instead - one line, returning whatever
 * the test wants for code paths that run before a fake clock is installed:
 *
 * @code
 * namespace mclib {
 * namespace time {
 * std::uint32_t systemMillis() { return 0; }
 * }  // namespace time
 * }  // namespace mclib
 * @endcode
 *
 * @return Milliseconds since the program started.
 */
std::uint32_t systemMillis();

namespace detail {

/**
 * @brief The clock installed by setClock(), or null to use systemMillis().
 */
inline ClockFn g_clock = nullptr;

}  // namespace detail

/**
 * @brief Milliseconds since the program started.
 *
 * @details Reads the installed clock, or the platform clock when no clock has
 * been installed. Equivalent to pros::millis() on the robot.
 *
 * @return Milliseconds elapsed.
 */
inline std::uint32_t millis() {
  const ClockFn clock = detail::g_clock;
  return clock != nullptr ? clock() : systemMillis();
}

/**
 * @brief Current time as a QTime.
 *
 * @return millis() converted to the units system's time type.
 */
inline QTime now() {
  return static_cast<double>(millis()) * millisecond;
}

/**
 * @brief Installs a clock source, replacing whatever was in use.
 *
 * @details Not synchronised. Install a clock before starting the tasks that
 * read it - on the robot the platform clock is in place from the first
 * instruction, and swapping clocks is meant for single-threaded host tests.
 *
 * @param clock The new clock, or nullptr to fall back to systemMillis().
 * @return The clock that was installed before this call, or nullptr if the
 *   platform clock was in use.
 */
inline ClockFn setClock(ClockFn clock) {
  const ClockFn previous = detail::g_clock;
  detail::g_clock = clock;
  return previous;
}

/**
 * @brief The clock installed by setClock().
 *
 * @return The installed clock, or nullptr when millis() is reading the
 *   platform clock.
 */
inline ClockFn getClock() { return detail::g_clock; }

/**
 * @brief Goes back to the platform clock, undoing a setClock() from a test.
 */
inline void restoreSystemClock() { detail::g_clock = nullptr; }

/**
 * @brief Installs a clock for the lifetime of the object, then restores the
 * one that was in use before.
 *
 * @details Intended for host tests:
 * @code
 * static std::uint32_t fake_ms = 0;
 * mclib::time::ScopedClock clock([]() { return fake_ms; });
 * fake_ms += 20;
 * @endcode
 */
class ScopedClock {
 public:
  explicit ScopedClock(ClockFn clock) : m_previous(setClock(clock)) {}

  ~ScopedClock() { setClock(m_previous); }

  ScopedClock(const ScopedClock&) = delete;
  ScopedClock& operator=(const ScopedClock&) = delete;
  ScopedClock(ScopedClock&&) = delete;
  ScopedClock& operator=(ScopedClock&&) = delete;

 private:
  ClockFn m_previous;
};

}  // namespace time
}  // namespace mclib
