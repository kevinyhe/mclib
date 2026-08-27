// mclib
#pragma once

/**
 * @brief Minimal host-side assertion helpers for `make test`.
 *
 * There is no test framework here on purpose. A test is a single `.cpp` file
 * under `tests/` with its own `int main()` that returns 0 on success and
 * non-zero on failure. Include this header, use the CHECK macros, and end
 * `main` with `return mclib::test::summary("name of this test");`.
 *
 * Every macro records the failure and keeps going, so one run reports every
 * broken assertion instead of only the first. Failures print file, line, the
 * expression, and the actual values.
 */

#include <cmath>
#include <cstdio>
#include <string>

namespace mclib::test {

/** @brief Number of failed assertions recorded so far in this binary. */
inline int& failureCount() {
  static int count = 0;
  return count;
}

/** @brief Number of assertions attempted so far in this binary. */
inline int& checkCount() {
  static int count = 0;
  return count;
}

/** @brief Format a double with enough digits to see a real mismatch. */
inline std::string show(double value) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.10g", value);
  return std::string(buf);
}

/** @brief Record one boolean assertion. Returns true when it passed. */
inline bool recordBool(bool ok, const char* expr, const char* file, int line) {
  ++checkCount();
  if (!ok) {
    ++failureCount();
    std::printf("  FAIL %s:%d: CHECK(%s) is false\n", file, line, expr);
  }
  return ok;
}

/** @brief Record one |a - b| <= eps assertion, printing both actual values. */
inline bool recordNear(double actual, double expected, double eps,
                       const char* actual_expr, const char* expected_expr,
                       const char* file, int line) {
  ++checkCount();
  // Exact equality first so an expected +/-inf can pass. NaN never passes.
  const bool ok = (actual == expected) ||
                  (std::isfinite(actual) && std::isfinite(expected) &&
                   std::fabs(actual - expected) <= eps);
  if (!ok) {
    ++failureCount();
    std::printf(
        "  FAIL %s:%d: CHECK_NEAR(%s, %s, %s)\n"
        "       expected %s, got %s (diff %s)\n",
        file, line, actual_expr, expected_expr, show(eps).c_str(),
        show(expected).c_str(), show(actual).c_str(),
        show(actual - expected).c_str());
  }
  return ok;
}

/** @brief Record one exact equality assertion on doubles. */
inline bool recordEq(double actual, double expected, const char* actual_expr,
                     const char* expected_expr, const char* file, int line) {
  ++checkCount();
  const bool ok = (actual == expected);
  if (!ok) {
    ++failureCount();
    std::printf(
        "  FAIL %s:%d: CHECK_EQ(%s, %s)\n"
        "       expected %s, got %s\n",
        file, line, actual_expr, expected_expr, show(expected).c_str(),
        show(actual).c_str());
  }
  return ok;
}

/**
 * @brief Record a known bug without ever failing the run.
 *
 * Use this instead of a CHECK when a test documents behaviour that is wrong
 * but owned by someone else. Asserting the wrong value would turn the eventual
 * fix into a red build blamed on whoever fixed it. This prints either way and
 * never affects the exit code, so the note surfaces in the log and flips to
 * "appears fixed" on its own.
 *
 * @param still_present true when the buggy behaviour is still observed.
 * @param what One-line description of the bug.
 */
inline void knownBug(bool still_present, const char* what) {
  if (still_present) {
    std::printf("  KNOWN BUG (still present): %s\n", what);
  } else {
    std::printf(
        "  KNOWN BUG (appears FIXED, update this test): %s\n", what);
  }
}

/**
 * @brief Print a one-line result for the whole binary.
 * @return 0 when every assertion passed, 1 otherwise. Return this from `main`.
 */
inline int summary(const char* test_name) {
  const int failures = failureCount();
  const int checks = checkCount();
  if (failures == 0) {
    std::printf("PASS %s (%d checks)\n", test_name, checks);
    return 0;
  }
  std::printf("FAIL %s (%d of %d checks failed)\n", test_name, failures,
              checks);
  return 1;
}

}  // namespace mclib::test

/** @brief Assert a boolean expression is true. */
#define CHECK(cond) \
  ::mclib::test::recordBool((cond), #cond, __FILE__, __LINE__)

/** @brief Assert |actual - expected| <= eps, printing both on failure. */
#define CHECK_NEAR(actual, expected, eps)                                    \
  ::mclib::test::recordNear((actual), (expected), (eps), #actual, #expected, \
                            __FILE__, __LINE__)

/** @brief Assert two doubles are exactly equal. */
#define CHECK_EQ(actual, expected)                                  \
  ::mclib::test::recordEq((actual), (expected), #actual, #expected, \
                          __FILE__, __LINE__)
