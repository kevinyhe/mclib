# Installation

## Install the template

Requires the PROS CLI (`pip install pros-cli`) and a PROS 4 project.

1. Download `mclib@<version>.zip` from the
   [releases page](https://github.com/kevinyhe/mclib/releases). Keep the
   filename; `pros c fetch` reads the name and version from it.
2. In the project directory:

```sh
pros c fetch mclib@0.1.0.zip
pros c apply mclib
```

```cpp
#include "mclib/mclib.hpp"
```

`fetch` adds the template to the shared PROS cache. `apply` copies it into the
current project and records it in `project.pros`.

| Task | Command |
| --- | --- |
| Pin a version | `pros c apply mclib@0.1.0` |
| Upgrade | fetch the new zip, then `pros c apply mclib --force-apply` |
| Remove | `pros c uninstall mclib` |

The template installs:

- `include/mclib/**` - headers
- `include/Eigen/**` - Eigen headers
- `firmware/mclib.a` - the library, with debug symbols

## Build from source

```sh
pros make            # build the project
pros make library    # build bin/mclib.a
pros make template   # package mclib@<version>.zip
```

## Eigen

Eigen is header-only. Either path works:

```text
include/Eigen/Core
```

```text
eigen/Eigen/Core
```

Do not add Eigen to the compiled sources.

## Layout

```text
mclib/
  firmware/
  include/
    Eigen/
    mclib/
      auton/
      chassis/
      command/
      device/
      mechanism/
      snapshot/
      path/
      telemetry/
      units/
      control/
      control.hpp
      math.hpp
      mclib.hpp
      pid.hpp
      robot_geometry.hpp
      utils.hpp
    main.h
  src/
    mclib/
      auton/
      chassis/
      device/
      mechanism/
      path/
      snapshot/
      telemetry/
      control/
      math.cpp
      pid.cpp
      utils.cpp
    main.cpp
  tests/
  Makefile
  common.mk
  project.pros
```

## Tests

```sh
make test
```

`make test` builds each `tests/*.cpp` into its own host binary with `g++` and
runs it. It does not use PROS or the ARM toolchain. Use `make -j8 test` to build
in parallel. Binaries go to `bin/tests/`.

Keep `tests/` outside `src/`. `common.mk` compiles everything under `src/` into
the library.

### Writing a test

One file per test, each with its own `main()`. New files are picked up
automatically.

```cpp
// mclib
#include "mclib/math.hpp"

#include "test_assert.hpp"

int main() {
  CHECK_NEAR(mclib::wrapAngle(3.0 * mclib::kPi), mclib::kPi, 1e-12);
  CHECK_EQ(mclib::clamp(11.0, 0.0, 10.0), 10.0);
  CHECK(mclib::clamp(5.0, 10.0, 0.0) == 5.0);
  return mclib::test::summary("my feature");
}
```

| Macro | Checks |
| --- | --- |
| `CHECK(cond)` | condition is true |
| `CHECK_NEAR(actual, expected, eps)` | within a tolerance |
| `CHECK_EQ(actual, expected)` | exact equality |

A failure prints the file, line, expression and both values, then continues:

```text
== utils_test
  FAIL ./tests/utils_test.cpp:44: CHECK_NEAR(getRadius(0.0, 0.0, 3.0, 4.0, 0.0), 6.4, 1e-12)
       expected 6.4, got 3.125 (diff -3.275)
FAIL utils (1 of 120 checks failed)

FAILED TESTS: utils_test
```

`mclib::test::summary()` returns 0 if every check passed.

### Known bugs

Record a known bug with `knownBug()` instead of checking for the wrong value. It
prints the bug's status and does not affect the exit code.

```cpp
const double near_degenerate = getRadius(0.0, 0.0, 0.0, 1.0, -90.0);
mclib::test::knownBug(
    std::isfinite(near_degenerate),
    "getRadius compares the denominator to exactly 0, so angle == -90 returns "
    "~1e15 instead of +infinity");
```

### Host-testable sources

`HOST_TEST_SRC` in the `Makefile` lists the sources that build without PROS:
math, PID, odometry, profiles, feedforward, paths, snapshot, mechanisms,
scheduler, telemetry and drive curves. Defining `MCLIB_HOST_BUILD` makes
`sync.hpp` use `std::mutex`. Add a source to `HOST_TEST_SRC` once it builds
without PROS headers.
