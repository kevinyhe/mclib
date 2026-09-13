# Installation and building

How to get mclib into a PROS project, what it depends on, and how to build it from source.

[Documentation index](README.md) · [Project README](../README.md)

## Install into a PROS project

mclib ships as a PROS template, the same way LemLib does. You need the PROS CLI
(`pip install pros-cli`) and a PROS 4 project.

1. Download `mclib@<version>.zip` from the
   [releases page](https://github.com/kevinyhe/mclib/releases).
2. From inside your project directory:

```sh
pros c fetch mclib@0.1.0.zip
pros c apply mclib
```

`fetch` puts the template in the local cache, which every project on the
machine shares. `apply` copies the headers and the compiled archive into one
project and records the dependency in its `project.pros`, so run `apply` in
each project that uses mclib.

Then include the umbrella header:

```cpp
#include "mclib/mclib.hpp"
```

To move to a newer mclib later, fetch the new zip and run `pros c apply mclib
--force-apply`. To remove it, `pros c uninstall mclib`.

### Version pinning

`pros c apply mclib@0.1.0` pins an exact version when several are cached. With
no version, the newest cached one wins. Pin it. You do not want to find out at a
competition that a library upgrade changed a default gain.

### What lands in your project

- `include/mclib/**` - the public headers.
- `include/Eigen/**` - the bundled Eigen headers the public API needs.
- `firmware/mclib.a` - the compiled library.

The archive is built with debug symbols, so it is large (roughly 21 MB) and the
template zip is larger still. The linker only pulls in the code you call, so
none of that size reaches the brain.

## Building

Build the PROS project:

```sh
pros make
```

Build the PROS library archive:

```sh
pros make library
```

Create a PROS template package:

```sh
pros make template
```

## Eigen

Eigen is header-only here. Keep the headers local so this path exists:

```text
include/Eigen/Core
```

You can also use this alternate layout:

```text
eigen/Eigen/Core
```

The `Makefile` already adds `eigen/` as an extra include directory. Do not add
Eigen files to the compiled source list.

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

`make test` compiles the host-buildable library sources once into objects,
links every `tests/*.cpp` into its own binary with the system `g++`, and runs
them all. `make -j8 test` builds in parallel; a full build takes about a
minute and an incremental one under a second. It never touches the ARM
toolchain, never links PROS, and never runs the firmware build. Binaries land
in `bin/tests/`, which is already gitignored, and `make clean` removes them.

`tests/` sits at the repo root, outside `src/`. `common.mk` globs
`src/**` recursively into the firmware, so a test directory under `src/` would
be compiled into the library. Do not move it, and do not add `tests/` to
`TEMPLATE_FILES`.

### Writing a test

One test per file, each with its own `int main()`. There is no framework, no
registration macro, and nothing to add to the `Makefile`. The glob picks up
any new `tests/*.cpp` on the next run.

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

`tests/test_assert.hpp` gives you three macros:

| Macro | Use |
| --- | --- |
| `CHECK(cond)` | boolean condition |
| `CHECK_NEAR(actual, expected, eps)` | floating point within a tolerance |
| `CHECK_EQ(actual, expected)` | exact equality on doubles |

Prefer the numeric ones. A failure prints the file, the line, the expression,
and both values:

```text
== utils_test
  FAIL ./tests/utils_test.cpp:44: CHECK_NEAR(getRadius(0.0, 0.0, 3.0, 4.0, 0.0), 6.4, 1e-12)
       expected 6.4, got 3.125 (diff -3.275)
FAIL utils (1 of 120 checks failed)

FAILED TESTS: utils_test
```

A failed assertion does not stop the run, so one invocation reports every
broken check. `mclib::test::summary()` returns 0 when everything passed and 1
otherwise; `make test` exits non-zero and names each failing binary.

### Documenting a bug you are not allowed to fix

Do not `CHECK` the wrong value. Pinning known-bad behaviour means the person
who eventually fixes it gets a red build blamed on their commit. Use
`mclib::test::knownBug(still_present, "what is wrong")` instead. It prints
either `KNOWN BUG (still present)` or `KNOWN BUG (appears FIXED, update this
test)` and never touches the exit code.

```cpp
const double near_degenerate = getRadius(0.0, 0.0, 0.0, 1.0, -90.0);
mclib::test::knownBug(
    std::isfinite(near_degenerate),
    "getRadius compares the denominator to exactly 0, so angle == -90 returns "
    "~1e15 instead of +infinity");
```

### What can be tested

Tests link against `HOST_TEST_SRC` in the `Makefile`, the library sources that
compile without PROS headers: the math, the PID, the odometry integrator, the
motion profile and feedforward, the path follower, the snapshot solver, the
mechanisms, the command scheduler, the telemetry logger and the drive curves.

`robot_state.cpp` and `odometry.cpp` were written to stay PROS-free. `sync.hpp`
picks `std::mutex` over `pros::Mutex` when `MCLIB_HOST_BUILD` is defined, and
`Odometry` takes a struct of raw sensor readings instead of reading devices
itself. The task that reads them lives in `control/odometry_task.cpp`, which is
not host-testable and holds no math.

This list is expected to grow. Anything that pulls in `pros/...` cannot be
linked on the host, so making a source testable usually means putting a seam in
front of the PROS call (a time source, a motor interface) rather than changing
the test setup. When a source becomes PROS-free, add it to `HOST_TEST_SRC` and
it is available to every test.
