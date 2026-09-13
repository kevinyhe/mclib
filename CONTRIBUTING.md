# Contributing to mclib

Bug reports, questions and pull requests are all welcome.

## Before you start

Read [docs/README.md](docs/README.md). The library is built around a few fixed
ideas that a change has to respect:

- **The field frame is compass convention.** Heading 0 is +Y, clockwise
  positive. See [docs/units.md](docs/units.md).
- **Units are types.** A gain or a distance crossing a public API is a
  `units::` quantity, not a bare `double`. Handing `kV` a number is meant to be
  a compile error.
- **Algorithms stay PROS-free.** Anything that can be tested on a host machine
  lives in a file that does not include PROS headers, so `make test` can reach
  it. The PROS-shaped parts (tasks, motors, the screen) are thin wrappers.
- **Nothing deletes a command.** The scheduler holds raw `Command*`. See the
  ownership rules in [docs/commands.md](docs/commands.md).

## Building and testing

The host test suite needs only `g++`:

```sh
make -j test
```

Every test is one file under `tests/` with its own `int main()` that returns 0
on success. There is no framework and no registration: add a file, and
`make test` picks it up. Use the `CHECK` macros from `tests/test_assert.hpp`,
which record every failure instead of stopping at the first.

Run the sanitizers before sending a change that touches ownership or lifetimes:

```sh
make -j test \
  TESTBINDIR=bin/tests-asan \
  HOST_CXXFLAGS="-std=gnu++20 -Iinclude -Itests -DMCLIB_HOST_BUILD -Wall -Wextra -g -O1 -fsanitize=address,undefined -fno-sanitize=vptr" \
  HOST_LDFLAGS="-pthread -fsanitize=address,undefined -fno-sanitize=vptr"
```

`-fno-sanitize=vptr` is needed because UBSan's vptr check wants typeinfo that
the header-only command classes have no key function to emit.

### Building the firmware

You need the Arm GNU toolchain on your `PATH`:

```sh
curl -sL https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz | tar xJ
export PATH="$PWD/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin:$PATH"
make -j
```

### Building the template

`make template` needs the PROS CLI:

```sh
pip install pros-cli
```

pros-cli 3.5.6 cannot work out its own version on Python 3.12 and newer,
because `pkg_resources` is gone. Write the file it looks for first:

```sh
python -c "import pros, os, importlib.metadata as m; open(os.path.join(os.path.dirname(pros.__file__), '..', 'version'), 'w').write(m.version('pros-cli'))"
```

Then `make template` produces `mclib@<version>.zip`.

## Style

- Every source file starts with the `// mclib` marker and the MPL notice. Copy
  the header from any existing file.
- Comments explain why, not what. A comment that restates the line below it is
  noise; a comment recording the failure a line prevents is worth keeping.
- Doxygen comments on public API. `@brief`, then `@details` when the short form
  is not enough, then `@param`/`@return`.
- Match the surrounding file. The command framework uses tabs and WPILib-style
  naming because it started as a port; the rest of the library uses two spaces.

## Pull requests

1. Branch off `main`.
2. Keep `make test` green, and add a test for the behaviour you changed.
   A bug fix without a test that fails before it is not finished.
3. Update the docs under `docs/` in the same change.
4. Add an entry to [CHANGELOG.md](CHANGELOG.md) under "Unreleased".
5. CI runs the host tests, the sanitizer build, the ARM firmware build and the
   template package. All four have to pass.

## Releasing

1. Bump `VERSION` in the `Makefile`.
2. Move the "Unreleased" changelog entries under the new version with a date.
3. Tag `vX.Y.Z` and push the tag. The release workflow checks that the tag
   matches `VERSION`, builds the template, and attaches the zip to a GitHub
   release.

## License

Contributions are licensed under the [MPL 2.0](LICENSE), the same as the rest
of the library.
