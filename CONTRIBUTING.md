# Contributing

Issues and pull requests are welcome.

## Design rules

- **Field frame.** Heading 0 is +Y, clockwise positive. See
  [docs/units.md](docs/units.md).
- **Units.** Public APIs take unit types (`QLength`, `QAngle`, ...), not bare
  `double`s.
- **Host-testable code.** Algorithms do not include PROS headers, so `make test`
  can build them. PROS-specific code (tasks, devices, screen) stays in thin
  wrappers.
- **Command ownership.** The scheduler stores plain pointers and never deletes
  commands. See [docs/commands.md](docs/commands.md).

## Tests

```sh
make -j test
```

Each test is one file in `tests/` with its own `main()` returning 0 on success.
Use the `CHECK` macros from `tests/test_assert.hpp`.

Run the sanitizers for changes that affect memory or object lifetimes:

```sh
make -j test \
  TESTBINDIR=bin/tests-asan \
  HOST_CXXFLAGS="-std=gnu++20 -Iinclude -Itests -DMCLIB_HOST_BUILD -Wall -Wextra -g -O1 -fsanitize=address,undefined -fno-sanitize=vptr" \
  HOST_LDFLAGS="-pthread -fsanitize=address,undefined -fno-sanitize=vptr"
```

## Firmware build

```sh
curl -sL https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz | tar xJ
export PATH="$PWD/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin:$PATH"
make -j
```

## Template

```sh
pip install pros-cli
make template
```

On Python 3.12 and newer, pros-cli 3.5.6 fails with "Could not determine
version". Fix it with:

```sh
python -c "import pros, os, importlib.metadata as m; open(os.path.join(os.path.dirname(pros.__file__), '..', 'version'), 'w').write(m.version('pros-cli'))"
```

## Style

- Start each source file with the `// mclib` line and the MPL notice from an
  existing file.
- Comments explain why the code does something.
- Document public APIs with Doxygen: `@brief`, `@details`, `@param`, `@return`.
- Follow the style of the surrounding file. `command/` uses tabs and WPILib
  naming; the rest uses two spaces.

## Pull requests

1. Branch from `main`.
2. Add a test that fails without your change.
3. Update `docs/`.
4. Add an entry under "Unreleased" in [CHANGELOG.md](CHANGELOG.md).
5. CI must pass: host tests, sanitizers, firmware build and template.

## Releases

1. Update `VERSION` in the `Makefile`.
2. Move "Unreleased" entries under the new version.
3. Push a `vX.Y.Z` tag. The release workflow builds the template and publishes
   it on GitHub.

## License

Contributions are licensed under [MPL-2.0](LICENSE).
