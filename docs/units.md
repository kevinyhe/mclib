# Units and the coordinate frame

The compile-time unit types, and the field frame every pose and heading is expressed in.

[Documentation index](README.md) · [Project README](../README.md)

## Units (`units/units.hpp`)

Every dimensioned value in mclib is a `Quantity`: one `double` wrapped in a type
that carries five integer exponents - length, time, angle, voltage, current.
Multiplying adds the exponents, dividing subtracts them, so `QLength / QTime`
*is* `QVelocity` and `QLength + QTime` does not compile.

```cpp
QLength distance = 24_in;
QTime   timeout  = 1500_ms;
QAngle  heading  = 90_deg;

QVelocity speed = distance / timeout;   // QVelocity, checked at compile time
QLength   back  = speed * timeout;      // 24 in again
// QLength wrong = distance + timeout;  // error: no operator+
```

A `Quantity` is the same size as the `double` it replaces, all its operations
are `constexpr` and `inline`, and none of it survives to the ELF - the section
sizes of `bin/cold.package.elf` are unchanged by the migration.

It is still floating point underneath, so do not expect exact decimal
round-trips: `(24_in).in()` is `23.999999999999996`, because `24.0 * 0.0254` is
not representable. Compare with a tolerance, never with `==`.

### Aliases

`QNumber`, `QLength`, `QArea`, `QTime`, `QAngle`, `QVoltage`, `QCurrent`,
`QVelocity`, `QAcceleration`, `QJerk`, `QAngularVelocity`,
`QAngularAcceleration`, `QCurvature` (1/length), `QFrequency` (1/time).

Anything else you need is a valid type already: `QVoltage / QCurrent` is a
resistance, `square(QLength{}) ` is an area.

### Literals

`_m` `_cm` `_mm` `_in` `_ft` `_tile` - length (a `_tile` is 24 in)
`_s` `_ms` `_min` - time
`_rad` `_deg` `_rot` - angle
`_V` `_mV` `_A` `_mA` - voltage and current
`_mps` `_inps` `_mps2` `_radps` `_degps` `_rpm` - rates

Named constants exist for all of them too, for when the number is not a
literal: `metre`, `inch`, `tile`, `minute`, `degree`, `radian`, `rotation`,
`volt`, `millivolt`, `ampere`, `milliampere`, `rpm`, `percent`. These live in
`mclib::units` and are **not** global, so qualify them or pull the namespace in:

```cpp
using namespace mclib::units;
QLength target = 24 * inch;
```

The two exceptions are `millisecond` and `second`, which are global for
back-compat — `250 * millisecond` and `pros::millis() * millisecond` work
unqualified anywhere.

### Getting a raw double back out

Both directions of the escape hatch are explicit, because both are where unit
bugs come from.

```cpp
double ms = timeout.ms();       // named accessor - says what the number means
double in = distance.in();
double dg = heading.deg();
double mv = battery.mV();

QLength from_raw{0.6096};       // explicit ctor, value in SI base units
```

Accessors: `.m() .cm() .mm() .in() .ft()`, `.s() .ms()`, `.rad() .deg()`,
`.volts() .mV()`, `.amps() .mA()`, `.mps() .inps()`, `.radps() .degps() .rpm()`.
Each is constrained to its own dimension, so `timeout.in()` is a compile error
rather than a wrong number. Free-function spellings - `inches(x)`,
`milliseconds(t)`, `degrees(a)` - exist for call sites where they read better.
`.raw()` gives the stored value in SI base units and is the escape hatch of last
resort.

### Angles

`QAngle` stores **radians**. The public motion API of this library speaks
degrees and `Pose2D::theta` speaks radians; making the conversion a method call
(`.deg()` / `.rad()`) is the point. Angle is a real dimension, so
`radius * angle` does not type-check on its own - use the sanctioned crossings:

```cpp
QLength arc     = arcLength(radius, angle);      // radius * angle
QAngle  swept   = arcAngle(arc, radius);         // the inverse
QVelocity rim   = rimVelocity(radius, omega);    // radius * angular velocity
QAngularVelocity turn = turnRate(speed, curvature);   // speed * curvature
QCurvature k    = turnCurvature(speed, omega);        // the inverse
```

`turnRate` is the one pure pursuit needs: `speed * curvature` on its own is a
`QFrequency` — right arithmetic, wrong dimension.

`wrap(angle)` folds an angle into [-180 deg, +180 deg], using the same algorithm
and the same closed range as the pre-existing `mclib::wrapAngle(double)`,
including at exactly -180 deg. Two wrapping functions that disagreed on that
edge would be a trap for motion code migrating from `double` to `QAngle`.

### Math helpers

`abs`, `min`, `max` and `clamp` return the same dimension they were given.
`sign` returns a plain double (-1, 0 or +1) and `square` doubles the exponents.
`sin/cos/tan` take a `QAngle` and return a plain double; `asin/acos/atan` go the
other way; `atan2(QLength, QLength)` returns a `QAngle` and
`hypot(QLength, QLength)` a `QLength`.

### Namespaces and back-compat

Everything lives in `mclib::units`. For back-compat the type aliases, the
`millisecond` / `second` constants and all literal operators are also pulled
into the global namespace, so the pre-existing idiom keeps working unchanged:

```cpp
QTime now = pros::millis() * millisecond;
if (now - start >= 250 * millisecond) { /* ... */ }
```

The global surface is split in two. `QTime`, `millisecond` and `second` are
unconditional, because mclib's own headers use them unqualified at ~33 sites —
making those conditional would only mean the library stops compiling. Everything
else — the other 13 aliases and all the literal suffixes — is convenience, and
defining `MCLIB_NO_GLOBAL_UNITS` before including mclib switches it off.

That opt-out exists for a specific collision: okapilib declares the same
`QLength` / `QAngle` / `QArea` / `QJerk` / `QFrequency` / `QAcceleration` names
and the same `_in` / `_ft` / `_deg` / `_rad` / `_ms` / `_s` / `_rpm` suffixes, so
a project doing `using namespace okapi;` alongside mclib would get ambiguity on
all of them. With the macro defined, reach for `mclib::units::` instead.

The `Quantity` template itself is never exported globally — spell it
`mclib::units::Quantity` — because a downstream global `class Quantity` would
otherwise become ambiguous.

One deliberate hole in the type safety: `QNumber` (all exponents zero) converts
implicitly to and from `double`, because gains and gear ratios have to
interoperate with plain arithmetic. Every other dimension requires the explicit
constructor.

### Tests

`tests/units_test.cpp` is a standalone host program:

```sh
g++ -std=gnu++20 -Iinclude -o /tmp/units_test tests/units_test.cpp && /tmp/units_test
```

It covers dimension composition, literal values, round-tripping and the old
`QTime` millisecond semantics. The checks that dimensionally-wrong code does
*not* compile are written as concepts whose negation is asserted - if
`QLength + QTime` ever starts compiling, that file stops building.

## Coordinate Frame

mclib has one canonical frame, the **compass / field frame**:

- `theta = 0` points along **+Y**.
- `theta` increases **clockwise**, so **+90 deg points along +X**.
- The unit vector for a heading is `(sin(theta), cos(theta))` -- x uses sin, y
  uses cos.
- A bearing from A to B is `atan2(b.x - a.x, b.y - a.y)` -- x first, y second.

Both of those are the transpose of the usual textbook formulas. This is
deliberate: it matches how VEX field diagrams are drawn, and it is what
`control/odometry.cpp`, `control/motion.cpp` and `snapshot/raycast.cpp`
already do.

Units: angles are **radians** everywhere inside `math.hpp` (`Pose2D::theta`,
`wrapAngle`), and **degrees** at the public motion API (`Chassis`,
`control/motion.cpp`, `RobotState::correctAngleDeg()`). Convert at that boundary with
`degToRad` / `radToDeg` from `utils.hpp`. Translations are inches.

The robot frame is chosen to coincide with the field frame at `theta = 0`:
**+Y is forward, +X is the robot's right.**

### Converting between frames

Use these instead of writing `sin`/`cos` by hand:

```cpp
Pose2D compose(const Pose2D& base, const Pose2D& local);

Vec2 headingVector(double rad);                  // (sin, cos)
double headingToward(const Vec2& from, const Vec2& to);  // compass bearing

Vec2 fieldToRobot(const Vec2& field_vec, double heading_rad);
Vec2 robotToField(const Vec2& robot_vec, double heading_rad);
Vec2 fieldPointToRobot(const Vec2& field_point, const Pose2D& robot_pose);
Vec2 robotPointToField(const Vec2& robot_point, const Pose2D& robot_pose);

double arcRadius(const Pose2D& from, const Vec2& target);
```

`fieldToRobot` / `robotToField` take a displacement and only rotate it. The
`...Point...` variants translate first, so they answer "where is this field
point relative to the robot".

### `rotationMatrix` and `rotate` are not the field convention

`rotationMatrix(rad)` is the standard textbook rotation: counter-clockwise,
zero along +X, `[[cos, -sin], [sin, cos]]`. Keep using it for generic linear
algebra. Do **not** use it to move a pose or a waypoint between frames.

Feeding it a compass heading turns the wrong way: `rotate({0, 1}, rad)` gives
`(-sin, cos)`, while a robot at that heading actually points at `(sin, cos)`.
Because the two frames are transposes, `rotationMatrix(rad)` happens to equal
the field-to-robot matrix, so `rotate()` with a compass heading silently does
`fieldToRobot()` -- the inverse of what "rotate my local offset into the
field" means. Call `fieldToRobot` / `robotToField` and the bug cannot happen.

### `Pose2D::operator+` is not a compose

`operator+` adds `x`, `y`, and `theta` component-wise and wraps `theta`. It
does **not** rotate the incoming translation by the existing heading, so it is
"add a field-frame offset", not "move in my own frame". `compose(base, local)`
is the real SE(2) operation: `local` is interpreted in `base`'s frame, with
`local.x` to the right and `local.y` forward.

```cpp
Pose2D p{0, 0, degToRad(90)};            // facing +X
p + Pose2D{0, 10, 0}      // -> (0, 10) : offset added in field coordinates
compose(p, Pose2D{0, 10, 0})  // -> (10, 0) : 10 inches forward
```
