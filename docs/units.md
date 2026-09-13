# Units and coordinates

## Units

`mclib/units/units.hpp`. Distances, times, angles, voltages and currents are
typed values instead of plain numbers. The compiler checks that units combine
correctly: a length divided by a time is a velocity, and adding a length to a
time does not compile.

```cpp
QLength distance = 24_in;
QTime   timeout  = 1500_ms;
QAngle  heading  = 90_deg;

QVelocity speed = distance / timeout;   // QVelocity, checked at compile time
QLength   back  = speed * timeout;      // 24 in again
// QLength wrong = distance + timeout;  // error: no operator+
```

A unit type is exactly as fast and as large as a `double`. Values are still
floating point, so `(24_in).in()` is `23.999999999999996`; compare with a
tolerance, not `==`.

### Types

`QNumber`, `QLength`, `QArea`, `QTime`, `QAngle`, `QVoltage`, `QCurrent`,
`QVelocity`, `QAcceleration`, `QJerk`, `QAngularVelocity`,
`QAngularAcceleration`, `QCurvature` (1/length), `QFrequency` (1/time).

Other combinations are valid types too, e.g. `QVoltage / QCurrent`.

`QNumber` (a unitless number) converts to and from `double` automatically.
Other types must be created with a literal or constructor.

### Literals

| Dimension | Literals |
| --- | --- |
| Length | `_m` `_cm` `_mm` `_in` `_ft` `_tile` (24 in) |
| Time | `_s` `_ms` `_min` |
| Angle | `_rad` `_deg` `_rot` |
| Voltage, current | `_V` `_mV` `_A` `_mA` |
| Rates | `_mps` `_inps` `_mps2` `_radps` `_degps` `_rpm` |

Named constants (`metre`, `inch`, `tile`, `minute`, `degree`, `radian`,
`rotation`, `volt`, `millivolt`, `ampere`, `milliampere`, `rpm`, `percent`) are
in `mclib::units`:

```cpp
using namespace mclib::units;
QLength target = 24 * inch;
```

`millisecond` and `second` are also global.

### Converting to `double`

```cpp
double ms = timeout.ms();       // named accessor - says what the number means
double in = distance.in();
double dg = heading.deg();
double mv = battery.mV();

QLength from_raw{0.6096};       // explicit ctor, value in SI base units
```

| Dimension | Accessors |
| --- | --- |
| Length | `.m()` `.cm()` `.mm()` `.in()` `.ft()` |
| Time | `.s()` `.ms()` |
| Angle | `.rad()` `.deg()` |
| Voltage | `.volts()` `.mV()` |
| Current | `.amps()` `.mA()` |
| Velocity | `.mps()` `.inps()` |
| Angular velocity | `.radps()` `.degps()` `.rpm()` |

An accessor for the wrong dimension does not compile. Free functions
`inches(x)`, `milliseconds(t)` and `degrees(a)` do the same. `.raw()` returns
SI base units.

### Angles

`QAngle` stores radians. The motion functions take degrees and `Pose2D::theta`
is in radians. Use these helpers to combine angles with lengths:

```cpp
QLength arc     = arcLength(radius, angle);      // radius * angle
QAngle  swept   = arcAngle(arc, radius);         // the inverse
QVelocity rim   = rimVelocity(radius, omega);    // radius * angular velocity
QAngularVelocity turn = turnRate(speed, curvature);   // speed * curvature
QCurvature k    = turnCurvature(speed, omega);        // the inverse
```

`wrap(angle)` wraps into [-180°, +180°], matching `mclib::wrapAngle(double)`.

### Math

`abs`, `min`, `max` and `clamp` keep the unit. `sign` returns -1, 0 or +1.
`square` squares the unit (a length becomes an area). `sin`/`cos`/`tan` take a `QAngle`;
`asin`/`acos`/`atan` return one. `atan2(QLength, QLength)` returns a `QAngle`
and `hypot(QLength, QLength)` a `QLength`.

### Global names

The type aliases, `millisecond`, `second` and all literals are also in the
global namespace:

```cpp
QTime now = pros::millis() * millisecond;
if (now - start >= 250 * millisecond) { /* ... */ }
```

Define `MCLIB_NO_GLOBAL_UNITS` before including mclib to remove the aliases
(except `QTime`) and the literals from the global namespace, e.g. when using
okapilib. `QTime`, `millisecond` and `second` stay global. `Quantity` is only
available as `mclib::units::Quantity`.

### Tests

```sh
g++ -std=gnu++20 -Iinclude -o /tmp/units_test tests/units_test.cpp && /tmp/units_test
```

## Coordinate frame

mclib uses one field frame:

- `theta = 0` points along +Y
- `theta` increases clockwise, so +90° points along +X
- the unit vector for a heading is `(sin(theta), cos(theta))`
- the bearing from A to B is `atan2(b.x - a.x, b.y - a.y)`

The robot frame matches the field frame at `theta = 0`: +Y forward, +X right.

`math.hpp` uses radians (`Pose2D::theta`, `wrapAngle`). The motion API
(`Chassis`, `control/motion.cpp`, `RobotState::correctAngleDeg()`) uses
degrees. Convert with `degToRad` / `radToDeg` from `utils.hpp`. Distances are
inches.

### Frame conversions

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

`fieldToRobot` and `robotToField` rotate a displacement. The `...Point...`
versions also translate.

`rotationMatrix(rad)` and `rotate()` use the standard counter-clockwise frame
with zero along +X. Do not use them for poses or waypoints; use `fieldToRobot`
and `robotToField`.

### `Pose2D::operator+`

`operator+` adds `x`, `y` and `theta` component-wise in the field frame and
wraps `theta`. `compose(base, local)` applies `local` in `base`'s frame, with
`local.x` right and `local.y` forward.

```cpp
Pose2D p{0, 0, degToRad(90)};            // facing +X
p + Pose2D{0, 10, 0}      // -> (0, 10) : offset added in field coordinates
compose(p, Pose2D{0, 10, 0})  // -> (10, 0) : 10 inches forward
```
