# Core math API

The geometry and helper functions the rest of the library is built on.

[Documentation index](README.md) · [Project README](../README.md)

```cpp
using Vec2 = Eigen::Matrix<double, 2, 1>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Mat2 = Eigen::Matrix<double, 2, 2>;
using Mat3 = Eigen::Matrix<double, 3, 3>;

double clamp(double val, double min, double max);
double wrapAngle(double rad);  // radians, wraps into [-pi, pi] (closed both ends)

struct Pose2D {
  double x;      // inches, field frame
  double y;      // inches, field frame
  double theta;  // radians, 0 = +Y, clockwise-positive

  Pose2D operator+(const Pose2D& other) const;  // component-wise, not a compose
  double distanceTo(const Pose2D& other) const;
  Vec2 translation() const;
  Vec3 vector() const;
};

Pose2D compose(const Pose2D& base, const Pose2D& local);

Vec2 headingVector(double rad);
double headingToward(const Vec2& from, const Vec2& to);
Vec2 fieldToRobot(const Vec2& field_vec, double heading_rad);
Vec2 robotToField(const Vec2& robot_vec, double heading_rad);
Vec2 fieldPointToRobot(const Vec2& field_point, const Pose2D& robot_pose);
Vec2 robotPointToField(const Vec2& robot_point, const Pose2D& robot_pose);
double arcRadius(const Pose2D& from, const Vec2& target);

// Standard frame (CCW, 0 = +X). NOT the field convention; see units.md.
Mat2 rotationMatrix(double rad);
Vec2 rotate(const Vec2& vec, double rad);
```

`utils.hpp` lives in the **global namespace** and holds the degree/radian
bridge plus `getRadius`:

```cpp
double degToRad(double deg);
double radToDeg(double rad);

// x/y/x1/y1 in inches (field frame), angle in DEGREES (compass frame).
// Legacy and frame-buggy; see below. Returns +infinity when the
// denominator degenerates.
double getRadius(double x, double y, double x1, double y1, double angle);
```

`getRadius` is a legacy helper with a frame bug. Its denominator uses
`delta_y` where the target's **lateral** offset in the robot frame belongs, so a
target 10 in dead ahead of a robot at heading 0 (a straight line, with infinite
radius) comes back as 5. Use `mclib::arcRadius(from, target)`, which computes it
correctly.

Nothing in the library calls `getRadius` any more: `boomerang` switched to
`arcRadius()`. It stays in `utils.hpp` for existing robot code. Its degenerate
case returns infinity; it used to return `999`, which silently became a finite
speed limit downstream.

Other modules are split into matching header/source pairs:

- `auton/*.hpp` / `auton/*.cpp`: owning autonomous routine builder
- `chassis/*.hpp` / `chassis/*.cpp`: chassis hardware and PID controller
- `robot_geometry.hpp`: an optional place to declare your drive geometry once
- `control/motion_config.hpp` / `control/motion_config.cpp`: the one tuning record every drive loop reads
- `control/*.hpp` / `control/*.cpp`: the drive-hardware seam (`chassis_io.hpp`), scaling, motion, odometry, drive curves and shared state helpers
- `pid.hpp` / `pid.cpp`: PID controller
- `utils.hpp` / `utils.cpp`: angle and geometry utilities
- `device/*.hpp` / `device/*.cpp`: the only place that calls PROS motor, controller, pneumatic, and sensor APIs directly
- `mechanism/*.hpp` / `mechanism/*.cpp`: generic stateful mechanisms (conveyor, position, velocity, toggle, multi-position, homing, PTO) plus motor and pneumatic subsystem wrappers
- `snapshot/*.hpp` / `snapshot/*.cpp`: distance-sensor pose correction. `raycast.cpp`
  casts rays against the static field map in `collision_map.hpp`; `snapshot_pose.cpp`
  runs the damped Gauss-Newton solve for `(x, y)` and the accept/reject gate;
  `snapshot.hpp` is the only part that reads a real sensor. `sensor.hpp` holds the one
  PROS-dependent type, so the geometry and the solve are host-testable
  (`tests/snapshot_test.cpp`).
