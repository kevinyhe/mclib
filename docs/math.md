# Math

`mclib/math.hpp`:

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

`utils.hpp` is in the global namespace:

```cpp
double degToRad(double deg);
double radToDeg(double rad);

// x/y/x1/y1 in inches (field frame), angle in DEGREES (compass frame).
// Legacy and frame-buggy; see below. Returns +infinity when the
// denominator degenerates.
double getRadius(double x, double y, double x1, double y1, double angle);
```

`getRadius` puts `delta_y` where the robot-frame lateral offset belongs, so it
returns 5 for a target 10 in straight ahead. Use `mclib::arcRadius()`.

## Modules

| Path | Contents |
| --- | --- |
| `auton/` | autonomous routine builder, selector |
| `chassis/` | chassis hardware and controllers |
| `command/` | command scheduler |
| `control/` | motion routines, odometry, profiles, feedforward, drive curves, `chassis_io.hpp` drive interface |
| `control/motion_config.hpp` | tuning shared by every drive loop |
| `device/` | wrappers over PROS devices |
| `mechanism/` | mechanism classes |
| `path/` | paths, splines, pure pursuit |
| `pid.hpp` | PID controller |
| `robot_geometry.hpp` | optional place to declare drive geometry |
| `snapshot/` | distance-sensor pose correction |
| `telemetry/` | CSV logging |
| `units/` | unit types |
| `utils.hpp` | angle and geometry utilities |

`snapshot/` corrects the robot's position using distance sensors aimed at the
field walls. `raycast.cpp` predicts what each sensor should read from the field
map in `collision_map.hpp`, `snapshot_pose.cpp` finds the `(x, y)` that best
matches the real readings and rejects bad fits, and `snapshot.hpp` reads the
sensors. Only `sensor.hpp` depends on PROS.
