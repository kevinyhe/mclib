# Telemetry and time

## CSV logging

`mclib/telemetry/telemetry.hpp`. Logs values at a fixed rate to a CSV file on the
SD card. Column names include units.

```cpp
#include "mclib/telemetry/telemetry.hpp"

void autonomous() {
  // One Logger per run. The buffer is static because 256 rows (~50 KB) is too
  // large for a task stack.
  static mclib::telemetry::RowBuffer<256> tlm_buffer;
  mclib::telemetry::SdCardSink tlm_sink("auton");
  mclib::telemetry::Logger tlm(tlm_sink, tlm_buffer,
                               {.period = 20 * mclib::units::millisecond});

  // Register channels before the first sample. One line each.
  auto x = tlm.addLength("x");            // column "x_in"
  auto heading = tlm.addAngle("heading"); // column "heading_deg"
  auto cmd = tlm.addVoltage("left_cmd");  // column "left_cmd_V"
  auto err = tlm.addNumber("error");      // column "error", no unit

  // Writes to the SD card from a background task. Declare it after the logger.
  mclib::telemetry::FlushTask flusher(tlm);

  while (!controller.settled()) {
    // ... control tick ...
    const mclib::Pose2D pose = chassis.getPose();
    tlm.set(x, pose.x * mclib::units::inch);
    tlm.set(heading, pose.theta * mclib::units::radian);
    tlm.set(cmd, left_command);
    tlm.set(err, controller.error());
    tlm.sample();  // no-op until the 20 ms period elapses
    pros::delay(10);
  }
  flusher.stop();  // joins the task, drains the buffer, closes the file
}
```

A `Logger` is single-use: add channels, sample, close. Create a new one for each
run. To share one across autonomous and driver control, keep it in a
`std::optional` and reconstruct it for each run.

`SdCardSink("auton")` writes `/usd/auton000.csv`, then `auton001.csv` on the next
run, and so on. Existing files are never overwritten.

```
t_ms,x_in,heading_deg,left_cmd_V,error
0,0,0,12,24
20,1.4,0.2,12,22.6
40,3.1,0.4,11.8,20.9
...
# rows=412 dropped=0 discarded=0
```

A value set on an `addLength` channel is written in inches; `24_in` is written
as `24` in the `x_in` column. An `addAngle` channel is written in degrees, so
pass `Pose2D::theta`, which is radians, as `pose.theta * mclib::units::radian`.

### Performance

`set()` stores a number. `sample()` copies one row (about 200 bytes) into a
buffer. Neither allocates memory, locks or writes to the SD card, so logging
does not slow the control loop.

`FlushTask` writes the buffer to the card every 100 ms from a low-priority task.

### Limits

| Limit | Default | When reached |
| --- | --- | --- |
| Buffer full | `RowBuffer` size | new rows are dropped; the count is written to the last line |
| Row cap | `LoggerConfig::max_rows` = 30000 (10 min at 20 ms) | logging stops |
| File size | `SdSinkConfig::max_bytes` = 4 MiB | the file is closed |
| Channels | 24, names up to 31 characters including the unit suffix | extra channels are ignored; longer names are cut |

### No SD card

The sink checks for the card once, on the first write. Without a card, or if the
card fails, logging does nothing and the program continues.

### Testing

`MemorySink` stores output in a `std::string`, and `NullSink` discards it.
`tests/telemetry_test.cpp` checks the CSV output on the host. Only
`src/mclib/telemetry/flush_task.cpp` includes PROS.

### Log a routine for the path planner

The path planner's **Robot log…** button draws a log over the plan and the
simulated run, and says how far apart they are in each step. It needs the
pose: columns `x_in` and `y_in`, and `heading_deg` for the heading gaps.
Nothing logs the pose unless you add it. This runs a routine exported from
the planner and logs the pose every 20 ms from a task of its own:

```cpp
#include "main.h"

#include <atomic>

using namespace mclib::units::literals;

// Runs @p routine and logs where odometry puts the robot to
// /usd/<base_name>NNN.csv.
void runAndLog(void (*routine)(), const char* base_name) {
  static mclib::telemetry::RowBuffer<256> buffer;
  mclib::telemetry::SdCardSink sink(base_name);
  mclib::telemetry::Logger log(sink, buffer, {.period = 20 * mclib::units::millisecond});
  auto x = log.addLength("x");             // x_in
  auto y = log.addLength("y");             // y_in
  auto heading = log.addAngle("heading");  // heading_deg
  mclib::telemetry::FlushTask flusher(log);

  std::atomic<bool> running{true};
  pros::Task sampler([&] {
    while (running) {
      const mclib::Pose2D pose = mclib::control::robotState().pose();
      log.set(x, pose.x * 1_in);
      log.set(y, pose.y * 1_in);
      log.set(heading, pose.theta * 1_rad);
      log.sample();
      pros::delay(10);
    }
  });
  routine();
  running = false;
  sampler.join();
  flusher.stop();
}

void autonomous() { runAndLog(runLeftSide, "auton"); }
```

`runLeftSide` is the function the planner exported for a routine named
"Left side". The first row is written when the routine starts, so the
planner lines it up with the start of the run. If it is off, set **Log
starts at** under Activity.

The log is what odometry believed, not where the robot really was. The
planner compares it with the simulated odometry at the same time into each
step, with the planned path, and with each step's target.

Without a step column the planner splits the log by the simulated run's step
times, which drift if the robot is faster or slower than the simulation. To
split it by the robot's own step times, add `auto step = log.addNumber("step");`
and set it to the planner's step number, from 1, before each step:
`log.set(step, 3.0);`. The exported routine marks each step with a comment
such as `// 3. Turn to heading`.

## USB serial

`mclib/telemetry/file_sink.hpp`. `stdoutSink()` sends the CSV to the PROS
terminal over USB.

```cpp
#include "mclib/telemetry/file_sink.hpp"
#include "mclib/telemetry/telemetry.hpp"

void opcontrol() {
  static mclib::telemetry::RowBuffer<256> tlm_buffer;
  mclib::telemetry::Logger tlm(mclib::telemetry::stdoutSink(), tlm_buffer,
                               {.period = 50 * mclib::units::millisecond});
  auto x = tlm.addLength("x");
  auto heading = tlm.addAngle("heading");
  mclib::telemetry::FlushTask flusher(tlm);

  while (true) {
    const mclib::Pose2D pose = chassis.getPose();
    tlm.set(x, pose.x * mclib::units::inch);
    tlm.set(heading, pose.theta * mclib::units::radian);
    tlm.sample();
    pros::delay(10);
  }
}
```

Run `pros terminal > run.csv` to save it.

`FileSink(file)` writes to any open `std::FILE*` and never closes it. If a write
fails, the sink stops writing.

Serial is slower than the SD card. Sample every 20 ms or slower with few
channels, and check the dropped count on the last line.

## Time

`mclib/time.hpp`.

```cpp
#include "mclib/time.hpp"

const std::uint32_t t = mclib::time::millis();  // raw milliseconds
const QTime now = mclib::time::now();           // same value as a QTime
```

`mclib::time::millis()` returns milliseconds since the program started, like
`pros::millis()`. `now()` returns the same value as a `QTime`.

Code that reads time through `mclib::time` can be tested with a fake clock.
`pid.cpp`, `chassis_controller.cpp`, the mechanisms and `waitCommand.h` do.
`control/motion.cpp` and `control/odometry_task.cpp` call `pros::millis()` and
`pros::delay()` directly, so a fake clock does not affect them.
`auton/autonomous_routine.cpp` reads `mclib::time` but sleeps with
`pros::delay()`.

### Fake clocks in tests

A host test provides `systemMillis()` and installs its own clock:

```cpp
namespace mclib {
namespace time {
std::uint32_t systemMillis() { return 0; }
}  // namespace time
}  // namespace mclib

static std::uint32_t fake_ms = 0;

{
  mclib::time::ScopedClock clock([]() { return fake_ms; });

  PID pid(1.0, 0.0, 0.0);
  pid.setTarget(10.0);
  for (int tick = 0; tick < 6; ++tick) {
    fake_ms = tick * 20;
    pid.update(10.0);
  }
  // small settle window is 100 ms, so arrival latches on the tick at t = 100 ms
  assert(pid.targetArrived());
}
// ScopedClock put the previous clock back here
```

`setClock()` installs a clock and returns the previous one, `getClock()` returns
the current one, and `restoreSystemClock()` restores the real clock.
`ScopedClock` restores the previous clock when it goes out of scope.
