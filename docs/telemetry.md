# Telemetry and time

CSV logging to the SD card, streaming over USB serial, and the clock seam that makes both testable.

[Documentation index](README.md) · [Project README](../README.md)

## Telemetry: CSV logging to the SD card

Tuning a PID by watching the robot and guessing is the slowest way to do it.
`mclib/telemetry/telemetry.hpp` gives you a fixed-rate CSV log on the V5 SD
card, with columns that say what unit they are in.

```cpp
#include "mclib/telemetry/telemetry.hpp"

void autonomous() {
  // A Logger is one-shot: register, sample, close. Build a fresh one per run
  // so a second autonomous over field control gets a second log. The ring
  // storage is static because sizeof(Row) is ~200 bytes and 256 rows has no
  // business on a task stack; the logger is not.
  static mclib::telemetry::RowBuffer<256> tlm_buffer;
  mclib::telemetry::SdCardSink tlm_sink("auton");
  mclib::telemetry::Logger tlm(tlm_sink, tlm_buffer,
                               {.period = 20 * mclib::units::millisecond});

  // Register channels before the first sample. One line each.
  auto x = tlm.addLength("x");            // column "x_in"
  auto heading = tlm.addAngle("heading"); // column "heading_deg"
  auto cmd = tlm.addVoltage("left_cmd");  // column "left_cmd_V"
  auto err = tlm.addNumber("error");      // column "error", no unit

  // Starts a background pros::Task that does the SD writes. Declared after the
  // logger, so it is destroyed first and never outlives what it drains.
  mclib::telemetry::FlushTask flusher(tlm);

  while (!controller.settled()) {
    // ... control tick ...
    tlm.set(x, pose.x);
    tlm.set(heading, pose.theta);
    tlm.set(cmd, left_command);
    tlm.set(err, controller.error());
    tlm.sample();  // no-op until the 20 ms period elapses
    pros::delay(10);
  }
  flusher.stop();  // joins the task, drains the buffer, closes the file
}
```

If the logger has to live longer than one run - shared between auton and
driver control, say - keep it in a `std::optional` and re-construct it at the
start of each run. Registration, sampling and `close()` are all one-way on a
given `Logger`: once `close()` has run, that object is finished.

The file lands at `/usd/auton000.csv`, then `auton001.csv`, and so on - the
sink picks the first index that does not already exist, so a re-run never
overwrites the previous match's log. Output looks like this:

```
t_ms,x_in,heading_deg,left_cmd_V,error
0,0,0,12,24
20,1.4,0.2,12,22.6
40,3.1,0.4,11.8,20.9
...
# rows=412 dropped=0
```

Column names carry the unit, which is what makes a log readable a week later.
`24_in` written into an `addLength` channel reads back as `24.0` in the column
called `x_in`.

### What it costs the control loop

`set()` is one double store. `sample()` is a clock read, and on a sampling tick
a `sizeof(Row)` copy (about 200 bytes) into a lock-free ring plus one release
store. No allocation, no lock, no file I/O on the producer side. That is the
whole worst case a control tick pays - sub-microsecond on the V5's Cortex-A9,
and independent of how slow or jittery the SD card is.

The writing happens in `FlushTask`, a `pros::Task` at priority
`TASK_PRIORITY_DEFAULT - 1`, which drains the ring every 100 ms by default.
Batching means one `fwrite` per five rows at a 20 ms period rather than one per
row. The ring is single-producer / single-consumer over two `std::atomic`
indices, so the control loop never waits on the flush task and never waits on
the card.

### Bounded resources

Nothing here can grow without limit:

- **Buffer full: drop newest.** The new row is discarded and the buffered rows
  are kept, so the log is a contiguous prefix of the run with a gap at the end.
  A contiguous prefix beats a log with a hole punched in the middle, and it is
  the only policy a lock-free SPSC ring can offer without the producer racing
  the consumer's read cursor. Dropped rows are counted and the count is written
  into the CSV trailer, so a truncated log says so.
- **Row cap.** `LoggerConfig::max_rows` defaults to 30000 - ten minutes at
  20 ms. Past it the logger stops committing.
- **File size cap.** `SdSinkConfig::max_bytes` defaults to 4 MiB. Past it the
  sink closes the file. A log that fills the card mid-match is worse than none.
- **Channel cap.** 24 columns, names truncated to 32 characters. Registering a
  25th channel returns an invalid handle; writing through one is ignored.

### No SD card

`SdCardSink` probes for the card exactly once, on the first flush, by opening
the file. If there is no card the sink reports itself unavailable and every
later call is a no-op: no exceptions, no crash, no retry storm. The logger
still drains its ring so the control loop never wedges behind a dead sink. The
same holds for a card that dies mid-match.

### Testing without a card

The sink is an interface. `MemorySink` captures everything in a `std::string`
and `NullSink` is permanently unavailable, so `tests/telemetry_test.cpp` drives
the logger under a `mclib::time::ScopedClock` and asserts on real CSV text -
the header, the timestamps, the drop policy, and the round-trip of `24_in` back
to `24.0`. Only `src/mclib/telemetry/flush_task.cpp` includes a PROS header;
the logger and both test sinks are header-only.

## Telemetry over USB serial

`mclib/telemetry/file_sink.hpp`. The logger writes into any `Sink`; `FileSink`
is a sink over a `std::FILE*` it does not own, and on the V5 `stdout` is the
USB serial link. So a logger pointed at `stdoutSink()` streams its CSV into
the PROS terminal while the robot is tethered, no SD card involved:

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
    tlm.set(x, pose.x);
    tlm.set(heading, pose.theta);
    tlm.sample();
    pros::delay(10);
  }
}
```

Run `pros terminal` on the laptop and the CSV appears there, one row per
sample. Redirect it to a file (`pros terminal > run.csv`) to keep it.

`FileSink(file)` works over any open stream - `std::tmpfile()` in the host
test, `stdout` on the robot. It never calls `fclose`; `close()` only stops
further writes, so closing the sink never takes the serial link away from
`printf`. A short write (the link dropped, the stream closed underneath it)
marks the sink dead and every later write is refused, so a dead link costs
the flush task nothing. The trailer and the `# rows=... dropped=...` line come
out the same as on the SD card.

Serial is slower than the card and shared with everything else that prints.
Keep the sample period at 20 ms or above and the channel count modest, or
`sample()` will start dropping rows when the ring fills faster than the link
drains it. The dropped count in the trailer tells you if that happened.

## Time (`mclib/time.hpp`)

`mclib::time::millis()` returns milliseconds since the program started, exactly
like `pros::millis()` does, and `mclib::time::now()` returns the same value as a
`QTime`.

```cpp
#include "mclib/time.hpp"

const std::uint32_t t = mclib::time::millis();  // raw milliseconds
const QTime now = mclib::time::now();           // same value as a QTime
```

`time.hpp` includes no PROS header. `src/mclib/time.cpp` is the single
translation unit in the library that reads `pros::millis()`, through the
out-of-line function `mclib::time::systemMillis()`. That is what `millis()`
calls when no clock has been installed, so the clock is live from the first
static constructor onwards - there is no initialisation order to get wrong -
and every caller's undefined reference to it forces the linker to pull
`time.cpp.o` out of `mclib.a`.

### Converted so far

`pid.cpp`, `chassis_controller.cpp`, `mechanism.hpp`, `preset_position_mechanism.hpp`,
`waitCommand.h` and the mechanism sources (`auto_trigger`, `conveyor`, `homing`,
`position`, `pto`, `toggle_group`, `velocity`) all read time through the seam.
`control/motion.cpp`, `control/odometry.cpp` and `auton/autonomous_routine.cpp`
still call `pros::millis()` and `pros::delay()` directly; they are Phase 3 work.
Until then a fake clock does not affect those loops, so do not mix a
`ScopedClock` with a routine that drives them.

### Host tests

Because the seam is a function pointer, a host test can install its own clock,
step it by hand, and check timing behaviour without a robot or the PROS
toolchain. A host build does not link `time.cpp`, so it supplies its own
`systemMillis()` - one line, used only before a fake clock is installed:

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

`setClock()` installs a clock and returns the previous one, `getClock()` reports
it, and `restoreSystemClock()` goes back to the platform clock. `ScopedClock`
does the save/restore for you.

`src/mclib/pid.cpp` compiles and links with no PROS headers reachable at all,
which is what makes host-side testing of the control code possible.
