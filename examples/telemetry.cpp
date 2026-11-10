// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: logging with mclib/telemetry, and the clock in mclib/time.hpp. A
// lift moves to a height and a log records what it did, as a CSV file to
// graph afterwards. Autonomous logs to numbered files on the SD card. A
// button in driver control logs one test move to a file with a fixed name.
// Driver control itself streams a few columns over USB. Change every port and
// every number to match your robot.
#include "main.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <optional>
#include <string>

using namespace mclib::units::literals;
using mclib::device::AnalogAxis;
using mclib::device::DigitalButton;
using mclib::telemetry::Channel;
using mclib::telemetry::Logger;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
mclib::device::Controller master(mclib::device::ControllerId::Master);
mclib::device::MotorGroup lift_motors({-3, 4}, mclib::device::Gearset::Blue);

// ---------------------------------------------------------------------------
// Status messages. The PROS terminal and the USB log share one link, so a
// line printed while that log runs lands between two rows of the CSV. Every
// message in this file starts with #, like the last line of a log, so
// whatever reads the CSV can skip them.
//
// The controller screen is 3 lines of 15 characters. The text is padded to a
// full line so that nothing of the message before it is left showing.
// ---------------------------------------------------------------------------
void showOnController(std::uint8_t line, std::string text) {
  text.resize(15, ' ');
  if (!master.setText(line, 0, text)) {
    std::printf("# the controller did not take \"%s\"\n", text.c_str());
  }
}

// ---------------------------------------------------------------------------
// The lift: an arm on a geared pivot.
// ---------------------------------------------------------------------------
const double arm_turns_per_motor_turn = 12.0 / 60.0;
const QLength arm_length = 14_in;

// Arm angles, measured up from where the arm rests.
const QAngle lift_score_angle = 75_deg;
const QAngle lift_tolerance = 1.5_deg;
const QTime lift_move_limit = 2_s;

// Volts for each degree still to go: full voltage from 30 degrees out.
const auto lift_gain = 12_V / 30_deg;

std::optional<QAngle> armAngle() {
  const std::optional<QAngle> motor_angle = lift_motors.averagePosition();
  if (!motor_angle) {
    return std::nullopt;
  }
  return *motor_angle * arm_turns_per_motor_turn;
}

// ---------------------------------------------------------------------------
// The columns of the lift's log. Every row also starts with t_ms, the time
// the row was recorded.
// ---------------------------------------------------------------------------
enum class LiftEvent { None, MoveStarted, Arrived, OutOfTime, NoAngle };

struct LiftColumns {
  // Add every column before the first row is recorded. Each call puts its
  // unit on the end of the name: addAngle("arm") is the column arm_deg and
  // addAngularVelocity("arm") is arm_degps. A log takes 24 columns, and a
  // name is cut at 31 characters, unit included.
  explicit LiftColumns(Logger& log)
      : target(log.addAngle("target")),
        angle(log.addAngle("arm")),
        arm_speed(log.addAngularVelocity("arm")),
        // addChannel takes any unit and the text to label it with.
        motor_speed(log.addChannel("motor", mclib::units::rpm, "rpm")),
        command(log.addVoltage("command")),
        current(log.addCurrent("current")),
        tip_travel(log.addLength("tip")),
        tip_speed(log.addVelocity("tip")),
        move_time(log.addTime("move")),
        pass_time(log.addTime("pass")),
        // addNumber is a column with no unit. This one holds a LiftEvent.
        event(log.addNumber("event")) {}

  // A column that was not added ignores every set(). Columns are handed out
  // in order, so if the last one was added they all were.
  bool allAdded() const { return event.valid(); }

  // A handle only takes its own kind of value: setting `command` to an angle
  // does not compile.
  Channel<QAngle> target;
  Channel<QAngle> angle;
  Channel<QAngularVelocity> arm_speed;
  Channel<QAngularVelocity> motor_speed;
  Channel<QVoltage> command;
  Channel<QCurrent> current;
  Channel<QLength> tip_travel;
  Channel<QVelocity> tip_speed;
  Channel<QTime> move_time;
  Channel<QTime> pass_time;
  Channel<double> event;
};

// A column keeps its last value until it is set again, so a read that failed
// has to be set too, or the row repeats the old reading. NaN is written to
// the file as nan. This is for the log only: a loop that drives a motor tests
// the optional and stops, as moveLift() does below.
template <typename Reading>
Reading orNan(std::optional<Reading> reading) {
  return reading.value_or(Reading(std::numeric_limits<double>::quiet_NaN()));
}

struct LiftPass {
  QAngle target;
  QVoltage command;
  QTime since_start;
  QTime pass_time;
};

void recordPass(Logger& log, const LiftColumns& columns, const LiftPass& pass) {
  // due() is true when the period has passed and the next sample() will
  // record a row. Testing it first skips the sensor reads on the passes that
  // record nothing.
  if (!log.due()) {
    return;
  }

  const QAngle angle = orNan(armAngle());
  const QAngularVelocity motor_speed = orNan(lift_motors.averageVelocity());
  const QAngularVelocity arm_speed = motor_speed * arm_turns_per_motor_turn;

  log.set(columns.target, pass.target);
  log.set(columns.angle, angle);
  log.set(columns.arm_speed, arm_speed);
  log.set(columns.motor_speed, motor_speed);
  log.set(columns.command, pass.command);
  log.set(columns.current, orNan(lift_motors.averageCurrent()));
  // How far the end of the arm has swung, and how fast it is moving.
  log.set(columns.tip_travel, mclib::units::arcLength(arm_length, angle));
  log.set(columns.tip_speed, mclib::units::rimVelocity(arm_length, arm_speed));
  log.set(columns.move_time, pass.since_start);
  log.set(columns.pass_time, pass.pass_time);
  log.sample();
}

// commit() records a row now, without waiting for the period. The event
// column is a number to search the file for. The other columns repeat what
// they last held.
void recordEvent(Logger& log, const LiftColumns& columns, LiftEvent event) {
  log.set(columns.event, static_cast<double>(event));
  log.commit();
  log.set(columns.event, static_cast<double>(LiftEvent::None));
}

// ---------------------------------------------------------------------------
// Moving the lift, and the clock.
// ---------------------------------------------------------------------------
LiftEvent moveEnding(QAngle target, std::optional<QAngle> angle,
                     std::uint32_t start_ms) {
  if (!angle) {
    return LiftEvent::NoAngle;
  }
  if (mclib::units::abs(target - *angle) <= lift_tolerance) {
    return LiftEvent::Arrived;
  }
  // hasElapsed() is the test for "has this long passed since then". It
  // counts whole milliseconds and is right when the clock wraps.
  if (mclib::time::hasElapsed(start_ms, lift_move_limit)) {
    return LiftEvent::OutOfTime;
  }
  return LiftEvent::None;
}

// Drives the lift to an arm angle and logs every pass. Returns how the move
// ended.
LiftEvent moveLift(QAngle target, Logger& log, const LiftColumns& columns) {
  // millis() is milliseconds since the program started.
  const std::uint32_t start_ms = mclib::time::millis();
  std::uint32_t last_pass_ms = start_ms;

  // now() is the same clock as a QTime, for sums with other typed values.
  // Do not test a time limit with it: two now() values subtracted can come
  // out a hair short, and a test made every 10 ms then passes one pass late.
  const QTime started = mclib::time::now();

  recordEvent(log, columns, LiftEvent::MoveStarted);
  while (true) {
    const std::optional<QAngle> angle = armAngle();
    const LiftEvent ending = moveEnding(target, angle, start_ms);
    if (ending != LiftEvent::None) {
      lift_motors.stop();
      recordEvent(log, columns, ending);
      if (ending != LiftEvent::Arrived) {
        std::printf("# lift move to %.0f deg ended with event %d\n",
                    target.deg(), static_cast<int>(ending));
      }
      return ending;
    }

    const QVoltage command =
        mclib::units::clamp((target - *angle) * lift_gain, -12_V, 12_V);
    lift_motors.setVoltage(command);

    // msSince() is whole milliseconds since a millis() reading. A 10 ms loop
    // that logs 14 in this column is being held up by something.
    const std::uint32_t pass_ms = mclib::time::msSince(last_pass_ms);
    last_pass_ms += pass_ms;

    recordPass(log, columns,
               {.target = target,
                .command = command,
                .since_start = mclib::time::now() - started,
                .pass_time = pass_ms * 1_ms});
    pros::delay(10);
  }
}

// ---------------------------------------------------------------------------
// Reading a log's health. The same three counts are the last line of the
// file:
//   # rows=412 dropped=0 discarded=0
// Dropped rows found the buffer full and were never stored: use a bigger
// RowBuffer or a longer period. Discarded rows were stored and then thrown
// away because the sink could not take them.
// ---------------------------------------------------------------------------
void reportLog(const char* name, const Logger& logger) {
  std::printf("# %s log: %d rows, %d dropped, %d discarded\n", name,
              static_cast<int>(logger.rowCount()),
              static_cast<int>(logger.droppedRows()),
              static_cast<int>(logger.discardedRows()));

  const std::size_t row_cap = logger.config().max_rows;
  if (row_cap != 0 && logger.rowCount() >= row_cap) {
    std::printf("# %s log: stopped at its cap of %d rows\n", name,
                static_cast<int>(row_cap));
  }
}

// ---------------------------------------------------------------------------
// The autonomous log: numbered files on the SD card.
//
// A Logger is used once: add columns, record rows, close. Each run builds a
// new one, and a new sink with it. This struct is one run's log, with its
// members in the order they have to be built.
// ---------------------------------------------------------------------------

// NullSink takes nothing, like a brain with no card in it. Set this true to
// see what the program reports then, without pulling the card.
const bool rehearse_missing_card = false;

struct CardLog {
  // Rows wait here between being recorded and being written. 256 rows is
  // about 50 KB, too big for a task's stack: keep it in a global or a static.
  mclib::telemetry::RowBuffer<256> rows;

  // Writes /usd/lift000.csv, then lift001.csv on the next run, and never
  // replaces a file. Each run tries the names in order until one is free, so
  // clear old logs off the card now and then. A second argument,
  // SdSinkConfig, changes the 4 MiB size cap and the highest number, 999.
  mclib::telemetry::SdCardSink card{"lift"};
  mclib::telemetry::NullSink missing_card;
  mclib::telemetry::Sink& sink =
      rehearse_missing_card
          ? static_cast<mclib::telemetry::Sink&>(missing_card)
          : card;

  // The last argument is a LoggerConfig. sample() records a row at most once
  // per period.
  Logger logger{sink, rows, {.period = 20_ms}};
  LiftColumns columns{logger};

  // Writes the buffered rows to the sink every 100 ms from a task of its
  // own, below the priority of the loop that records them. A slow card then
  // never holds up a 10 ms loop. It comes last so that it starts after the
  // columns exist.
  mclib::telemetry::FlushTask flusher{logger};
};

// PROS ends autonomous() and opcontrol() by deleting their task. Nothing
// after the line the task had reached runs, and that includes the
// destructors of its local variables. A flush task made as a local would
// carry on, writing through a logger that is gone. So each log is a global,
// built when its run starts and closed from disabled().
std::optional<CardLog> card_log;

void reportCardLog(const CardLog& log) {
  reportLog("card", log.logger);

  // The sink is not asked for a file until there is a row to write.
  if (log.logger.rowCount() == 0) {
    return;
  }
  const char* path = log.card.path();

  // sinkAlive() is false once any sink has turned rows away. Each kind of
  // sink can then say why.
  if (log.logger.sinkAlive()) {
    std::printf("# card log: %d bytes in %s\n",
                static_cast<int>(log.card.bytesWritten()), path);
    const char* file_name =
        path + std::strlen(mclib::telemetry::kSdMountPoint);
    showOnController(0, std::string("LOG ") + file_name);
  } else if (path[0] == '\0') {
    std::printf("# card log: no file. No card in the brain, a card that "
                "cannot be written, or every file name taken\n");
    showOnController(0, "NO LOG FILE");
  } else if (log.card.capped()) {
    std::printf("# card log: %s reached its size cap and was cut short\n",
                path);
    showOnController(0, "LOG CUT SHORT");
  } else if (log.card.writeFailed()) {
    std::printf("# card log: a write to %s failed. The card is full or was "
                "pulled out\n",
                path);
    showOnController(0, "LOG WRITE FAIL");
  }
}

// ---------------------------------------------------------------------------
// The driver log: a few columns down the USB cable, to watch live with
// `pros terminal`. `pros terminal > run.csv` saves it.
// ---------------------------------------------------------------------------
struct UsbLog {
  mclib::telemetry::RowBuffer<64> rows;

  // On the brain, stdout is the USB link. stdoutSink() is a ready-made
  // FileSink over it, shared by the whole program: the first logger to close
  // it closes it for good, and a later logger on it writes nothing. A
  // FileSink of this log's own does not have that problem.
  mclib::telemetry::FileSink usb{stdout};

  // USB is slower than the card. Keep the period long and the columns few.
  // precision is the number of digits in each value, 6 unless changed.
  // max_rows = 0 lifts the cap of 30000 rows, which a practice session at
  // this period reaches in 25 minutes.
  Logger logger{usb, rows, {.period = 50_ms, .max_rows = 0, .precision = 4}};

  // percent is the unit 0.01, so a stick fraction of 0.5 is logged as 50.
  Channel<QNumber> stick =
      logger.addChannel("stick", mclib::units::percent, "pct");
  Channel<QAngle> angle = logger.addAngle("arm");
  Channel<QCurrent> current = logger.addCurrent("current");

  mclib::telemetry::FlushTask flusher{logger, 50_ms};
};

std::optional<UsbLog> usb_log;

void finishLogs() {
  if (card_log) {
    // stop() waits for the flush task to end, writes the rows still in the
    // buffer and the line of counts, and closes the file.
    card_log->flusher.stop();
    reportCardLog(*card_log);
    card_log.reset();
  }
  if (usb_log) {
    usb_log->flusher.stop();
    reportLog("USB", usb_log->logger);
    std::printf("# USB log: %d rows by the flush task, %d bytes in all\n",
                static_cast<int>(usb_log->flusher.rowsWritten()),
                static_cast<int>(usb_log->usb.bytesWritten()));
    if (usb_log->usb.writeFailed()) {
      std::printf("# USB log: a write failed and the log stopped there\n");
    }
    usb_log.reset();
  }
}

void reportIfColumnsMissing(const char* name, bool all_added) {
  if (!all_added) {
    std::printf("# %s log: a column was not added\n", name);
    showOnController(1, "LOG COLUMN LOST");
  }
}

void startCardLog() {
  // A mode change with no disabled() in between leaves the last log open.
  finishLogs();
  card_log.emplace();
  reportIfColumnsMissing("card", card_log->columns.allAdded());
}

void startUsbLog() {
  finishLogs();
  usb_log.emplace();
  reportIfColumnsMissing("USB", usb_log->current.valid());
}

// ---------------------------------------------------------------------------
// The step test: one move, logged to a file the program opens itself, with
// no flush task.
// ---------------------------------------------------------------------------
const char* const step_test_path = "/usd/lift_step.csv";

void runStepTest() {
  static mclib::telemetry::RowBuffer<256> rows;

  // /usd/ is the SD card. "w" replaces the file, so the newest test always
  // has this name.
  std::FILE* file = std::fopen(step_test_path, "w");
  if (file == nullptr) {
    std::printf("# step test: cannot open %s: %s\n", step_test_path,
                std::strerror(errno));
    showOnController(0, "NO STEP FILE");
    return;
  }

  // FileSink writes to any open file, and leaves closing it to the program.
  mclib::telemetry::FileSink sink(file);
  Logger logger(sink, rows, {.period = 10_ms});
  const LiftColumns columns(logger);
  reportIfColumnsMissing("step", columns.allAdded());

  // With no flush task, rows wait in the buffer until flush() or close(), so
  // nothing touches the card while the lift moves. The buffer has to hold the
  // whole move. A RowBuffer<256> holds 255 rows, 2.55 s at this period, and
  // the move is limited to 2 s. If the robot is disabled part-way the file
  // is left empty.
  const LiftEvent ending = moveLift(lift_score_angle, logger, columns);

  // close() writes every row and the line of counts.
  logger.close();
  reportLog("step", logger);
  if (sink.writeFailed()) {
    std::printf("# step test: a write to %s failed\n", step_test_path);
  }
  if (std::fclose(file) != 0) {
    std::printf("# step test: closing %s failed: %s\n", step_test_path,
                std::strerror(errno));
  }
  showOnController(0, ending == LiftEvent::Arrived ? "STEP LOGGED"
                                                   : "STEP FAILED");
}

// ---------------------------------------------------------------------------
// A dry run: the exact text of a log, printed at startup without moving the
// lift and without waiting.
// ---------------------------------------------------------------------------
std::uint32_t pretend_ms = 0;

void previewLog() {
  // ScopedClock makes millis(), now() and every timer built on them read the
  // function it is given, until it goes out of scope. setClock() and
  // restoreSystemClock() do the same by hand, and getClock() returns the
  // function in use. Every task shares the one clock, so swap it only while
  // nothing else is running, as in initialize().
  pretend_ms = 0;
  const mclib::time::ScopedClock pretend_clock([]() { return pretend_ms; });

  // MemorySink keeps the text in a std::string that grows with every row. It
  // suits a few rows like these, not a match.
  static mclib::telemetry::RowBuffer<8> rows;
  mclib::telemetry::MemorySink text;
  Logger logger(text, rows, {.period = 20_ms});
  const LiftColumns columns(logger);

  // The event row, then five passes of a 10 ms loop on the pretend clock:
  // rows at 0, 20 and 40.
  recordEvent(logger, columns, LiftEvent::MoveStarted);
  for (int pass = 0; pass < 5; ++pass) {
    recordPass(logger, columns,
               {.target = lift_score_angle,
                .command = 0_V,
                .since_start = pretend_ms * 1_ms,
                .pass_time = 10_ms});
    pretend_ms += 10;
  }

  // flush() turns the buffered rows into text, hands them to the sink and
  // returns how many it wrote. The first one also writes the line of names.
  const std::size_t rows_written = logger.flush();
  logger.close();

  std::printf("# preview: %d rows\n", static_cast<int>(rows_written));
  for (const std::string& line : text.lines()) {
    std::printf("# preview: %s\n", line.c_str());
  }
}

void initialize() {
  lift_motors.setBrakeMode(mclib::device::BrakeMode::Hold);
  // The lift has to be resting at the bottom when the program starts: this
  // makes that position zero.
  lift_motors.tarePosition();
  previewLog();
}

void disabled() {
  lift_motors.stop();
  finishLogs();
}

void competition_initialize() {}

void autonomous() {
  startCardLog();
  Logger& log = card_log->logger;
  const LiftColumns& columns = card_log->columns;

  if (moveLift(lift_score_angle, log, columns) == LiftEvent::Arrived) {
    moveLift(0_deg, log, columns);
  }

  // No more rows. The flush task writes the ones still buffered, and
  // disabled() closes the file, whether this routine got here or PROS
  // stopped it first.
  log.stop();
}

void opcontrol() {
  startUsbLog();
  UsbLog& log = *usb_log;

  std::uint32_t last_status_ms = mclib::time::millis();
  while (true) {
    const QNumber stick = master.analog(AnalogAxis::RightY);
    const double lift_power = stick;
    if (std::abs(lift_power) < 0.08) {
      lift_motors.stop();
    } else {
      lift_motors.setPercent(lift_power);
    }

    if (master.getNewPress(DigitalButton::X)) {
      runStepTest();
    }

    // The plain way to log: set every column and call sample() on every
    // pass. set() only stores a number, and sample() records nothing until
    // the period has passed.
    log.logger.set(log.stick, stick);
    log.logger.set(log.angle, orNan(armAngle()));
    log.logger.set(log.current, orNan(lift_motors.averageCurrent()));
    log.logger.sample();

    // pending() is the rows recorded and not yet written. A number climbing
    // toward the buffer's 63 means the link is not keeping up and rows are
    // about to be dropped.
    if (mclib::time::hasElapsed(last_status_ms, 1_s)) {
      last_status_ms = mclib::time::millis();
      if (!master.print(2, 0, "LOG WAITING %-3d",
                        static_cast<int>(log.logger.pending()))) {
        std::printf("# the controller did not take the log status\n");
      }
    }

    pros::delay(10);
  }
}
