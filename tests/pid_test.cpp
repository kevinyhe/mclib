// mclib
//
// Numeric tests for PIDController. This unit is pure arithmetic, so the whole
// verification is numbers: a recorded output trace pinned against the
// pre-template implementation, plus the specific edge semantics that callers in
// src/mclib/control/motion.cpp depend on.

#include "mclib/pid.hpp"

#include "mclib/time.hpp"
#include "mclib/units/units.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdint>

using namespace mclib::units;

namespace {

std::uint32_t g_fake_ms = 0;
std::uint32_t fakeClock() { return g_fake_ms; }

/** @brief One recorded tick of the golden trace: the output and the latch. */
struct GoldenTick {
  double output;
  int arrived;
};

// Captured from the implementation on `main`, before PID became a template:
// PID(0.5, 0.01, 2.0), setTarget(24), a plant of `process += output * 0.02`,
// ticked every 10 ms with every other setting left at its constructor default.
// Printed with %.17g, so these are the exact doubles that implementation
// produced, not rounded copies of them.
constexpr GoldenTick kGoldenDefault[] = {
    {12.24, 0},
    {11.865551999999996, 0},
    {11.997053289600004, 0},
    {12.104602184062081, 0},
    {12.20961276494824, 0},
    {12.311232850015589, 0},
    {12.409510107495016, 0},
    {12.504456403481667, 0},
    {12.596085583687465, 0},
    {12.684411939605701, 0},
    {12.769450262548331, 0},
    {12.851215833528059, 0},
    {12.929724415709789, 0},
    {13.004992246738489, 0},
    {13.077036031053666, 0},
    {13.145872932188038, 0},
    {13.21152056505184, 0},
    {13.273996988204836, 0},
    {13.333320696117076, 0},
    {13.389510611420613, 0},
    {13.442586077153173, 0},
    {13.492566848995821, 0},
    {13.539473087505828, 0},
    {13.583325350346552, 0},
    {13.624144584515566, 0},
    {13.661952118572851, 0},
    {13.696769654870332, 0},
    {13.728619261784232, 0},
    {13.757523365952004, 0},
    {13.783504744514739, 0},
    {13.806586517367148, 0},
    {13.826792139415975, 0},
    {13.844145392848571, 0},
    {13.858670379412924, 0},
    {13.87039151271048, 0},
    {13.879333510503184, 0},
    {13.885521387036061, 0},
    {13.888980445376591, 0},
    {13.889736269772339, 0},
    {13.887814718027961, 0},
    {13.883241913903028, 0},
    {13.87604423953179, 0},
    {13.866248327866202, 0},
    {13.85388105514348, 0},
    {13.838969533379236, 0},
    {13.82154110288762, 0},
    {13.801623324829439, 0},
    {13.779243973789534, 0},
    {13.754431030384543, 0},
    {13.727212673902123, 0},
    {13.697617274972849, 0},
    {13.665673388275749, 0},
    {13.631409745278672, 0},
    {13.594855247014511, 0},
    {13.556038956894273, 0},
    {13.514990093558104, 0},
    {13.471738023765216, 0},
    {13.426312255323776, 0},
    {13.378742430061632, 0},
    {13.329058316838916, 0},
};

// Same capture with a much tighter configuration, which exercises the
// derivative_tolerance gate, non-default settle durations, a sign flip in the
// error and the integral clamp: PID(1.0, 0.2, 0.0),
// setSmallBigErrorTolerance(0.1, 0.5), setDerivativeTolerance(0.05),
// setSmallBigErrorDuration(50, 200), target 5, plant gain 0.1, 10 ms ticks.
constexpr GoldenTick kGoldenTight[] = {
    {6, 0},
    {6.2800000000000002, 0},
    {6.4063999999999997, 0},
    {6.3920320000000004, 0},
    {6.2512601600000011, 0},
    {5.9995403008000006, 0},
    {5.6530016215040009, 0},
    {5.2280567777075202, 0},
    {4.741045282736537, 0},
    {-0.29513361427480689, 0},
    {-0.31874430341679166, 0},
    {-0.33361903757624223, 0},
    {-0.34033391756822234, 0},
    {-0.33957063120963993, 0},
    {-0.3320922608627237, 0},
    {-0.099184237783964235, 0},
    {-0.089265814005567989, 0},
    {-0.08033923260501119, 0},
    {-0.072305309344510071, 0},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
};

/**
 * @brief Replay a golden scenario and assert every tick matches exactly.
 */
void checkGoldenTrace(const char* name, PID& pid, double target, double gain,
                      const GoldenTick* expected, int ticks) {
  std::printf("-- golden trace: %s\n", name);
  g_fake_ms = 0;
  pid.setTarget(target);
  double process = 0.0;
  int mismatches = 0;
  for (int i = 0; i < ticks; ++i) {
    const double out = pid.update(process);
    if (out != expected[i].output ||
        static_cast<int>(pid.targetArrived()) != expected[i].arrived) {
      ++mismatches;
      std::printf("   tick %d: got out=%.17g arrived=%d, want out=%.17g arrived=%d\n",
                  i, out, static_cast<int>(pid.targetArrived()),
                  expected[i].output, expected[i].arrived);
    }
    process += out * gain;
    g_fake_ms += 10;
  }
  CHECK_EQ(static_cast<double>(mismatches), 0.0);
}

/**
 * @brief The claim this whole unit rests on: default settings are bit-identical
 * to the pre-template PID.
 */
void testBitIdenticalDefaults() {
  {
    PID pid(0.5, 0.01, 2.0);
    checkGoldenTrace("default", pid, 24.0, 0.02, kGoldenDefault, 60);
  }
  {
    PID pid(1.0, 0.2, 0.0);
    pid.setSmallBigErrorTolerance(0.1, 0.5);
    pid.setDerivativeTolerance(0.05);
    pid.setSmallBigErrorDuration(50, 200);
    checkGoldenTrace("tight", pid, 5.0, 0.1, kGoldenTight, 100);
  }
}

/**
 * @brief setUseDt(true) at a 10 ms tick multiplies the D term by exactly 100.
 */
void testUseDtScalesDerivative() {
  std::printf("-- setUseDt derivative scaling\n");

  // kp and ki zero, so the output is purely the D term.
  auto runTwoTicks = [](bool use_dt) {
    PID pid(0.0, 0.0, 2.0);
    pid.setArrive(false);
    pid.setUseDt(use_dt);
    pid.setTarget(10.0);
    g_fake_ms = 0;
    pid.update(0.0);  // error 10, first tick: delta 0
    g_fake_ms = 10;
    return pid.update(1.0);  // error 9, delta -1
  };

  const double raw = runTwoTicks(false);
  const double rate = runTwoTicks(true);
  std::printf("   raw-delta D = %.17g, rate D at 10 ms = %.17g, ratio %.17g\n",
              raw, rate, rate / raw);
  CHECK_EQ(raw, -2.0);
  CHECK_EQ(rate, -200.0);
  CHECK_EQ(rate / raw, 100.0);
}

/**
 * @brief In rate mode the integral is ki * sum(error * dt_seconds).
 */
void testUseDtScalesIntegral() {
  std::printf("-- setUseDt integral scaling\n");

  auto runTicks = [](bool use_dt, int ticks) {
    PID pid(0.0, 1.0, 0.0);
    pid.setArrive(false);
    pid.setSmallBigErrorTolerance(0.0, 0.0);
    pid.setIntegralMax(0.0);
    pid.setUseDt(use_dt);
    pid.setTarget(10.0);
    double out = 0.0;
    g_fake_ms = 0;
    for (int i = 0; i < ticks; ++i) {
      out = pid.update(0.0);
      g_fake_ms += 10;
    }
    return out;
  };

  // 5 ticks at a constant error of 10. Raw sums to 50. Rate sums to
  // 10 * 0.001 + 10 * 0.01 * 4 = 0.41: the first tick has no previous
  // timestamp to measure against, so its dt lands on the 1 ms floor.
  const double raw = runTicks(false, 5);
  const double rate = runTicks(true, 5);
  std::printf("   raw I = %.17g, rate I at 10 ms = %.17g, ratio %.17g\n", raw,
              rate, rate / raw);
  CHECK_EQ(raw, 50.0);
  CHECK_NEAR(rate, 10.0 * 0.001 + 10.0 * 0.01 * 4.0, 1e-12);
}

/**
 * @brief Two update() calls in the same millisecond must not divide by zero.
 */
void testZeroDtIsFinite() {
  std::printf("-- dt == 0 guard\n");

  PID pid(0.0, 0.0, 2.0);
  pid.setArrive(false);
  pid.setUseDt(true);
  pid.setTarget(10.0);

  g_fake_ms = 500;
  pid.update(0.0);
  const double same_ms = pid.update(1.0);  // dt measured as 0
  std::printf("   dt used = %.17g s, output = %.17g\n", pid.getLastDt(), same_ms);
  CHECK(std::isfinite(same_ms));
  // Clamped to the 1 ms floor: kd * -1 / 0.001.
  CHECK_EQ(pid.getLastDt(), 0.001);
  CHECK_EQ(same_ms, -2000.0);

  // An explicitly supplied zero is clamped the same way.
  PID explicit_pid(0.0, 0.0, 2.0);
  explicit_pid.setArrive(false);
  explicit_pid.setUseDt(true);
  explicit_pid.setTarget(10.0);
  explicit_pid.update(0.0, 0.0 * millisecond);
  const double zero_dt = explicit_pid.update(1.0, 0.0 * millisecond);
  CHECK(std::isfinite(zero_dt));
  CHECK_EQ(zero_dt, -2000.0);
}

/**
 * @brief A long pause is clamped to the dt ceiling instead of dumping one huge
 * timestep into the loop.
 */
void testHugeDtIsClamped() {
  std::printf("-- huge dt guard\n");

  PID pid(0.0, 1.0, 0.0);
  pid.setArrive(false);
  pid.setSmallBigErrorTolerance(0.0, 0.0);
  pid.setIntegralMax(0.0);
  pid.setUseDt(true);
  pid.setTarget(10.0);

  g_fake_ms = 0;
  pid.update(0.0);
  g_fake_ms = 5000;  // a 5 second pause
  const double out = pid.update(0.0);
  std::printf("   dt used after a 5 s pause = %.17g s, integral = %.17g\n",
              pid.getLastDt(), out);
  CHECK_EQ(pid.getLastDt(), 0.1);
  // First tick contributed 10 * 0.001 (the floor), this one 10 * 0.1.
  CHECK_NEAR(out, 10.0 * 0.001 + 10.0 * 0.1, 1e-12);

  // The bounds are configurable.
  PID wide(0.0, 0.0, 2.0);
  wide.setArrive(false);
  wide.setUseDt(true);
  wide.setDtRange(1.0 * millisecond, 1.0 * second);
  wide.setTarget(10.0);
  g_fake_ms = 0;
  wide.update(0.0);
  g_fake_ms = 5000;
  wide.update(1.0);
  CHECK_EQ(wide.getLastDt(), 1.0);
}

/**
 * @brief setUseDt(false) never reads the clock for a timestep, so getLastDt()
 * stays 0 and the numbers are the historical ones.
 */
void testDtIgnoredWhenOff() {
  std::printf("-- supplied dt ignored while use_dt is false\n");

  PID pid(0.0, 0.0, 2.0);
  pid.setArrive(false);
  pid.setTarget(10.0);
  pid.update(0.0, 10.0 * millisecond);
  const double out = pid.update(1.0, 10.0 * millisecond);
  CHECK_EQ(out, -2.0);
  CHECK_EQ(pid.getLastDt(), 0.0);
}

/**
 * @brief A supplied dt still stamps the clock, so a later measured tick sees
 * its own interval and not the whole run.
 */
void testSuppliedDtStampsClock() {
  std::printf("-- supplied dt still stamps the clock\n");

  PID pid(0.0, 0.0, 2.0);
  pid.setArrive(false);
  pid.setUseDt(true);
  pid.setTarget(10.0);

  g_fake_ms = 1000;
  pid.update(0.0, 10.0 * millisecond);
  g_fake_ms = 1010;
  pid.update(0.0, 10.0 * millisecond);
  g_fake_ms = 1020;
  pid.update(1.0);  // measured: should be 10 ms, not 1020 ms clamped to 100
  std::printf("   measured dt after two supplied-dt ticks = %.17g s\n",
              pid.getLastDt());
  CHECK_NEAR(pid.getLastDt(), 0.01, 1e-12);

  // Flipping rate mode on mid-run has no previous stamp, so the first rate tick
  // lands on the floor rather than charging itself the whole run so far.
  PID flipped(0.0, 0.0, 2.0);
  flipped.setArrive(false);
  flipped.setTarget(10.0);
  g_fake_ms = 0;
  flipped.update(0.0);
  g_fake_ms = 3000;
  flipped.update(0.0);
  flipped.setUseDt(true);
  g_fake_ms = 3010;
  flipped.update(1.0);
  std::printf("   first dt after setUseDt(true) mid-run = %.17g s\n",
              flipped.getLastDt());
  CHECK_EQ(flipped.getLastDt(), 0.001);
  g_fake_ms = 3020;
  flipped.update(2.0);
  CHECK_NEAR(flipped.getLastDt(), 0.01, 1e-12);
}

/**
 * @brief An inverted dt range collapses to the floor instead of quietly
 * ignoring the ceiling.
 */
void testInvertedDtRange() {
  std::printf("-- inverted setDtRange\n");

  PID pid(0.0, 0.0, 2.0);
  pid.setArrive(false);
  pid.setUseDt(true);
  pid.setDtRange(100.0 * millisecond, 1.0 * millisecond);
  pid.setTarget(10.0);
  g_fake_ms = 0;
  pid.update(0.0);
  g_fake_ms = 10;
  pid.update(1.0);
  std::printf("   dt with range (100 ms, 1 ms) = %.17g s\n", pid.getLastDt());
  CHECK_EQ(pid.getLastDt(), 0.1);
}

/**
 * @brief setIntegralMax(0) means unbounded, not zero. motion.cpp relies on it.
 */
void testIntegralMaxZeroIsUnbounded() {
  std::printf("-- setIntegralMax(0) is unbounded\n");

  auto sumAfter = [](double integral_max, int ticks) {
    PID pid(0.0, 1.0, 0.0);
    pid.setArrive(false);
    pid.setSmallBigErrorTolerance(0.0, 0.0);
    pid.setIntegralMax(integral_max);
    pid.setTarget(100.0);
    double out = 0.0;
    g_fake_ms = 0;
    for (int i = 0; i < ticks; ++i) {
      out = pid.update(0.0);
      g_fake_ms += 10;
    }
    return out;
  };

  const double unbounded = sumAfter(0.0, 20);
  const double clamped = sumAfter(500.0, 20);
  std::printf("   integral_max 0 -> %.17g, integral_max 500 -> %.17g\n",
              unbounded, clamped);
  CHECK_EQ(unbounded, 2000.0);
  CHECK_EQ(clamped, 500.0);
}

/**
 * @brief setIntegralRange(0) leaves the gate off, so the integral accumulates
 * at any error.
 */
void testIntegralRangeZeroDisablesGate() {
  std::printf("-- setIntegralRange(0) disables the gate\n");

  auto sumAfter = [](double range) {
    PID pid(0.0, 1.0, 0.0);
    pid.setArrive(false);
    pid.setSmallBigErrorTolerance(0.0, 0.0);
    pid.setIntegralMax(0.0);
    pid.setIntegralRange(range);
    pid.setTarget(100.0);
    double out = 0.0;
    g_fake_ms = 0;
    for (int i = 0; i < 5; ++i) {
      out = pid.update(0.0);
      g_fake_ms += 10;
    }
    return out;
  };

  // Error is a constant 100. Range 0 accumulates; range 10 blocks it entirely.
  std::printf("   range 0 -> %.17g, range 10 -> %.17g\n", sumAfter(0.0),
              sumAfter(10.0));
  CHECK_EQ(sumAfter(0.0), 500.0);
  CHECK_EQ(sumAfter(10.0), 0.0);
}

/**
 * @brief small_error_tolerance also gates the integral; (0, 0) keeps ki alive.
 */
void testSmallToleranceGatesIntegral() {
  std::printf("-- setSmallBigErrorTolerance(0, 0) keeps ki alive\n");

  auto sumAfter = [](double small_tol) {
    PID pid(0.0, 1.0, 0.0);
    pid.setArrive(false);
    pid.setIntegralMax(0.0);
    pid.setSmallBigErrorTolerance(small_tol, small_tol);
    pid.setTarget(0.5);  // error of 0.5, inside the default tolerance of 1
    double out = 0.0;
    g_fake_ms = 0;
    for (int i = 0; i < 4; ++i) {
      out = pid.update(0.0);
      g_fake_ms += 10;
    }
    return out;
  };

  const double defaulted = sumAfter(1.0);
  const double alive = sumAfter(0.0);
  std::printf("   small_tol 1 -> %.17g (ki disabled), small_tol 0 -> %.17g\n",
              defaulted, alive);
  CHECK_EQ(defaulted, 0.0);
  CHECK_EQ(alive, 2.0);
}

/**
 * @brief With the constructor defaults the small window latches at t = 100 ms,
 * and a tick outside tolerance restarts it.
 */
void testArrivalLatchTiming() {
  std::printf("-- arrival latch timing\n");

  PID pid(0.5, 0.0, 0.0);
  pid.setTarget(0.0);
  pid.setHoldOutput(true);  // so the latch is visible without zeroing output

  g_fake_ms = 0;
  int latch_ms = -1;
  for (int i = 0; i < 20 && latch_ms < 0; ++i) {
    pid.update(0.0);  // error 0, inside small_error_tolerance of 1
    if (pid.targetArrived()) {
      latch_ms = static_cast<int>(g_fake_ms);
    }
    g_fake_ms += 10;
  }
  std::printf("   latched at t = %d ms (small_error_duration 100)\n", latch_ms);
  CHECK_EQ(static_cast<double>(latch_ms), 100.0);

  // An out-of-tolerance tick restarts the window: the latch then needs another
  // full 100 ms.
  PID restart(0.5, 0.0, 0.0);
  restart.setTarget(0.0);
  restart.setHoldOutput(true);
  g_fake_ms = 0;
  for (int i = 0; i < 9; ++i) {  // 0..80 ms inside tolerance
    restart.update(0.0);
    g_fake_ms += 10;
  }
  CHECK(!restart.targetArrived());
  restart.update(50.0);  // 90 ms: way outside, window restarts here
  g_fake_ms += 10;
  int restart_ms = -1;
  for (int i = 0; i < 20 && restart_ms < 0; ++i) {
    restart.update(0.0);
    if (restart.targetArrived()) {
      restart_ms = static_cast<int>(g_fake_ms);
    }
    g_fake_ms += 10;
  }
  std::printf("   after a break at 90 ms, relatched at t = %d ms\n", restart_ms);
  CHECK_EQ(static_cast<double>(restart_ms), 190.0);

  // setArrive(false) never latches at all.
  PID never(0.5, 0.0, 0.0);
  never.setTarget(0.0);
  never.setArrive(false);
  g_fake_ms = 0;
  for (int i = 0; i < 100; ++i) {
    never.update(0.0);
    g_fake_ms += 10;
  }
  CHECK(!never.targetArrived());

  // setUseDt(true) must not move the latch tick.
  PID rate(0.5, 0.0, 0.0);
  rate.setTarget(0.0);
  rate.setHoldOutput(true);
  rate.setUseDt(true);
  g_fake_ms = 0;
  int rate_ms = -1;
  for (int i = 0; i < 20 && rate_ms < 0; ++i) {
    rate.update(0.0);
    if (rate.targetArrived()) {
      rate_ms = static_cast<int>(g_fake_ms);
    }
    g_fake_ms += 10;
  }
  std::printf("   with setUseDt(true), latched at t = %d ms\n", rate_ms);
  CHECK_EQ(static_cast<double>(rate_ms), 100.0);
}

// ---------------------------------------------------------------------------
// The typed instantiation.
// ---------------------------------------------------------------------------

using LengthPid = PIDController<QLength, QVoltage>;

/** @brief True when P's gains accept a value of type G. */
template <typename P, typename G>
concept GainAccepts = requires(P& pid, G gain) { pid.setCoefficient(gain, gain, gain); };

/** @brief True when P's setpoint accepts a value of type T. */
template <typename P, typename T>
concept TargetAccepts = requires(P& pid, T value) { pid.setTarget(value); };

// kp for a length-in / volts-out loop is volts per metre. Nothing else fits.
static_assert(GainAccepts<LengthPid, decltype(volt / metre)>);
static_assert(!GainAccepts<LengthPid, double>);
static_assert(!GainAccepts<LengthPid, QVoltage>);
static_assert(!GainAccepts<LengthPid, QLength>);
// The dimensionless alias still takes bare numbers, as every caller expects.
static_assert(GainAccepts<PID, double>);

/**
 * @brief PIDController<QLength, QVoltage> computes the same numbers the
 * dimensionless one does, with the units carried through.
 */
void testTypedInstantiation() {
  std::printf("-- PIDController<QLength, QVoltage>\n");

  LengthPid pid(0.5 * volt / inch, 0.0 * volt / inch, 2.0 * volt / inch);
  pid.setSmallBigErrorTolerance(1.0 * inch, 3.0 * inch);
  pid.setIntegralMax(0.0 * volt);
  pid.setTarget(24_in);

  g_fake_ms = 0;
  const QVoltage first = pid.update(0_in);
  g_fake_ms = 10;
  const QVoltage second = pid.update(2_in);

  std::printf("   first = %.17g V, second = %.17g V\n", first.volts(),
              second.volts());
  // 0.5 V/in * 24 in = 12 V; then 0.5 * 22 + 2.0 * (22 - 24) = 11 - 4 = 7 V.
  CHECK_NEAR(first.volts(), 12.0, 1e-9);
  CHECK_NEAR(second.volts(), 7.0, 1e-9);

  // Same run through the dimensionless controller, in inches and volts.
  PID plain(0.5, 0.0, 2.0);
  plain.setIntegralMax(0.0);
  plain.setTarget(24.0);
  g_fake_ms = 0;
  const double plain_first = plain.update(0.0);
  g_fake_ms = 10;
  const double plain_second = plain.update(2.0);
  CHECK_NEAR(first.volts(), plain_first, 1e-9);
  CHECK_NEAR(second.volts(), plain_second, 1e-9);

  // A typed controller starts with zero tolerances, so it does not latch
  // `arrived` 100 ms into a 24 inch motion the way inherited defaults of
  // "1 metre / 3 metres" would have.
  LengthPid defaulted(0.5 * volt / inch, 0.0 * volt / inch, 2.0 * volt / inch);
  defaulted.setTarget(24_in);
  QLength process = 0_in;
  g_fake_ms = 0;
  for (int i = 0; i < 60; ++i) {
    const QVoltage out = defaulted.update(process);
    process += 0.02 * out.volts() * inch;
    g_fake_ms += 10;
  }
  std::printf("   typed defaults after 600 ms: error = %.4f in, arrived = %d\n",
              (24_in - process).in(), static_cast<int>(defaulted.targetArrived()));
  CHECK(!defaulted.targetArrived());
  // Still driving, and it has covered real ground rather than latching at
  // 100 ms with 21.8 inches to go.
  CHECK((24_in - process).in() < 15.0);
  CHECK(defaulted.getOutput().volts() > 0.0);

  // An angle loop is a different type, so a QAngle target cannot be handed to
  // the length loop by accident.
  static_assert(TargetAccepts<LengthPid, QLength>);
  static_assert(!TargetAccepts<LengthPid, QAngle>);
  static_assert(!TargetAccepts<LengthPid, double>);

  PIDController<QAngle, QVoltage> turn(0.3 * volt / degree, 0.0 * volt / degree,
                                       1.5 * volt / degree);
  turn.setSmallBigErrorTolerance(1.0 * degree, 3.0 * degree);
  turn.setIntegralMax(0.0 * volt);
  turn.setTarget(90_deg);
  g_fake_ms = 0;
  const QVoltage turn_out = turn.update(0_deg);
  std::printf("   turn first = %.17g V\n", turn_out.volts());
  CHECK_NEAR(turn_out.volts(), 27.0, 1e-9);
}

}  // namespace

int main() {
  mclib::time::ScopedClock clock(fakeClock);

  testBitIdenticalDefaults();
  testUseDtScalesDerivative();
  testUseDtScalesIntegral();
  testZeroDtIsFinite();
  testHugeDtIsClamped();
  testDtIgnoredWhenOff();
  testSuppliedDtStampsClock();
  testInvertedDtRange();
  testIntegralMaxZeroIsUnbounded();
  testIntegralRangeZeroDisablesGate();
  testSmallToleranceGatesIntegral();
  testArrivalLatchTiming();
  testTypedInstantiation();

  return mclib::test::summary("pid");
}
