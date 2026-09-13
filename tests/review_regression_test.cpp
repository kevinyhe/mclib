#include "mclib/command/commandScheduler.h"
#include "mclib/command/conditionalCommand.h"
#include "mclib/command/functionalCommand.h"
#include "mclib/command/parallelCommandGroup.h"
#include "mclib/command/parallelRaceGroup.h"
#include "mclib/command/sequence.h"
#include "mclib/chassis/chassis_math.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/mechanism/position_mechanism.hpp"
#include "mclib/mechanism/homing_mechanism.hpp"
#include "mclib/mechanism/velocity_mechanism.hpp"
#include "mclib/pid.hpp"
#include "mclib/time.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"

namespace {
std::uint32_t now_ms = 0;
std::uint32_t fakeClock() { return now_ms; }
struct TracedSubsystem : Subsystem {
  int ticks = 0, stops = 0;
  void periodic() override { ++ticks; }
  void onDisabled() override { ++stops; }
};
}

int main() {
  mclib::time::ScopedClock clock(fakeClock);
  {
    Subsystem sub;
    int outgoing_ends = 0, incoming_ends = 0;
    FunctionalCommand replacement([] {}, [] {}, [](bool) {}, [] { return false; }, {&sub});
    FunctionalCommand outgoing([] {}, [] {}, [&](bool) {
      ++outgoing_ends; replacement.schedule();
    }, [] { return false; }, {&sub});
    FunctionalCommand incoming([] {}, [] {}, [&](bool) { ++incoming_ends; },
                               [] { return false; }, {&sub});
    outgoing.schedule(); incoming.schedule();
    CHECK_EQ(outgoing_ends, 1);
    CHECK_EQ(incoming_ends, 1);
    CHECK(!incoming.scheduled());
    CHECK(replacement.scheduled());
    CHECK(CommandScheduler::getRequiring(&sub).value() == &replacement);
    replacement.cancel();

    // A command can cancel itself during initialize, before execute ever runs.
    int executions = 0, ends = 0;
    FunctionalCommand* self = nullptr;
    FunctionalCommand self_cancel([&] { self->cancel(); }, [&] { ++executions; },
        [&](bool interrupted) { CHECK(interrupted); ++ends; },
        [] { return false; }, {&sub});
    self = &self_cancel;
    self_cancel.schedule(); CommandScheduler::run();
    CHECK(!self_cancel.scheduled());
    CHECK_EQ(ends, 1); CHECK_EQ(executions, 0);
  }
  {
    Subsystem a, b;
    bool choose_a = false;
    int predicate_calls = 0, starts = 0;
    FunctionalCommand owner([] {}, [] {}, [](bool) {}, [] { return false; }, {&a});
    FunctionalCommand branch_a([&] { ++starts; }, [] {}, [](bool) {}, [] { return false; }, {&a});
    FunctionalCommand branch_b([] {}, [] {}, [](bool) {}, [] { return false; }, {&b});
    ConditionalCommand conditional(&branch_a, &branch_b, [&] { ++predicate_calls; return choose_a; });
    CHECK_EQ(conditional.getRequirements().size(), 2);
    CHECK_EQ(predicate_calls, 0);
    FunctionalCommand change([&] { choose_a = true; }, [] {}, [](bool) {}, [] { return true; }, {});
    Sequence sequence({&change, &conditional});
    owner.schedule(); sequence.schedule(); CommandScheduler::run();
    CHECK_EQ(starts, 1); CHECK_EQ(predicate_calls, 1);
    CHECK(!owner.scheduled());
    sequence.cancel();
  }
  {
    int fast_ends = 0, slow_ends = 0;
    bool fast_interrupted = false, slow_interrupted = false;
    FunctionalCommand fast([] {}, [] {}, [&](bool i) { ++fast_ends; fast_interrupted = i; },
                           [] { return true; }, {});
    FunctionalCommand slow([] {}, [] {}, [&](bool i) { ++slow_ends; slow_interrupted = i; },
                           [] { return false; }, {});
    ParallelRaceGroup race({&fast, &slow});
    for (int n = 1; n <= 2; ++n) {
      race.schedule(); CommandScheduler::run();
      CHECK_EQ(fast_ends, n); CHECK_EQ(slow_ends, n);
      CHECK(!fast_interrupted); CHECK(slow_interrupted);
    }
    race.schedule(); race.cancel();
    CHECK_EQ(fast_ends, 3); CHECK_EQ(slow_ends, 3);
    CHECK(fast_interrupted); CHECK(slow_interrupted);
    ParallelCommandGroup all({&fast, &slow});
    all.schedule(); CommandScheduler::run(); all.cancel();
    CHECK_EQ(fast_ends, 4); CHECK_EQ(slow_ends, 4);
    CHECK(!fast_interrupted); CHECK(slow_interrupted);
  }
  {
    TracedSubsystem sub;
    sub.registerSelf();
    int executions = 0, ends = 0;
    FunctionalCommand replacement([] {}, [] {}, [](bool) {}, [] { return false; }, {&sub});
    FunctionalCommand active([] {}, [&] { ++executions; }, [&](bool interrupted) {
      CHECK(interrupted); ++ends; replacement.schedule();
    }, [] { return false; }, {&sub});
    active.schedule();
    mclib::test::setCompetitionStatus(1);
    CommandScheduler::run(); CommandScheduler::run();
    CHECK(!active.scheduled()); CHECK(!replacement.scheduled());
    CHECK_EQ(executions, 0); CHECK_EQ(ends, 1); CHECK_EQ(sub.ticks, 0);
    CHECK_EQ(sub.stops, 2);
    mclib::test::setCompetitionStatus(0);
    CommandScheduler::run();
    CHECK_EQ(executions, 0); CHECK_EQ(sub.ticks, 1);
    CommandScheduler::unregisterSubsystem(&sub);
  }
  {
    double sensor = NAN, voltage = 5;
    mclib::mechanism::VelocityMechanism velocity([&] { return sensor; }, [&](double v) { voltage = v; });
    velocity.setTargetRpm(300);
    for (int i = 0; i < 30; ++i) { velocity.periodic(); now_ms += 10; }
    CHECK(!velocity.atSpeed()); CHECK(velocity.hasSensorFault()); CHECK_EQ(voltage, 0);
    sensor = 300;
    for (int i = 0; i < 30; ++i) { velocity.periodic(); now_ms += 10; }
    CHECK(velocity.atSpeed()); CHECK(!velocity.hasSensorFault());
    velocity.onDisabled(); CHECK_EQ(voltage, 0); CHECK_EQ(velocity.getTargetRpm(), 0);
    CHECK(!velocity.atSpeed());
    mclib::mechanism::PositionMechanism position([&] { return sensor; }, [&](double v) { voltage = v; });
    sensor = NAN; position.moveTo(90); position.periodic();
    CHECK(!position.atTarget()); CHECK(position.hasSensorFault()); CHECK_EQ(voltage, 0);
    position.setManualVoltage(4); position.periodic(); CHECK_EQ(voltage, 4);
    position.onDisabled(); CHECK_EQ(voltage, 0);
    position.periodic(); CHECK_EQ(voltage, 0);
  }
  {
    using namespace mclib::control;
    resetOdometry({1, 2, 0});
    OdometrySample sample;
    odometryTick(sample);
    CHECK(correctOdometryPosition(8, 9));
    odometryTick(sample);
    CHECK_EQ(robotState().pose().x, 8); CHECK_EQ(robotState().pose().y, 9);
    CHECK(!correctOdometryPosition(NAN, 12));
    CHECK_EQ(robotState().pose().y, 9);
    sample.left_deg = sample.right_deg = 360;
    odometryTick(sample);
    CHECK_NEAR(robotState().pose().y,
        9 + getOdometryConfig().drive_inches_per_revolution.in(), 1e-10);
    CHECK_EQ(robotState().pose().x, 8);
  }
  {
    using mclib::chassis_math::encoderHeadingDeg;
    CHECK_EQ(encoderHeadingDeg(12, 12, 10), 0);
    CHECK_NEAR(encoderHeadingDeg(5 * std::acos(-1.0), 0, 10), 90, 1e-10);
    CHECK_NEAR(encoderHeadingDeg(0, 5 * std::acos(-1.0), 10), -90, 1e-10);
    CHECK_NEAR(encoderHeadingDeg(40 * std::acos(-1.0), 0, 10), 720, 1e-10);
    CHECK(std::isnan(encoderHeadingDeg(1, 2, 0)));
    CHECK(std::isnan(encoderHeadingDeg(NAN, 2, 10)));
  }
  {
    double voltage = 0;
    int resets = 0;
    mclib::mechanism::HomingMechanismConfig config;
    config.velocity_threshold_rpm = 5;
    mclib::mechanism::HomingMechanism homing(config, [&](double v) { voltage = v; });
    homing.setVelocitySource([] { return NAN; });
    homing.setPositionReset([&] { ++resets; });
    homing.startHoming(); homing.periodic();
    CHECK(homing.hasFailed()); CHECK(!homing.isHomed());
    CHECK_EQ(resets, 0); CHECK_EQ(voltage, 0);
  }
  {
    PID translation(1, 0, 0), heading(1, 0, 0);
    for (auto* pid : {&translation, &heading}) {
      pid->setLatchArrival(false); pid->setHoldOutput(true);
      pid->setSmallBigErrorDuration(20, 20);
      pid->setDerivativeTolerance(100);
    }
    translation.update(0); heading.update(10); now_ms += 30;
    translation.update(0); heading.update(10);
    CHECK(translation.targetArrived()); CHECK(!heading.targetArrived());
    translation.update(10); heading.update(0); now_ms += 30;
    translation.update(10); heading.update(0);
    CHECK(!translation.targetArrived()); CHECK(heading.targetArrived());
    translation.update(NAN);
    CHECK(!translation.targetArrived()); CHECK_EQ(translation.getOutput(), 0);
  }
  return mclib::test::summary("review regressions");
}
