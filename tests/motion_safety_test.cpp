// Exercise the real blocking loops against deterministic hardware and time.
#include "mclib/control/motion.hpp"
#include "mclib/control/chassis_io.hpp"
#include "mclib/control/motion_config.hpp"
#include "mclib/control/odometry.hpp"
#include "mclib/control/robot_state.hpp"
#include "mclib/time.hpp"
#include "support/host_pros.hpp"
#include "test_assert.hpp"
#include <functional>
#include <limits>

namespace {
std::uint32_t now_ms = 0;
std::function<void()> on_delay;
std::uint32_t fakeClock() { return now_ms; }
struct FakeDrive : mclib::control::DriveHardware {
  double left = 0, right = 0, heading = 0, encoder = 0;
  double current = 0, velocity = 100;
  bool invalid_output = false, missing_telemetry = false;
  mclib::units::DriveGeometry geometry{
      mclib::units::Wheel::fromDiameter(4 * mclib::units::inch),
      12 * mclib::units::inch, 1};
  void setDriveVoltage(double l, double r) override {
    left = l; right = r;
    invalid_output |= !std::isfinite(l) || !std::isfinite(r);
  }
  void setSideVoltage(bool l, double v) override {
    (l ? left : right) = v;
    invalid_output |= !std::isfinite(v);
  }
  void brakeDrive(mclib::device::BrakeMode) override { left = right = 0; }
  void brakeSide(bool l, mclib::device::BrakeMode) override { (l ? left : right) = 0; }
  void tareDrive() override { encoder = 0; }
  double leftPositionDeg() override { return encoder; }
  double rightPositionDeg() override { return encoder; }
  double headingDeg() override { return heading; }
  void setHeadingDeg(double v) override { heading = v; }
  std::vector<double> driveCurrentsMa() override {
    return missing_telemetry ? std::vector<double>{} : std::vector<double>{current};
  }
  std::vector<double> driveVelocitiesRpm() override { return {velocity}; }
  const mclib::units::DriveGeometry& driveGeometry() const override { return geometry; }
};
}
namespace pros {
extern "C" std::uint32_t millis() { return now_ms; }
extern "C" void delay(std::uint32_t ms) { now_ms += ms; if (on_delay) on_delay(); }
}

int main() {
  using namespace mclib::units::literals;
  using namespace mclib::control;
  mclib::time::ScopedClock clock(fakeClock);
  FakeDrive hw;
  bindDrive(&hw);
  const std::vector<std::function<void()>> motions{
    [] { turnToAngle(90_deg, 100_ms, false); },
    [] { driveTo(24_in, 100_ms, false); },
    [] { curveCircle(90_deg, 24_in, 100_ms, false); },
    [] { curveCircleReverse(90_deg, 24_in, 100_ms, false); },
    [] { swing(90_deg, 1, 100_ms, false); },
    [] { turnToPoint(24_in, 24_in, 1, 100_ms); },
    [] { moveToPoint(0_in, 24_in, 1, 100_ms, false); },
    [] { boomerang(0_in, 24_in, 1, 90_deg, 0.5, 100_ms, false); }
  };
  for (const auto& motion : motions) {
    for (int failure = 0; failure < 5; ++failure) {
      now_ms = 0;
      hw.heading = hw.encoder = 0;
      hw.left = hw.right = 6;
      clearCancel();
      mclib::test::setCompetitionStatus(0);
      resetOdometry({0, 0, 0});
      robotState().setCorrectAngleDeg(0);
      on_delay = [&] {
        CHECK(robotState().motionTelemetry().phase != MotionPhase::Idle);
        if (failure == 1) requestCancel();
        if (failure == 2) hw.heading = NAN;
        if (failure == 3) hw.encoder = NAN;
        if (failure == 4) mclib::test::setCompetitionStatus(1);
      };
      motion();
      CHECK_EQ(hw.left, 0);
      CHECK_EQ(hw.right, 0);
      CHECK(!hw.invalid_output);
      CHECK(!robotState().isTurning());
      CHECK_EQ(robotState().prevLeftOutput(), 0);
      CHECK(now_ms <= 100);
      const auto telemetry = robotState().motionTelemetry();
      CHECK(telemetry.phase == MotionPhase::Idle);
      CHECK(std::isnan(telemetry.target_heading_deg));
      CHECK(std::isnan(telemetry.carrot_x));
      CHECK(std::isnan(telemetry.drive_volts));
    }
  }
  on_delay = {};
  clearCancel();
  mclib::test::setCompetitionStatus(0);
  hw.heading = hw.encoder = 0;
  resetOdometry({1, 2, 0});
  now_ms = 0;
  CHECK(!wallReset(50_in, 60_in, 90_deg, 3_V, 100_ms));
  CHECK_EQ(now_ms, 100);
  CHECK_EQ(robotState().pose().x, 1);
  CHECK_EQ(hw.heading, 0);
  // The default threshold equals the V5 current limit; equality must count.
  hw.current = 2500; hw.velocity = 0;
  CHECK(wallReset(50_in, 60_in, 90_deg, 3_V, 1_s));
  CHECK_EQ(robotState().pose().x, 50);
  CHECK_NEAR(hw.heading, 90, 1e-10);
  hw.missing_telemetry = true;
  CHECK(!wallReset(70_in, 80_in, 0_deg, 3_V, 1_s));
  CHECK_EQ(robotState().pose().x, 50);
  hw.missing_telemetry = false;
  hw.velocity = NAN;
  CHECK(!wallReset(70_in, 80_in, 0_deg, 3_V, 1_s));
  CHECK_EQ(robotState().pose().x, 50);
  hw.velocity = 0;
  hw.current = std::numeric_limits<std::int32_t>::max();
  CHECK(!wallReset(70_in, 80_in, 0_deg, 3_V, 1_s));
  CHECK_EQ(robotState().pose().x, 50);

  // A genuine crossing still carries momentum into the next chained motion.
  hw.heading = 0; hw.encoder = 0;
  now_ms = 0;
  on_delay = [&] { hw.heading = 100; };
  turnToAngle(90_deg, 100_ms, false);
  CHECK(std::abs(hw.left) > 0);
  CHECK(std::abs(hw.right) > 0);
  on_delay = {};
  // Invalid targets must not hang normalization or energize the drivetrain.
  turnToAngle(QAngle::fromBase(INFINITY), 100_ms, false);
  CHECK_EQ(hw.left, 0);
  CHECK_EQ(hw.right, 0);
  CHECK(std::isnan(normalizeTarget(INFINITY)));

  // Settling 1.2 in short is inside the configured 1.5 in PID band. The
  // stopped point move must return on settlement, not wait forever at 0 V
  // for a separate 1 in crossing condition.
  now_ms = 0;
  hw.heading = hw.encoder = 0;
  resetOdometry({0, 22.8, 0});
  moveToPoint(0_in, 24_in, 1, 2_s);
  CHECK(now_ms < 1000);
  CHECK_EQ(hw.left, 0);

  // A chained move in the same settle band keeps a live output until the
  // actual line crossing; PID settlement must not latch it off.
  now_ms = 0;
  resetOdometry({0, 22.8, 0});
  bool drove_after_settle = false;
  on_delay = [&] {
    if (now_ms > 300 && now_ms < 500)
      drove_after_settle |= hw.left > 0 && hw.right > 0;
    if (now_ms >= 500) resetOdometry({0, 24.1, 0});
  };
  moveToPoint(0_in, 24_in, 1, 2_s, false);
  CHECK(drove_after_settle);
  CHECK(now_ms < 1000);
  CHECK(hw.left > 0 && hw.right > 0);
  on_delay = {};

  // A stationary chassis five inches from the pose is not a completed
  // boomerang. It must continue trying until its deadline, then stop safely.
  now_ms = 0;
  resetOdometry({0, 19.1, 0});
  boomerang(0_in, 24_in, 1, 0_deg, 0.5, 1_s);
  CHECK_EQ(now_ms, 1000);
  CHECK_EQ(hw.left, 0);
  CHECK_EQ(hw.right, 0);

  // Boomerang direction applies to translation, not the signs of individual
  // wheels during a turn. Slewing an already mixed (+12,-12) pivot used to
  // produce (+1,-12), sending a Forward move backwards on its first tick.
  const MotionConfig direction_config = motionConfig();
  auto first_boomerang_output = [&](double x, double y, int direction,
                                    double heading, double lead,
                                    QVoltage floor = -1_V,
                                    QVoltage cap = 12_V, double previous = 0,
                                    bool stop = true) {
    now_ms = 0;
    hw.heading = hw.encoder = 0;
    resetOdometry({0, 0, 0});
    robotState().setPrevOutputs(previous, previous);
    clearCancel();
    double left = NAN, right = NAN;
    on_delay = [&] {
      left = hw.left;
      right = hw.right;
      requestCancel();
    };
    boomerang(x * 1_in, y * 1_in, direction, heading * 1_deg,
              lead, 100_ms, stop, cap, true, floor);
    on_delay = {};
    clearCancel();
    CHECK_EQ(hw.left, 0);
    CHECK_EQ(hw.right, 0);
    return std::pair<double, double>{left, right};
  };
  for (int direction : {-1, 1}) {
    for (int side : {-1, 1}) {
      // A saturated yaw request stays a pure pivot in either travel direction.
      const auto pivot = first_boomerang_output(
          side * direction * 24, direction * 24, direction, -side * 90, .5);
      CHECK_NEAR(pivot.first + pivot.second, 0, 1e-10);
      CHECK(side * (pivot.first - pivot.second) > 0);

      // A partial turn preserves the mean-voltage acceleration bound too.
      const auto curve = first_boomerang_output(
          side * direction * 24, direction * 48, direction, 0, 0);
      CHECK_NEAR((curve.first + curve.second) / 2, direction, 1e-10);
      CHECK(side * (curve.first - curve.second) > 0);
    }
    const auto straight = first_boomerang_output(0, direction * 24, direction, 0, 0);
    CHECK_NEAR(straight.first, direction, 1e-10);
    CHECK_NEAR(straight.second, direction, 1e-10);

    // Chaining keeps the configured floor outside its endpoint band; an
    // explicit zero still disables that behavior.
    const auto creep = first_boomerang_output(
        0, direction * 2, direction, 0, 0, -1_V, 12_V, direction * 1.5, false);
    CHECK_NEAR(creep.first, direction * 1.5, 1e-10);
    CHECK_NEAR(creep.second, direction * 1.5, 1e-10);
    const auto no_floor = first_boomerang_output(
        0, direction * 2, direction, 0, 0, 0_V, 12_V, 0, false);
    CHECK_NEAR(no_floor.first, direction * .8, 1e-10);
    CHECK_NEAR(no_floor.second, direction * .8, 1e-10);

    // Unequal rates catch swapping reverse acceleration with deceleration.
    // Chained moves retain their actual translation baseline and both limits.
    MotionConfig distinct_slew = direction_config;
    distinct_slew.slew = {1, 2, 3, 4};
    setMotionConfig(distinct_slew);
    for (bool stop : {false, true}) {
      const auto acceleration = first_boomerang_output(
          0, direction * 24, direction, 0, 0, -1_V, 12_V, direction * 4, stop);
      const double accelerating_mean = direction > 0 ? 5 : -7;
      CHECK_NEAR(acceleration.first, accelerating_mean, 1e-10);
      CHECK_NEAR(acceleration.second, accelerating_mean, 1e-10);
      const auto deceleration = first_boomerang_output(
          0, direction * 2, direction, 0, 0, -1_V, 12_V, direction * 8, stop);
      const double slowing_mean = stop ? direction * .8 : (direction > 0 ? 6 : -4);
      CHECK_NEAR(deceleration.first, slowing_mean, 1e-10);
      CHECK_NEAR(deceleration.second, slowing_mean, 1e-10);
    }
    setMotionConfig(direction_config);

    // A low yaw gain isolates the negative-cosine bug from yaw saturation.
    // With a carrot behind the requested travel hemisphere, orient first;
    // a requested translation floor must not turn that pivot into driving.
    MotionConfig weak_yaw = direction_config;
    weak_yaw.heading_pid = {.01, 0, 0};
    setMotionConfig(weak_yaw);
    const auto behind = first_boomerang_output(0, -direction * 24, direction, 0, 0, 3_V);
    CHECK_NEAR(behind.first + behind.second, 0, 1e-10);
    CHECK(std::abs(behind.first - behind.second) > 0);

    // Recovery inside the position band may oppose the selected travel
    // direction. A previous motion's larger cap cannot limit that braking.
    weak_yaw.heading_pid = {0, 0, 0};
    setMotionConfig(weak_yaw);
    const auto recovery = first_boomerang_output(
        0, -direction, direction, 0, 0, -1_V, 3_V, direction * 12);
    CHECK_NEAR(recovery.first, -direction * .4, 1e-10);
    CHECK_NEAR(recovery.second, -direction * .4, 1e-10);
    setMotionConfig(direction_config);

    // Negative mean voltage may be legitimate forward braking (and positive
    // mean reverse braking). A rapidly closing, aligned target must be able
    // to brake immediately; don't fix direction by clamping PID output signs.
    now_ms = 0;
    hw.heading = hw.encoder = 0;
    resetOdometry({0, 0, 0});
    robotState().setPrevOutputs(0, 0);
    double brake_mean = NAN;
    on_delay = [&] {
      if (now_ms == 10) resetOdometry({0, direction * 20.0, 0});
      else {
        brake_mean = (hw.left + hw.right) / 2;
        requestCancel();
      }
    };
    boomerang(0_in, direction * 24_in, direction, 0_deg, 0, 100_ms);
    CHECK_NEAR(brake_mean, -direction * 12, 1e-10);
    CHECK_EQ(hw.left, 0);
    CHECK_EQ(hw.right, 0);
    on_delay = {};
    clearCancel();
  }

  // A chained boomerang hands off inside its configured position band.
  // An infinite line three inches early is not endpoint acceptance,
  // and consuming that line's edge must not prevent a later close crossing.
  for (int direction : {-1, 1}) {
    for (int heading : {-90, 0, 90, 180}) {
      const double angle = heading * mclib::kPi / 180;
      const double along_x = direction * std::sin(angle);
      const double along_y = direction * std::cos(angle);
      auto place = [&](double along, double lateral) {
        resetOdometry({along * along_x + lateral * std::cos(angle),
                       along * along_y - lateral * std::sin(angle), angle});
      };
      for (int fixture = 0; fixture < 6; ++fixture) {
        now_ms = 0;
        hw.heading = heading;
        hw.encoder = 0;
        hw.left = direction * 5;
        hw.right = direction * 4;
        robotState().setPrevOutputs(hw.left, hw.right);
        clearCancel();
        const bool far_lateral = fixture == 0 || fixture == 3;
        place(fixture == 5 ? 0 : fixture == 2 ? -2 : -4,
              far_lateral ? 100 : 0);
        on_delay = [&] {
          if (fixture == 0) place(.1, 100);  // Cross much too far laterally.
          if (fixture == 1) place(-2.9, 0);  // Still short of the endpoint.
          if (fixture == 2) place(.1, 0);   // Start inside the old early line.
          if (fixture == 3) place(.1, now_ms < 50 ? 100 : 0);
          if (fixture == 4) place(4, 0);    // Overshot outside the endpoint band.
        };
        boomerang(0_in, 0_in, direction, heading * 1_deg, .5, 100_ms, false);
        if (fixture == 0 || fixture == 1 || fixture == 4) {
          CHECK_EQ(now_ms, 100);
          CHECK_EQ(hw.left, 0);
          CHECK_EQ(hw.right, 0);
        } else {
          CHECK_EQ(now_ms, fixture == 5 ? 0 : fixture == 2 ? 10 : 50);
          CHECK(std::abs(hw.left) + std::abs(hw.right) > 0);
          if (fixture == 5) {
            CHECK_EQ(hw.left, direction * 5);
            CHECK_EQ(hw.right, direction * 4);
          }
        }
        CHECK_NEAR(robotState().prevLeftOutput(), hw.left, 1e-10);
        CHECK_NEAR(robotState().prevRightOutput(), hw.right, 1e-10);
        CHECK(!robotState().isTurning());
        on_delay = {};
      }
    }
  }

  // Arc chaining must keep controlling inside its settlement band until
  // the outer wheel actually crosses the endpoint. Check both sides and
  // drive directions; the synthetic readings follow the requested arc.
  for (int side : {-1, 1}) {
    for (int direction : {-1, 1}) {
      now_ms = 0;
      hw.heading = hw.encoder = 0;
      resetOdometry({0, 0, 0});
      robotState().setCorrectAngleDeg(0);
      robotState().setPrevOutputs(0, 0);
      const double outer_arc = 30 * mclib::kPi / 2;
      bool drove_after_settle = false;
      on_delay = [&] {
        if (now_ms > 300 && now_ms < 500)
          drove_after_settle |= direction * (hw.left + hw.right) > 0.1;
        const double progress = outer_arc + (now_ms < 500 ? -0.5 : 0.1);
        hw.encoder = direction * progress / (4 * mclib::kPi) * 360;
        hw.heading = side * direction * 90 * progress / outer_arc;
      };
      curveCircle(side * direction * 90_deg, side * 24_in, 2_s, false);
      CHECK(drove_after_settle);
      CHECK(now_ms < 1000);
      CHECK(direction * (hw.left + hw.right) > 0);
      CHECK_NEAR(robotState().prevLeftOutput(), hw.left, 1e-12);
      CHECK_NEAR(robotState().prevRightOutput(), hw.right, 1e-12);
    }
  }
  on_delay = {};

  // Arc geometry starts at the measured heading, even when a previous
  // motion's requested heading differs from its actual settled heading.
  now_ms = 0;
  hw.heading = 30;
  hw.encoder = 0;
  robotState().setCorrectAngleDeg(-90);
  curveCircle(30_deg, 24_in, 1_s);
  CHECK(now_ms <= 100);
  CHECK_EQ(hw.left, 0);
  CHECK_EQ(hw.right, 0);

  // Travel opposite the requested direction cannot complete a chained arc.
  now_ms = 0;
  hw.heading = hw.encoder = 0;
  robotState().setCorrectAngleDeg(0);
  on_delay = [&] { hw.encoder = -60 / (4 * mclib::kPi) * 360; };
  curveCircle(90_deg, 24_in, 100_ms, false);
  CHECK_EQ(now_ms, 100);
  CHECK_EQ(hw.left, 0);
  CHECK_EQ(hw.right, 0);

  // Arc exit is an explicit outer-wheel rule. Its legacy default refuses
  // to settle 1.2 inches short, while a configured 1.5 inch band is honored.
  const MotionConfig saved_config = motionConfig();
  for (bool wider_arc_band : {false, true}) {
    MotionConfig config = saved_config;
    if (wider_arc_band) config.arc_exit.big_error = 1.5;
    setMotionConfig(config);
    now_ms = 0;
    hw.heading = hw.encoder = 0;
    robotState().setCorrectAngleDeg(0);
    on_delay = [&] {
      const double outer_arc = 30 * mclib::kPi / 2;
      hw.encoder = (outer_arc - 1.2) / (4 * mclib::kPi) * 360;
      hw.heading = 90 * (outer_arc - 1.2) / outer_arc;
    };
    curveCircle(90_deg, 24_in, 1_s);
    CHECK(wider_arc_band ? now_ms < 500 : now_ms == 1000);
    CHECK_EQ(hw.left, 0);
    CHECK_EQ(hw.right, 0);
    CHECK_EQ(robotState().prevLeftOutput(), 0);
    CHECK_EQ(robotState().prevRightOutput(), 0);
  }
  setMotionConfig(saved_config);
  on_delay = {};

  // A swing chooses its stationary tread from the measured entry heading.
  // Here the requested change is 30 -> 10 degrees even though the previous
  // motion left a zero-degree heading request in the shared state.
  for (int direction : {-1, 1}) {
    now_ms = 0;
    hw.heading = 30;
    hw.encoder = 0;
    resetOdometry({0, 0, mclib::kPi / 6});
    robotState().setCorrectAngleDeg(0);
    robotState().setPrevOutputs(3, 4);
    double initial_left = 0, initial_right = 0;
    on_delay = [&] {
      if (now_ms == 10) {
        initial_left = hw.left;
        initial_right = hw.right;
      }
      hw.heading = 10;
    };
    swing(10_deg, direction, 1_s);
    CHECK(direction < 0 ? initial_left < 0 : initial_right > 0);
    CHECK_EQ(direction < 0 ? initial_right : initial_left, 0);
    CHECK(now_ms < 1000);
    CHECK_EQ(robotState().prevLeftOutput(), 0);
    CHECK_EQ(robotState().prevRightOutput(), 0);
  }

  // Disable the voltage floor to isolate arrival detection: a chained swing
  // must keep controlling inside its settlement band until actual crossing.
  MotionConfig swing_config = saved_config;
  swing_config.min_voltage = 0_V;
  setMotionConfig(swing_config);
  for (int heading_sign : {-1, 1}) {
    for (int direction : {-1, 1}) {
      now_ms = 0;
      hw.heading = hw.encoder = 0;
      resetOdometry({0, 0, 0});
      robotState().setCorrectAngleDeg(0);
      robotState().setPrevOutputs(0, 0);
      bool drove_after_settle = false;
      on_delay = [&] {
        if (now_ms > 300 && now_ms < 500)
          drove_after_settle |= direction * (hw.left + hw.right) > 0.05;
        hw.heading = heading_sign * (now_ms < 500 ? 89.5 : 90.1);
      };
      swing(heading_sign * 90_deg, direction, 2_s, false);
      CHECK(drove_after_settle);
      CHECK(now_ms < 1000);
      CHECK(direction * (hw.left + hw.right) > 0);
      CHECK_EQ(heading_sign * direction > 0 ? hw.right : hw.left, 0);
      CHECK_NEAR(robotState().prevLeftOutput(), hw.left, 1e-12);
      CHECK_NEAR(robotState().prevRightOutput(), hw.right, 1e-12);
    }
  }
  setMotionConfig(saved_config);
  on_delay = {};

  // Telemetry follows the actual moving carrot and sampled IMU, independently
  // of the heading hold state that is only committed when the motion returns.
  now_ms = 0;
  hw.heading = hw.encoder = 0;
  robotState().clearMotionOutputs();
  robotState().setCorrectAngleDeg(-123);
  resetOdometry({0, 0, 0});
  std::vector<MotionTelemetry> observations;
  on_delay = [&] {
    observations.push_back(robotState().motionTelemetry());
    if (observations.size() == 1) resetOdometry({2, 3, 0});
    else requestCancel();
  };
  boomerang(15.5_in, 18.5_in, 1, -90_deg, .5, 1_s);
  CHECK_EQ(observations.size(), 2);
  for (std::size_t i = 0; i < observations.size(); ++i) {
    const auto& value = observations[i];
    const double x = i == 0 ? 0 : 2, y = i == 0 ? 0 : 3;
    const double remaining = std::hypot(15.5 - x, 18.5 - y);
    CHECK(value.phase == MotionPhase::Pursuit);
    CHECK_NEAR(value.remaining_in, remaining, 1e-10);
    CHECK_NEAR(value.carrot_x, 15.5 + remaining * .5, 1e-10);
    CHECK_NEAR(value.carrot_y, 18.5, 1e-10);
    CHECK_NEAR(value.target_heading_deg,
               std::atan2(value.carrot_x - x, value.carrot_y - y) * 180 / mclib::kPi,
               1e-10);
    CHECK_NEAR(value.heading_error_deg, value.target_heading_deg, 1e-10);
    CHECK(std::isfinite(value.drive_volts));
    CHECK(std::isfinite(value.yaw_volts));
  }
  CHECK(std::abs(observations[0].target_heading_deg - observations[1].target_heading_deg) > .1);
  CHECK_EQ(observations[0].slew_limited, 1);
  CHECK_EQ(observations[0].voltage_limited, 1);
  CHECK(robotState().motionTelemetry().phase == MotionPhase::Idle);
  clearCancel();
  observations.clear();
  on_delay = [&] {
    observations.push_back(robotState().motionTelemetry());
    requestCancel();
  };
  turnToAngle(45_deg, 1_s);
  CHECK_EQ(observations.size(), 1);
  CHECK(observations[0].phase == MotionPhase::Turn);
  CHECK_NEAR(observations[0].target_heading_deg, 45, 1e-10);
  CHECK(std::isnan(observations[0].remaining_in));
  CHECK(std::isnan(observations[0].carrot_x));
  CHECK(std::isnan(observations[0].carrot_y));
  clearCancel();

  // The pivot and final-align observations expose the branch actually used,
  // including zero translation and no active carrot during final alignment.
  for (bool align : {false, true}) {
    now_ms = 0;
    hw.heading = hw.encoder = 0;
    robotState().clearMotionOutputs();
    resetOdometry({0, align ? 23.0 : 0.0, 0});
    bool saw_phase = false;
    on_delay = [&] {
      const auto value = robotState().motionTelemetry();
      if (value.phase == (align ? MotionPhase::FinalAlign : MotionPhase::Pivot)) {
        saw_phase = true;
        CHECK_EQ(value.drive_volts, 0);
        CHECK_EQ(value.slew_limited, 0);
        CHECK(align ? std::isnan(value.carrot_x) : std::isfinite(value.carrot_x));
        CHECK_NEAR(value.target_heading_deg, align ? 90 : 180, 1e-10);
        requestCancel();
      }
    };
    boomerang(0_in, (align ? 24 : -24) * 1_in, 1,
              (align ? 90 : 0) * 1_deg, 0, 1_s);
    CHECK(saw_phase);
    CHECK(robotState().motionTelemetry().phase == MotionPhase::Idle);
    clearCancel();
  }
  on_delay = {};

  now_ms = 0;
  hw.heading = hw.encoder = 0;
  robotState().clearMotionOutputs();
  resetOdometry({0, 23, 0});
  // A held position demand inside the band still needs an exit ramp.
  robotState().setPrevOutputs(3, 3);
  MotionConfig immediate_alignment = saved_config;
  immediate_alignment.turn_exit.small_duration = 0_ms;
  immediate_alignment.turn_exit.big_duration = 0_ms;
  setMotionConfig(immediate_alignment);
  bool saw_deceleration = false;
  on_delay = [&] {
    const auto value = robotState().motionTelemetry();
    if (value.phase == MotionPhase::Decelerate) {
      saw_deceleration = true;
      CHECK_EQ(value.drive_volts, 0);
      CHECK_EQ(value.yaw_volts, 0);
      CHECK(std::isnan(value.target_heading_deg));
      CHECK(std::isnan(value.carrot_x));
    }
  };
  boomerang(0_in, 24_in, 1, 0_deg, 0, 1_s);
  CHECK(saw_deceleration);
  setMotionConfig(saved_config);
  on_delay = {};

  // Timeout arithmetic survives the 32-bit PROS clock rollover.
  now_ms = std::numeric_limits<std::uint32_t>::max() - 20;
  const auto start = now_ms;
  turnToAngle(180_deg, 100_ms, false);
  CHECK_EQ(static_cast<std::uint32_t>(now_ms - start), 100);
  CHECK_EQ(hw.left, 0);
  bindDrive(nullptr);
  return mclib::test::summary("motion safety");
}
