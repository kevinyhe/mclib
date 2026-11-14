// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
/**
 * @file pose_filter_test.cpp
 * @brief PoseFilter against a robot driving laps with drifting odometry.
 *
 * The true robot drives 60 in squares inside the field. Its odometry counts
 * 4% too much distance and reads every 90 deg turn as 91 deg, so the raw pose
 * wanders off. Four distance sensors see the perimeter with 2% noise, and a
 * tenth of their readings are blocked (something 8-20 in in front of the
 * sensor). The filter should stay near the truth and reject the blocked
 * readings.
 */

#include "mclib/control/pose_filter.hpp"
#include "mclib/math.hpp"
#include "mclib/snapshot/collision_map.hpp"
#include "mclib/snapshot/solver.hpp"
#include "test_assert.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <utility>
#include <vector>

using mclib::Pose2D;
using mclib::control::PoseFilter;
using mclib::control::PoseFilterConfig;

namespace {

constexpr double kDeg = mclib::kPi / 180.0;

// Two on the left, 10 in apart front to back, so heading shows up as a
// difference between them; one on the right; one at the back.
const std::vector<snapshot::SensorGeometry> kSensors = {
    {-6.0f, 5.0f, -90.0f, 0u, 78.0f},
    {-6.0f, -5.0f, -90.0f, 0u, 78.0f},
    {6.0f, 0.0f, 90.0f, 0u, 78.0f},
    {0.0f, -6.0f, 180.0f, 0u, 78.0f},
};

double trueRange(const snapshot::SensorGeometry& g, const Pose2D& p) {
  const auto prediction = snapshot::predict_range(
      g, static_cast<float>(p.x), static_cast<float>(p.y),
      static_cast<float>(p.theta / kDeg), snapshot::MAP_PERIMETER);
  return prediction.valid ? prediction.expected_in : -1.0;
}

struct Run {
  double odom_error_in = 0;
  double filter_error_in = 0;
  double filter_heading_error_deg = 0;
  double worst_filter_error_in = 0;
  int accepted = 0;
  int rejected = 0;
  int blocked = 0;
  double final_sigma_in = 0;
};

/// Drive @p laps squares. @p use_sensors false runs predict() only.
Run drive(int laps, bool use_sensors, std::uint32_t seed = 7) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  const Pose2D start{42, 42, 0};
  Pose2D truth = start, odom = start;
  PoseFilter filter;
  filter.reset(start, 0.5, 0.5);
  Run run;
  int tick = 0;

  auto step = [&](double forward_in, double turn_rad) {
    // Truth moves exactly; odometry over-counts distance and turns.
    Pose2D previous_odom = odom;
    truth.x += forward_in * std::sin(truth.theta);
    truth.y += forward_in * std::cos(truth.theta);
    truth.theta = mclib::wrapAngle(truth.theta + turn_rad);
    const double odom_forward = forward_in * 1.04;
    odom.x += odom_forward * std::sin(odom.theta);
    odom.y += odom_forward * std::cos(odom.theta);
    odom.theta = mclib::wrapAngle(odom.theta + turn_rad * 91.0 / 90.0);
    filter.predict(previous_odom, odom);

    // Sensors at 20 Hz on a 100 Hz loop.
    if (use_sensors && ++tick % 5 == 0) {
      for (const auto& g : kSensors) {
        double reading = trueRange(g, truth);
        if (reading <= 0) continue;
        if (uniform(rng) < 0.1) {
          reading = 8 + 12 * uniform(rng);  // something in the way
          ++run.blocked;
        } else {
          reading += 0.02 * reading * unit(rng);
        }
        filter.updateDistance(g, reading);
      }
    }
    run.worst_filter_error_in =
        std::max(run.worst_filter_error_in,
                 std::hypot(filter.pose().x - truth.x, filter.pose().y - truth.y));
  };

  for (int lap = 0; lap < laps; ++lap) {
    for (int side = 0; side < 4; ++side) {
      for (int i = 0; i < 250; ++i) step(60.0 / 250, 0);          // 60 in at 24 in/s
      for (int i = 0; i < 50; ++i) step(0, 90.0 * kDeg / 50);     // turn right
    }
  }
  run.odom_error_in = std::hypot(odom.x - truth.x, odom.y - truth.y);
  run.filter_error_in = std::hypot(filter.pose().x - truth.x, filter.pose().y - truth.y);
  run.filter_heading_error_deg = std::fabs(mclib::wrapAngle(filter.pose().theta - truth.theta)) / kDeg;
  run.accepted = filter.accepted();
  run.rejected = filter.rejected();
  run.final_sigma_in = filter.positionSigmaIn();
  return run;
}

}  // namespace

int main() {
  std::printf("-- predict only: the filter follows odometry exactly\n");
  {
    const Run r = drive(2, false);
    std::printf("   odometry %.3f in off, filter %.3f in off, sigma %.2f in\n",
                r.odom_error_in, r.filter_error_in, r.final_sigma_in);
    CHECK_NEAR(r.filter_error_in, r.odom_error_in, 1e-6);
    CHECK_EQ(r.accepted, 0);
    // With no corrections the uncertainty only grows.
    CHECK(r.final_sigma_in > 5.0);
  }

  std::printf("-- distance sensors, 4 laps, 10%% of readings blocked\n");
  {
    const Run r = drive(4, true);
    std::printf("   odometry %.2f in off; filter %.2f in and %.2f deg off (worst %.2f in), "
                "sigma %.2f in\n",
                r.odom_error_in, r.filter_error_in, r.filter_heading_error_deg,
                r.worst_filter_error_in, r.final_sigma_in);
    std::printf("   readings: %d accepted, %d rejected, %d blocked\n", r.accepted,
                r.rejected, r.blocked);
    CHECK(r.odom_error_in > 10.0);
    CHECK(r.filter_error_in < 1.5);
    CHECK(r.worst_filter_error_in < 3.0);
    CHECK(r.filter_heading_error_deg < 2.0);
    CHECK(r.final_sigma_in < 2.0);
    // Most blocked readings are rejected. Some blocked readings land close
    // to a wall's true range and can't be told apart; that's fine.
    CHECK(r.rejected >= r.blocked * 7 / 10);
    // And most real readings are kept.
    CHECK(r.accepted > 0.8 * (r.accepted + r.rejected - r.blocked));
  }

  std::printf("-- the same with other noise seeds\n");
  {
    double worst = 0;
    for (std::uint32_t seed = 1; seed <= 10; ++seed)
      worst = std::max(worst, drive(4, true, seed).filter_error_in);
    std::printf("   worst final error over 10 seeds: %.2f in\n", worst);
    CHECK(worst < 2.0);
  }

  std::printf("-- uncertainty doesn't depend on the loop rate\n");
  {
    // 48 in forward and a 90 deg turn, in 10 steps and in 1000.
    auto sigma_after = [](int steps) {
      PoseFilter filter;
      filter.reset(Pose2D{72, 72, 0}, 0.0, 0.0);
      Pose2D odom{72, 72, 0};
      for (int i = 0; i < steps; ++i) {
        const Pose2D previous = odom;
        odom.y += 48.0 / steps;
        odom.theta += 90.0 * kDeg / steps;
        filter.predict(previous, odom);
      }
      return std::make_pair(filter.covariance().trace(), filter.covariance()(2, 2));
    };
    const auto coarse = sigma_after(10), fine = sigma_after(1000);
    std::printf("   covariance trace: %.4f (10 steps), %.4f (1000 steps)\n",
                coarse.first, fine.first);
    // Rotating the along/side split changes the x-y share between steps;
    // the heading part and the total are what must match.
    CHECK_NEAR(coarse.second, fine.second, 1e-12);
    CHECK_NEAR(coarse.first, fine.first, 0.05 * fine.first);
  }

  std::printf("-- GPS\n");
  {
    PoseFilter filter;
    filter.reset(Pose2D{50, 50, 0}, 6.0, 5.0);
    // A good GPS fix pulls a very uncertain pose most of the way.
    CHECK(filter.updateGps(Pose2D{56, 50, 4 * kDeg}, 0.5, 1.0));
    std::printf("   after one fix 6 in away: x = %.3f, heading %.3f deg, sigma %.3f in\n",
                filter.pose().x, filter.pose().theta / kDeg, filter.positionSigmaIn());
    CHECK(filter.pose().x > 55.5);
    CHECK(filter.pose().theta / kDeg > 3.0);
    CHECK(filter.positionSigmaIn() < 0.6);
    // Now confident, a fix 20 in away is an outlier and changes nothing.
    const Pose2D before = filter.pose();
    CHECK(!filter.updateGps(Pose2D{76, 50, 0}, 0.5, 1.0));
    CHECK_EQ(filter.pose().x, before.x);
    CHECK_EQ(filter.rejected(), 1);
    // Position only: a negative heading sigma leaves the heading alone.
    CHECK(filter.updateGps(Pose2D{56.2, 50, NAN}, 0.5));
    CHECK_EQ(filter.pose().theta, before.theta);
    // Bad input is ignored, not counted.
    CHECK(!filter.updateGps(Pose2D{NAN, 50, 0}, 0.5));
    CHECK(!filter.updateGps(Pose2D{56, 50, 0}, 0.0));
    CHECK_EQ(filter.rejected(), 1);
  }

  std::printf("-- a late GPS reading is applied at the time it was taken\n");
  {
    // Driving +Y at 60 in/s on a 10 ms loop, odometry reading 10% short. The
    // GPS reads the true position, but each reading arrives 50 ms late.
    auto final_error = [](bool timed) {
      PoseFilter filter;
      filter.reset(Pose2D{72, 20, 0}, 0.5, 0.5);
      Pose2D odom{72, 20, 0};
      double true_y = 20;
      std::vector<double> past_truth;
      for (int tick = 1; tick <= 150; ++tick) {
        const Pose2D previous = odom;
        odom.y += 0.6 * 0.9;
        true_y += 0.6;
        past_truth.push_back(true_y);
        const std::uint32_t now_ms = 10 * tick;
        filter.predict(previous, odom, now_ms);
        if (tick % 2 == 0 && tick > 5) {
          const Pose2D taken{72, past_truth[past_truth.size() - 6], NAN};
          if (timed) {
            filter.updateGps(taken, 0.5, -1.0, now_ms - 50);
          } else {
            filter.updateGps(taken, 0.5);
          }
        }
      }
      return std::fabs(filter.pose().y - true_y);
    };
    const double timed = final_error(true), untimed = final_error(false);
    std::printf("   applied when taken: %.2f in off; applied when it arrived: %.2f in off\n",
                timed, untimed);
    CHECK(timed < 0.6);
    CHECK(untimed > 2.0);
  }

  std::printf("-- a late reading lands where an on-time one would\n");
  {
    // Same steps and readings; one filter gets the GPS fix on time, the
    // other 60 ms later, between distance readings it has already used.
    const snapshot::SensorGeometry left{-6.0f, 0.0f, -90.0f, 0u, 200.0f};
    auto run = [&](bool late) {
      PoseFilter filter;
      filter.reset(Pose2D{40, 30, 0}, 2.0, 1.0);
      Pose2D odom{40, 30, 0};
      for (int tick = 1; tick <= 20; ++tick) {
        const Pose2D previous = odom;
        odom.y += 0.4;
        odom.theta += 0.2 * kDeg;
        filter.predict(previous, odom, 10 * tick);
        if (tick == 8 && !late) filter.updateGps(Pose2D{41, 33.5, NAN}, 0.7, -1.0, 80);
        if (tick % 3 == 0) filter.updateDistance(left, 33.5 + 0.02 * tick, 10 * tick);
        if (tick == 14 && late) filter.updateGps(Pose2D{41, 33.5, NAN}, 0.7, -1.0, 80);
      }
      return filter;
    };
    const PoseFilter on_time = run(false), late = run(true);
    CHECK_NEAR(late.pose().x, on_time.pose().x, 1e-9);
    CHECK_NEAR(late.pose().y, on_time.pose().y, 1e-9);
    CHECK_NEAR(late.pose().theta, on_time.pose().theta, 1e-12);
    CHECK_NEAR((late.covariance() - on_time.covariance()).norm(), 0.0, 1e-9);
    CHECK_EQ(late.accepted(), on_time.accepted());
  }

  std::printf("-- a reading older than the remembered steps is refused\n");
  {
    PoseFilterConfig config;
    config.history_ms = 100;
    PoseFilter filter(config);
    filter.reset(Pose2D{72, 72, 0}, 3.0, 1.0);
    Pose2D odom{72, 72, 0};
    for (int tick = 1; tick <= 30; ++tick) {
      const Pose2D previous = odom;
      odom.y += 0.5;
      filter.predict(previous, odom, 10 * tick);
    }
    const Pose2D before = filter.pose();
    CHECK(!filter.updateGps(Pose2D{75, 87, NAN}, 0.5, -1.0, 150));
    CHECK_EQ(filter.tooOld(), 1);
    CHECK_EQ(filter.pose().x, before.x);
    CHECK(filter.updateGps(Pose2D{75, 87, NAN}, 0.5, -1.0, 250));
    CHECK(filter.pose().x > before.x);
    // The untimed predict() forgets the steps: nothing late can land after it.
    const Pose2D previous = odom;
    odom.y += 0.5;
    filter.predict(previous, odom);
    CHECK(filter.updateGps(Pose2D{75, 87.5, NAN}, 0.5, -1.0, 290));
    CHECK_EQ(filter.tooOld(), 1);
  }

  std::printf("-- a reading taken history_ms ago lands though no step was then\n");
  {
    // Steps every 10 ms, the last one 3 ms late. A reading taken 100 ms
    // before it falls between the steps at 200 and 210 ms.
    PoseFilterConfig config;
    config.history_ms = 100;
    PoseFilter filter(config);
    filter.reset(Pose2D{72, 72, 0}, 3.0, 1.0);
    Pose2D odom{72, 72, 0};
    for (int tick = 1; tick <= 30; ++tick) {
      const Pose2D previous = odom;
      odom.y += 0.5;
      filter.predict(previous, odom, tick == 30 ? 303 : 10 * tick);
    }
    CHECK(filter.updateGps(Pose2D{75, 87, NAN}, 0.5, -1.0, 203));
    CHECK_EQ(filter.tooOld(), 0);
    CHECK(!filter.updateGps(Pose2D{75, 87, NAN}, 0.5, -1.0, 202));
    CHECK_EQ(filter.tooOld(), 1);
  }

  std::printf("-- PoseFusion uses a distance sensor as late as the filter remembers\n");
  {
    mclib::control::PoseFusionConfig config;
    config.filter.history_ms = 100;
    config.field_offset_x_in = 72.0;
    config.field_offset_y_in = 72.0;
    mclib::control::DistanceSensorInput late_sensor{.geometry = {0.0f, 0.0f, 0.0f, 0u, 200.0f},
                                                    .read_in = [] { return 72.0; },
                                                    .period_ms = 300,
                                                    .latency_ms = 100};
    mclib::control::PoseFusion fusion(config, {late_sensor});
    CHECK(!fusion.hasConfigurationError());
    fusion.reset(Pose2D{0, 0, 0});
    for (int tick = 1; tick <= 30; ++tick) {
      fusion.step(Pose2D{0, 0, 0}, tick == 30 ? 303 : 10 * tick);
    }
    CHECK_EQ(fusion.accepted(), 1);
    CHECK_EQ(fusion.tooOld(), 0);
  }

  std::printf("-- PoseFusion refuses a latency longer than the filter remembers\n");
  {
    mclib::control::PoseFusionConfig config;
    config.filter.history_ms = 100;
    mclib::control::PoseFusion fine(config, {});
    CHECK(!fine.hasConfigurationError());
    mclib::control::GpsInput gps;
    gps.read = [] { return std::optional<Pose2D>(); };
    gps.latency_ms = 100;
    CHECK(fine.setGps(gps));
    gps.latency_ms = 101;
    CHECK(!fine.setGps(gps));

    mclib::control::DistanceSensorInput late_sensor{.geometry = {0.0f, 0.0f, 0.0f},
                                                    .read_in = [] { return 10.0; },
                                                    .latency_ms = 101};
    mclib::control::PoseFusion misconfigured(config, {late_sensor});
    CHECK(misconfigured.hasConfigurationError());
    misconfigured.reset(Pose2D{72, 72, 0});
    CHECK(!misconfigured.step(Pose2D{72, 80, 0}, 10).has_value());
    CHECK_EQ(misconfigured.accepted() + misconfigured.rejected(), 0);
  }

  std::printf("-- distance reading edge cases\n");
  {
    PoseFilter filter;
    filter.reset(Pose2D{72, 72, 0});
    // Facing +Y, a forward sensor sees the far wall 72 in away.
    const snapshot::SensorGeometry forward{0.0f, 0.0f, 0.0f, 0u, 200.0f};
    CHECK(filter.updateDistance(forward, 72.0));
    CHECK(!filter.updateDistance(forward, 0.0));
    CHECK(!filter.updateDistance(forward, NAN));
    // Out of range: the ray hits nothing.
    const snapshot::SensorGeometry short_range{0.0f, 0.0f, 0.0f, 0u, 20.0f};
    CHECK(!filter.updateDistance(short_range, 15.0));
    // A reading that says the robot is 3 in closer to the wall than it
    // thinks moves it that way.
    filter.reset(Pose2D{72, 72, 0}, 3.0, 0.5);
    CHECK(filter.updateDistance(forward, 69.0));
    std::printf("   72 in expected, 69 in read: y moved to %.3f\n", filter.pose().y);
    CHECK(filter.pose().y > 73.5);
    CHECK_NEAR(filter.pose().x, 72.0, 1e-6);
  }

  std::printf("-- PoseFusion: laps with the correction written back into odometry\n");
  {
    // The same drifting odometry as above, but each tick the odometry
    // position is replaced by what PoseFusion returns, as the odometry task
    // does. Odometry here starts at 0 in the field centre: offset 72.
    std::mt19937 rng(7);
    std::normal_distribution<double> unit(0.0, 1.0);
    Pose2D truth{-30, -30, 0};
    Pose2D odom = truth;
    std::vector<double> latest(kSensors.size(), -1);
    std::vector<mclib::control::DistanceSensorInput> inputs;
    for (std::size_t i = 0; i < kSensors.size(); ++i) {
      inputs.push_back({kSensors[i], [&latest, i] { return latest[i]; }, 50});
    }
    mclib::control::PoseFusionConfig config;
    config.field_offset_x_in = config.field_offset_y_in = 72;
    mclib::control::PoseFusion fusion(config, inputs);
    fusion.reset(odom);
    std::uint32_t now_ms = 0;
    double worst_step_in = 0;  // largest correction while standing still
    auto tick = [&](double forward_in, double turn_rad) {
      now_ms += 10;
      truth.x += forward_in * std::sin(truth.theta);
      truth.y += forward_in * std::cos(truth.theta);
      truth.theta = mclib::wrapAngle(truth.theta + turn_rad);
      odom.x += forward_in * 1.04 * std::sin(odom.theta);
      odom.y += forward_in * 1.04 * std::cos(odom.theta);
      odom.theta = mclib::wrapAngle(odom.theta + turn_rad * 91.0 / 90.0);
      const Pose2D field_truth{truth.x + 72, truth.y + 72, truth.theta};
      for (std::size_t i = 0; i < kSensors.size(); ++i) {
        const double r = trueRange(kSensors[i], field_truth);
        latest[i] = r > 0 ? r + 0.02 * r * unit(rng) : -1;
      }
      const auto corrected = fusion.step(odom, now_ms);
      if (corrected.has_value()) {
        if (forward_in == 0 && turn_rad == 0)
          worst_step_in = std::max(worst_step_in,
                                   std::hypot(corrected->x() - odom.x, corrected->y() - odom.y));
        odom.x = corrected->x();
        odom.y = corrected->y();
      }
    };
    for (int lap = 0; lap < 4; ++lap)
      for (int side = 0; side < 4; ++side) {
        for (int i = 0; i < 250; ++i) tick(60.0 / 250, 0);
        for (int i = 0; i < 50; ++i) tick(0, 90.0 * kDeg / 50);
      }
    const double drive_error = std::hypot(odom.x - truth.x, odom.y - truth.y);
    for (int i = 0; i < 100; ++i) tick(0, 0);  // stand still for a second
    std::printf("   odometry %.2f in off after 4 laps (12.18 uncorrected); largest "
                "correction per tick standing still %.4f in; %d readings used\n",
                drive_error, worst_step_in, fusion.accepted());
    CHECK(drive_error < 2.0);
    // Standing still, only the 0.002 in floor moves it.
    CHECK(worst_step_in <= config.correction_floor_in + 1e-12);
    // pose() is in the odometry frame.
    CHECK(std::hypot(fusion.pose().x - truth.x, fusion.pose().y - truth.y) < 2.0);
    // A non-finite pose is ignored.
    CHECK(!fusion.step(Pose2D{NAN, 0, 0}, now_ms + 10).has_value());
  }

  std::printf("-- PoseFusion: GPS only\n");
  {
    // Odometry 4% long; a GPS reporting the true pose from the field centre
    // (0.5 in noise, every 20 ms, a reading missing every 10th time) pulls
    // it back. Odometry starts at 0 in the field centre: offset 72.
    std::mt19937 rng(11);
    std::normal_distribution<double> unit(0.0, 1.0);
    Pose2D truth{-30, -30, 0};
    Pose2D odom = truth;
    mclib::control::PoseFusionConfig config;
    config.field_offset_x_in = config.field_offset_y_in = 72;
    mclib::control::PoseFusion fusion(config, {});
    int reads = 0;
    fusion.setGps({.read = [&]() -> std::optional<Pose2D> {
                     if (++reads % 10 == 0) return std::nullopt;
                     return Pose2D{truth.x + 0.5 * unit(rng), truth.y + 0.5 * unit(rng), truth.theta};
                   },
                   .error_in = [] { return 0.5; }});
    fusion.reset(odom);
    std::uint32_t now_ms = 0;
    for (int side = 0; side < 8; ++side) {
      for (int i = 0; i < 250; ++i) {
        now_ms += 10;
        truth.y += 60.0 / 250;
        odom.y += 60.0 / 250 * 1.04;
        if (const auto corrected = fusion.step(odom, now_ms)) {
          odom.x = corrected->x();
          odom.y = corrected->y();
        }
      }
      truth.y -= 60;  // jump back: the same 60 in again, odometry follows
      odom.y -= 60;
      fusion.step(odom, now_ms);
    }
    const double error = std::hypot(odom.x - truth.x, odom.y - truth.y);
    std::printf("   odometry %.2f in off after 480 in (19.2 uncorrected); %d readings used, "
                "%d GPS reads\n", error, fusion.accepted(), reads);
    CHECK(error < 1.5);
    // Read no more often than every 20 ms.
    CHECK(reads <= static_cast<int>(now_ms / 20) + 1);
    CHECK(fusion.accepted() > 0);
  }

  return mclib::test::summary("pose_filter");
}
