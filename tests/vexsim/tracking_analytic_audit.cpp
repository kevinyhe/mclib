// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
// Standalone audit: compile with the real odometry.cpp, robot_state.cpp, math.cpp.
// The oracle integrates world velocities and wheel-contact velocities using
// Simpson quadrature. It never calls the library's arc/chord or frame helpers.
#include "mclib/control/odometry.hpp"
#include "mclib/control/odometry_task.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <random>
#include <string>

using mclib::Pose2D;
using mclib::control::Odometry;
using mclib::control::OdometryConfig;
using mclib::control::OdometrySample;

namespace {
constexpr double pi = 3.141592653589793238462643383279502884;
int checks = 0;
int failed = 0;

void check(bool result, const char* message) {
  ++checks;
  if (!result) {
    ++failed;
    std::printf("FAIL: %s\n", message);
  }
}

double headingDifference(double a, double b) {
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

double positionError(const Pose2D& a, const Pose2D& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

struct Geometry {
  double drive_diameter = 3.25;
  double drive_gear = .75;  // wheel turns per encoder turn
  double track_width = 11.5;
  double vertical_diameter = 2.75;
  double vertical_gear = 1.0;
  double vertical_right = -3.0;
  double horizontal_diameter = 2.0;
  double horizontal_gear = 1.0;
  double horizontal_forward = -4.0;

  std::array<double, 4> inchesPerEncoderTurn() const {
    return {pi * drive_diameter * drive_gear,
            pi * drive_diameter * drive_gear,
            pi * vertical_diameter * vertical_gear,
            pi * horizontal_diameter * horizontal_gear};
  }

  OdometryConfig config(int layout) const {
    using namespace mclib::units;
    mclib::control::OdometrySetup setup;
    setup.drive = {Wheel::fromDiameter(drive_diameter * inch),
                   track_width * inch, drive_gear};
    if (layout >= 1) {
      setup.vertical = mclib::control::TrackingWheelSensor{
          [] { return 0.0; },
          {Wheel::fromDiameter(vertical_diameter * inch),
           vertical_right * inch, vertical_gear}};
    }
    if (layout >= 2) {
      setup.horizontal = mclib::control::TrackingWheelSensor{
          [] { return 0.0; },
          {Wheel::fromDiameter(horizontal_diameter * inch),
           horizontal_forward * inch, horizontal_gear}};
    }
    return mclib::control::odometryConfigFrom(setup);
  }
};

struct Motion {
  double right;
  double forward;
  double heading;
  double clockwise_rate;
};

using Velocity = std::function<Motion(double)>;
using Integral = std::array<double, 6>;  // world x,y and four wheel travels

Integral derivatives(const Motion& v, const Geometry& g) {
  const double c = std::cos(v.heading), s = std::sin(v.heading);
  const double vx = v.right * c + v.forward * s;
  const double vy = -v.right * s + v.forward * c;
  Integral result{vx, vy, 0, 0, 0, 0};
  const std::array<std::array<double, 2>, 4> offsets = {{
      {{-g.track_width / 2, 0}}, {{g.track_width / 2, 0}},
      {{g.vertical_right, 0}}, {{0, g.horizontal_forward}}}};
  for (int wheel = 0; wheel != 4; ++wheel) {
    // A clockwise angular velocity is a negative world Z angular velocity.
    // Independently compute omega cross radius at the physical sensor point.
    const double rx = c * offsets[wheel][0] + s * offsets[wheel][1];
    const double ry = -s * offsets[wheel][0] + c * offsets[wheel][1];
    const double px = vx + v.clockwise_rate * ry;
    const double py = vy - v.clockwise_rate * rx;
    const double axis_x = wheel == 3 ? c : s;
    const double axis_y = wheel == 3 ? -s : c;
    result[wheel + 2] = px * axis_x + py * axis_y;
  }
  return result;
}

Integral integrate(const Velocity& velocity, const Geometry& geometry,
                   double start, double end, int intervals = 64) {
  Integral result{};
  const double h = (end - start) / intervals;
  for (int i = 0; i <= intervals; ++i) {
    const double weight = i == 0 || i == intervals ? 1 : (i % 2 ? 4 : 2);
    const auto value = derivatives(velocity(start + h * i), geometry);
    for (std::size_t k = 0; k < result.size(); ++k) {
      result[k] += weight * value[k];
    }
  }
  for (double& value : result) value *= h / 3;
  return result;
}

struct Rig {
  Geometry geometry;
  Pose2D truth;
  OdometrySample sample;

  Rig(Geometry g, Pose2D initial) : geometry(g), truth(initial) {
    // Encoders deliberately do not start at zero or agree with each other.
    sample = {initial.theta, 1234.567, -2345.678, 3456.789, -4567.890};
  }

  void advance(const Velocity& velocity, double start, double end,
               bool wrap_heading = false) {
    const auto delta = integrate(velocity, geometry, start, end);
    truth.x += delta[0];
    truth.y += delta[1];
    truth.theta = velocity(end).heading;
    auto ipr = geometry.inchesPerEncoderTurn();
    sample.left_deg += delta[2] * 360 / ipr[0];
    sample.right_deg += delta[3] * 360 / ipr[1];
    sample.vertical_deg += delta[4] * 360 / ipr[2];
    sample.horizontal_deg += delta[5] * 360 / ipr[3];
    sample.heading_rad = wrap_heading
        ? std::atan2(std::sin(truth.theta), std::cos(truth.theta)) : truth.theta;
  }
};

struct Run {
  Pose2D measured;
  Pose2D truth;
};

Run runMotion(const Geometry& geometry, int layout, const Pose2D& start,
              const Velocity& velocity, double duration, int steps) {
  Odometry odometry(geometry.config(layout));
  odometry.reset(start);
  Rig rig(geometry, start);
  const auto seeded = odometry.update(rig.sample);
  check(positionError(seeded, start) == 0, "nonzero initial encoders must only seed");
  for (int i = 0; i < steps; ++i) {
    rig.advance(velocity, duration * i / steps, duration * (i + 1) / steps,
                i % 2 == 0);
    // Configured trackers must make unused drive fields irrelevant, even NaN.
    auto input = rig.sample;
    if (layout > 0) input.left_deg = input.right_deg = NAN;
    if (layout < 2) input.horizontal_deg = NAN;
    if (layout == 0) input.vertical_deg = NAN;
    odometry.update(input);
  }
  return {odometry.getPose(), rig.truth};
}

void randomizedConstantTwists() {
  std::mt19937_64 rng(0x4f444f4d45545259ULL);
  auto uniform = [&rng](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  double worst_position = 0, worst_heading = 0, worst_subdivision = 0;
  int scenarios = 0;
  for (int index = 0; index < 240; ++index) {
    Geometry g;
    g.drive_diameter = uniform(2, 4.25);
    g.drive_gear = uniform(.3, 2.5);
    g.track_width = uniform(7, 18);
    g.vertical_diameter = uniform(1.75, 3.25);
    g.vertical_gear = uniform(.5, 2);
    g.vertical_right = uniform(-8, 8);
    g.horizontal_diameter = uniform(1.75, 3.25);
    g.horizontal_gear = uniform(.5, 2);
    g.horizontal_forward = uniform(-8, 8);
    const Pose2D start{uniform(-100, 100), uniform(-100, 100), uniform(-18, 18)};
    const double duration = uniform(.1, 2);
    const double turn = index % 12 == 0 ? 1e-12 : uniform(-2.9, 2.9);
    const double forward = index % 13 == 0 ? 0 : uniform(-60, 60);
    for (int layout : {0, 1, 2}) {
      const double right = layout == 2 ? uniform(-20, 20) : 0;
      const Velocity velocity = [=](double t) {
        return Motion{right, forward, start.theta + turn * t / duration, turn / duration};
      };
      const auto reference_delta = integrate(velocity, g, 0, duration, 2048);
      const Pose2D reference{start.x + reference_delta[0], start.y + reference_delta[1],
                             start.theta + turn};
      Pose2D coarse;
      for (int steps : {1, 7, 79}) {
        const auto run = runMotion(g, layout, start, velocity, duration, steps);
        const double pe = positionError(run.measured, reference);
        const double he = std::abs(headingDifference(run.measured.theta, reference.theta));
        worst_position = std::max(worst_position, pe);
        worst_heading = std::max(worst_heading, he);
        check(pe < 2e-8, "random constant body twist must match independent quadrature");
        check(he < 2e-12, "signed/wrapped heading must match independent oracle");
        if (steps == 1) coarse = run.measured;
        else worst_subdivision = std::max(worst_subdivision,
                                         positionError(coarse, run.measured));
        ++scenarios;
      }
    }
  }
  std::printf("randomized_constant_twists scenarios=%d worst_position_in=%.12g "
              "worst_heading_rad=%.12g worst_subdivision_in=%.12g\n",
              scenarios, worst_position, worst_heading, worst_subdivision);
}

void signedFullTurnsAndOffsets() {
  double worst = 0;
  int cases = 0;
  for (int layout : {0, 1, 2}) {
    for (double offset : {-7.5, 0.0, 6.25}) {
      Geometry g;
      g.vertical_right = offset;
      g.horizontal_forward = -offset;
      for (double turn : {-8 * pi, -2 * pi, 2 * pi, 8 * pi}) {
        for (double forward : {-18.0, 0.0, 18.0}) {
          const Pose2D start{13, -9, 3.10};
          const Velocity velocity = [=](double t) {
            return Motion{layout == 2 ? 2.5 : 0, forward,
                          start.theta + turn * t, turn};
          };
          const auto run = runMotion(g, layout, start, velocity, 1, 500);
          worst = std::max(worst, positionError(run.measured, run.truth));
          check(positionError(run.measured, run.truth) < 1e-8,
                "multiple CW/CCW turns with signed offsets must integrate correctly");
          check(std::abs(headingDifference(run.measured.theta, run.truth.theta)) < 1e-11,
                "wrap crossings may not reverse incremental heading");
          ++cases;
        }
      }
    }
  }
  std::printf("signed_full_turns cases=%d worst_position_in=%.12g\n", cases, worst);
}

void varyingCurvatureConvergence() {
  for (int layout : {0, 1, 2}) {
    Geometry g;
    const Pose2D start{-2, 4, -2.6};
    const Velocity velocity = [=](double t) {
      return Motion{layout == 2 ? 3 * std::cos(.9 * t) : 0,
                    17 + 8 * std::sin(1.3 * t),
                    start.theta + .3 * t + .8 * std::sin(1.7 * t) + .15 * t * t,
                    .3 + 1.36 * std::cos(1.7 * t) + .3 * t};
    };
    std::array<double, 4> errors{};
    int index = 0;
    for (int steps : {30, 60, 120, 300}) {
      const auto run = runMotion(g, layout, start, velocity, 3, steps);
      errors[index++] = positionError(run.measured, run.truth);
    }
    check(errors[1] < .30 * errors[0] && errors[2] < .30 * errors[1],
          "smooth changing curvature must show second-order sample refinement");
    check(errors[3] < .001, "10ms varying-curvature sample error below .001 inch");
    std::printf("changing_curvature layout=%d errors_in_dt100_50_25_10ms="
                "%.12g,%.12g,%.12g,%.12g\n", layout,
                errors[0], errors[1], errors[2], errors[3]);
  }
}

void sensorObservability() {
  Geometry g;
  std::array<double, 3> lateral_error{};
  std::array<double, 3> wheelspin_error{};
  for (int layout : {0, 1, 2}) {
    const Velocity sideways = [](double) { return Motion{12, 0, 0, 0}; };
    auto slide = runMotion(g, layout, {}, sideways, 1, 100);
    lateral_error[layout] = positionError(slide.measured, slide.truth);
    check(std::abs(lateral_error[layout] - (layout < 2 ? 12.0 : 0.0)) < 1e-9,
          "lateral observability requires the horizontal tracking measurement");

    Odometry odometry(g.config(layout));
    Rig rig(g, {});
    odometry.update(rig.sample);
    const double left0 = rig.sample.left_deg, right0 = rig.sample.right_deg;
    const Velocity forward = [](double) { return Motion{0, 20, 0, 0}; };
    for (int i = 0; i < 100; ++i) {
      rig.advance(forward, i / 100.0, (i + 1) / 100.0);
      auto input = rig.sample;
      input.left_deg = left0 + 1.5 * (input.left_deg - left0);
      input.right_deg = right0 + 1.5 * (input.right_deg - right0);
      odometry.update(input);
    }
    wheelspin_error[layout] = positionError(odometry.getPose(), rig.truth);
    check(std::abs(wheelspin_error[layout] - (layout == 0 ? 10.0 : 0.0)) < 1e-9,
          "vertical tracker rejects drive-only wheelspin; drive encoders cannot");
  }
  std::printf("observability lateral12in_errors_layout012=%.6f,%.6f,%.6f "
              "drive_spin50pct_errors=%.6f,%.6f,%.6f\n",
              lateral_error[0], lateral_error[1], lateral_error[2],
              wheelspin_error[0], wheelspin_error[1], wheelspin_error[2]);
}

void dropoutConstantCurvature() {
  for (int layout : {0, 1, 2}) {
    Geometry g;
    const Pose2D start{6, -4, 2.5};
    Odometry odometry(g.config(layout));
    odometry.reset(start);
    Rig rig(g, start);
    odometry.update(rig.sample);
    const Velocity velocity = [=](double t) {
      return Motion{layout == 2 ? -3.0 : 0.0, -15, start.theta - 1.7 * t, -1.7};
    };
    for (int i = 0; i < 100; ++i) {
      rig.advance(velocity, i / 100.0, (i + 1) / 100.0, true);
      auto input = rig.sample;
      if (i >= 20 && i < 70) {
        // Exercise both IMU and enabled encoder rejection paths.
        if (i % 2 == 0) input.heading_rad = NAN;
        else if (layout == 0) input.left_deg = INFINITY;
        else input.vertical_deg = NAN;
      }
      odometry.update(input);
    }
    check(odometry.faultCount() == 50, "dropout faults must count rejected samples");
    check(positionError(odometry.getPose(), rig.truth) < 1e-8,
          "constant-twist dropout below pi radians can recover accumulated travel");
  }
}

void dropoutChangingCurvatureIsUnobservable() {
  Geometry g;
  std::array<Run, 2> paths{};
  std::array<OdometrySample, 2> final_samples{};
  for (int order : {0, 1}) {
    Rig rig(g, {});
    Odometry odometry(g.config(2));
    odometry.update(rig.sample);
    for (int segment = 0; segment < 2; ++segment) {
      const bool translate = segment == order;
      const double theta0 = rig.truth.theta;
      const Velocity velocity = [=](double t) {
        return Motion{0, translate ? 40.0 : 0.0,
                      theta0 + (translate ? 0.0 : pi * t), translate ? 0.0 : pi};
      };
      for (int i = 0; i < 50; ++i) {
        rig.advance(velocity, i / 100.0, (i + 1) / 100.0);
        auto input = rig.sample;
        if (!(segment == 1 && i == 49)) input.heading_rad = NAN;
        odometry.update(input);
      }
    }
    paths[order] = {odometry.getPose(), rig.truth};
    final_samples[order] = rig.sample;
    check(odometry.faultCount() == 99, "missing samples must be rejected until recovery");
    check(std::isfinite(paths[order].measured.x + paths[order].measured.y),
          "changing-curvature dropout must not poison pose");
  }
  check(positionError(paths[0].measured, paths[1].measured) < 1e-9,
        "identical recovered encoder/heading totals cannot distinguish motion order");
  check(std::abs(final_samples[0].vertical_deg - final_samples[1].vertical_deg) < 1e-9 &&
        std::abs(final_samples[0].horizontal_deg - final_samples[1].horizontal_deg) < 1e-9 &&
        std::abs(final_samples[0].left_deg - final_samples[1].left_deg) < 1e-9 &&
        std::abs(final_samples[0].right_deg - final_samples[1].right_deg) < 1e-9,
        "motion-order ambiguity fixture must have identical actual sensor totals");
  check(positionError(paths[0].truth, paths[1].truth) > 28,
        "missing motion order admits very different true endpoints");
  std::printf("dropout_order_ambiguity true_endpoint_separation_in=%.9f "
              "same_estimate_error_in=%.9f,%.9f\n",
              positionError(paths[0].truth, paths[1].truth),
              positionError(paths[0].measured, paths[0].truth),
              positionError(paths[1].measured, paths[1].truth));
}

void layoutReconfigurationCannotImportIgnoredNaNs() {
  Geometry g;
  Odometry odometry(g.config(0));
  OdometrySample first{0, 10, 10, NAN, NAN};
  odometry.update(first);  // Both NaN readings are explicitly disabled/ignored.
  odometry.setConfig(g.config(2));
  const auto pose = odometry.update({0, 11, 11, 100, 200});
  check(std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta),
        "enabling trackers after ignored NaN readings must not poison pose");
  std::printf("layout_reconfiguration_after_ignored_nan pose=(%.6f,%.6f,%.6f) "
              "fault_count=%u\n", pose.x, pose.y, pose.theta, odometry.faultCount());
}
}  // namespace

int main() {
  randomizedConstantTwists();
  signedFullTurnsAndOffsets();
  varyingCurvatureConvergence();
  sensorObservability();
  dropoutConstantCurvature();
  dropoutChangingCurvatureIsUnobservable();
  layoutReconfigurationCannotImportIgnoredNaNs();
  std::printf("analytic_odometry_audit checks=%d failed=%d\n", checks, failed);
  return failed == 0 ? 0 : 1;
}
