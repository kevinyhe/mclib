// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The path planner's calls into path_helpers.hpp, linked into the simulator
// library next to tests/vexsim/bridge.cpp.
//
// A section is 10 doubles: start, start handle, end handle and end as x, y
// pairs in inches, the speed limit in in/s, and 1 when reversed. Limits are
// acceleration and cornering acceleration in in/s^2 and the track width in
// inches. RAMSETE settings are kS in V, kV in V per in/s, b, zeta and the
// effective track width in inches.
#include "path_helpers.hpp"

#include <vector>

namespace {

constexpr int kSectionValues = 10;
constexpr int kStateValues = 8;

std::vector<path_planner::Section> readSections(const double* values, int count) {
  std::vector<path_planner::Section> sections;
  for (int i = 0; i < count; ++i) {
    const double* v = values + kSectionValues * i;
    sections.push_back({{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}, v[8], v[9] != 0});
  }
  return sections;
}

path_planner::Limits readLimits(const double* values) {
  return {values[0], values[1], values[2]};
}

}  // namespace

// The first section that stops or doubles back on itself, or -1.
extern "C" int planner_folded_section(const double* sections, int section_count) {
  const auto all = readSections(sections, section_count);
  for (int i = 0; i < section_count; ++i) {
    std::vector<mclib::path::PathPoint> points;
    if (!path_planner::detail::appendSection(points, all[i])) return i;
  }
  return -1;
}

// Writes every planned state as (part, time s, distance in, x in, y in,
// heading deg, signed speed in/s, curvature 1/in), with time and distance
// counted from the start of the path. Returns the number of states, -1 when
// the sections can't be planned, -2 when @p max_states is too small.
extern "C" int planner_plan(const double* sections, int section_count, const double* limits,
                            double* out, int max_states) {
  const auto plans = path_planner::planSections(readSections(sections, section_count), readLimits(limits));
  if (plans.empty()) return -1;
  int written = 0;
  double time_offset_s = 0, distance_offset_in = 0;
  for (std::size_t part = 0; part < plans.size(); ++part) {
    for (const auto& state : plans[part].states()) {
      if (written == max_states) return -2;
      double* row = out + kStateValues * written++;
      row[0] = static_cast<double>(part);
      row[1] = time_offset_s + state.time.s();
      row[2] = distance_offset_in + state.distance.in();
      row[3] = state.x.in();
      row[4] = state.y.in();
      row[5] = state.heading.deg();
      row[6] = state.velocity.inps();
      row[7] = state.curvature.raw() * (1.0 * mclib::units::inch).raw();
    }
    time_offset_s += plans[part].duration().s();
    distance_offset_in += plans[part].length().in();
  }
  return written;
}

// Plans the sections and follows them. Returns the MotionResult as an int, or
// -1 when the sections can't be planned.
extern "C" int planner_follow(const double* sections, int section_count, const double* limits,
                              const double* ramsete, double timeout_ms, double volts) {
  using namespace mclib::units;
  const auto plans = path_planner::planSections(readSections(sections, section_count), readLimits(limits));
  if (plans.empty()) return -1;
  mclib::control::RamseteConfig config;
  config.feedforward.kS = ramsete[0] * volt;
  config.feedforward.kV = ramsete[1] * volt / inps;
  config.gains = {ramsete[2], ramsete[3]};
  config.track_width = ramsete[4] * inch;
  return static_cast<int>(path_planner::followSections(plans, config, timeout_ms * millisecond, volts * volt));
}
