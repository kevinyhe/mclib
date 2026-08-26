// mclib
#pragma once

#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"
#include "mclib/units/units.hpp"

#include <array>
#include <cstddef>
#include <memory>

namespace mclib {
namespace mechanism {

/**
 * @brief The discrete angles a two-solenoid scoring hood can hold.
 *
 * The enumerators run from lowest angle to highest, and their underlying
 * values double as the index into HoodConfig::decode.
 */
enum class HoodState {
  Down = 0,
  Mid = 1,
  Up = 2,
};

/// Number of entries in HoodState, and therefore in the decode table.
inline constexpr std::size_t kHoodStateCount = 3;

/**
 * @brief The pair of logical solenoid values that produce one hood angle.
 *
 * These are LOGICAL values: true means "that solenoid is extended". The raw
 * signal sent to the ADI port is worked out by device::Pneumatic from its own
 * extended_state polarity, so an inverted solenoid never leaks into this table.
 */
struct HoodOutput {
  bool first = false;
  bool second = false;
};

/**
 * @brief Hardware and decode configuration for a Hood.
 *
 * The default table uses three of the four possible solenoid combinations:
 *
 * | HoodState | first | second |
 * |-----------|-------|--------|
 * | Down      | false | false  |
 * | Mid       | true  | false  |
 * | Up        | true  | true   |
 *
 * `(false, true)` is left unused so a two-stage hood only pushes the second
 * stage while the first is already out. Supply a different `decode` table to
 * use all four combinations, or to reorder the angles.
 */
struct HoodConfig {
  char first_port = 'A';
  char second_port = 'B';

  /// Whether the first solenoid is physically extended when its signal is true.
  bool first_extended_state = true;
  /// Whether the second solenoid is physically extended when its signal is true.
  bool second_extended_state = true;

  /// Value handed to both solenoids when their ADI ports are opened, in
  /// logical (extended) terms rather than raw signal terms. It only covers the
  /// window before the constructor applies `initial_state`, which happens
  /// immediately after.
  bool default_state = false;

  /// The logical angle the mechanism starts cached at. Kept separate from
  /// `default_state` on purpose: the seed is an angle, not a solenoid value.
  /// An angle outside `decode` falls back to HoodState::Down.
  HoodState initial_state = HoodState::Down;

  /// Single source of truth for state -> solenoid values, indexed by HoodState.
  std::array<HoodOutput, kHoodStateCount> decode = {{
      HoodOutput{false, false},
      HoodOutput{true, false},
      HoodOutput{true, true},
  }};
};

/**
 * @brief A multi-state pneumatic scoring hood.
 *
 * Two solenoids are decoded into three named angles through a table. The
 * logical HoodState is the mechanism's state; each solenoid's raw polarity
 * lives in device::Pneumatic and is never mixed into the cached state.
 */
class Hood : public StateMechanism<HoodState> {
public:
  explicit Hood(const HoodConfig& config);
  Hood(char first_port, char second_port);

  void setHoodState(HoodState state);
  HoodState getHoodState() const;

  /// Step one angle up, stopping at HoodState::Up.
  void next();
  /// Step one angle down, stopping at HoodState::Down.
  void previous();
  /// Step one angle up, wrapping from HoodState::Up back to HoodState::Down.
  void cycle();

  /// The solenoid values a state decodes to. A state outside the table
  /// reports the entry for the current state instead.
  HoodOutput outputFor(HoodState state) const;

  std::unique_ptr<Command> makeHoodStateCommand(HoodState state);
  std::unique_ptr<Command> makeHoodStateForCommand(HoodState state, QTime duration);
  std::unique_ptr<Command> makeNextCommand();
  std::unique_ptr<Command> makePreviousCommand();
  std::unique_ptr<Command> makeCycleCommand();

protected:
  /// Runs every scheduler tick. A state outside the decode table is ignored
  /// rather than read out of bounds or thrown on.
  void applyState(const HoodState& state) override;

private:
  // Hidden so no caller can cache a state the decode table cannot express.
  // Everything public goes through setHoodState, which rejects those.
  using StateMechanism<HoodState>::setState;
  using StateMechanism<HoodState>::makeStateCommand;
  using StateMechanism<HoodState>::makeStateOnceCommand;
  using StateMechanism<HoodState>::makeStateUntilCommand;
  using StateMechanism<HoodState>::makeStateForCommand;

  static bool isDecodable(HoodState state);
  static std::size_t indexOf(HoodState state);
  static HoodState seedState(const HoodConfig& config);

  HoodConfig m_config;
  device::Pneumatic m_first;
  device::Pneumatic m_second;
};

}  // namespace mechanism
}  // namespace mclib
