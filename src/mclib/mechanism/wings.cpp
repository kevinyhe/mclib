// mclib
#include "mclib/mechanism/wings.hpp"

namespace mclib {
namespace mechanism {

namespace {

WingsConfig makeConfig(char left_port, char right_port, bool default_state) {
  WingsConfig config;
  config.left_port = left_port;
  config.right_port = right_port;
  config.default_state = default_state;
  return config;
}

}  // namespace

Wings::Wings(const WingsConfig& config)
    : StateMechanism<WingsState>(
          WingsState{config.default_state, config.default_state}),
      m_left(config.left_port, config.default_state, config.left_extended_state),
      m_right(config.right_port, config.default_state,
              config.right_extended_state) {}

Wings::Wings(char left_port, char right_port, bool default_state)
    : Wings(makeConfig(left_port, right_port, default_state)) {}

void Wings::set(WingSide side, bool extended) {
  setState(withSide(side, extended));
}

void Wings::extend(WingSide side) {
  set(side, true);
}

void Wings::retract(WingSide side) {
  set(side, false);
}

void Wings::toggle(WingSide side) {
  WingsState next = getState();

  switch (side) {
    case WingSide::Left:
      next.left = !next.left;
      break;
    case WingSide::Right:
      next.right = !next.right;
      break;
    case WingSide::Both:
      next.left = !next.left;
      next.right = !next.right;
      break;
  }

  setState(next);
}

bool Wings::isExtended(WingSide side) const {
  const WingsState& state = getState();

  switch (side) {
    case WingSide::Left:
      return state.left;
    case WingSide::Right:
      return state.right;
    case WingSide::Both:
      return state.left && state.right;
  }

  return false;
}

void Wings::extendBoth() {
  extend(WingSide::Both);
}

void Wings::retractBoth() {
  retract(WingSide::Both);
}

std::unique_ptr<Command> Wings::makeSetCommand(WingSide side, bool extended) {
  return runOnce([this, side, extended]() { set(side, extended); });
}

std::unique_ptr<Command> Wings::makeExtendCommand(WingSide side) {
  return makeSetCommand(side, true);
}

std::unique_ptr<Command> Wings::makeRetractCommand(WingSide side) {
  return makeSetCommand(side, false);
}

std::unique_ptr<Command> Wings::makeToggleCommand(WingSide side) {
  return runOnce([this, side]() { toggle(side); });
}

std::unique_ptr<Command> Wings::makeExtendForCommand(WingSide side,
                                                     QTime duration) {
  // makeStateForCommand holds one fixed WingsState for the whole duration, so
  // the other side's value is snapshotted here, when the command is built.
  return makeStateForCommand(withSide(side, true), duration);
}

void Wings::applyState(const WingsState& state) {
  m_left.set_value(state.left);
  m_right.set_value(state.right);
}

WingsState Wings::withSide(WingSide side, bool extended) const {
  WingsState next = getState();

  switch (side) {
    case WingSide::Left:
      next.left = extended;
      break;
    case WingSide::Right:
      next.right = extended;
      break;
    case WingSide::Both:
      next.left = extended;
      next.right = extended;
      break;
  }

  return next;
}

}  // namespace mechanism
}  // namespace mclib
