// mclib
#include "mclib/mechanism/hood.hpp"

namespace mclib {
namespace mechanism {

Hood::Hood(const HoodConfig& config)
    : StateMechanism<HoodState>(seedState(config)),
      m_config(config),
      m_first(config.first_port, config.default_state, config.first_extended_state),
      m_second(config.second_port, config.default_state, config.second_extended_state) {
  // The solenoids power on at default_state, which is a raw value and need not
  // agree with the logical seed. Drive them to the seeded angle right away so
  // the cached state is never a lie, even before the first scheduler tick.
  Hood::applyState(getHoodState());
}

Hood::Hood(char first_port, char second_port)
    : Hood([&]() {
        HoodConfig config;
        config.first_port = first_port;
        config.second_port = second_port;
        return config;
      }()) {}

void Hood::setHoodState(HoodState state) {
  if (!isDecodable(state)) {
    return;
  }
  setState(state);
}

HoodState Hood::getHoodState() const {
  return getState();
}

void Hood::next() {
  const std::size_t index = indexOf(getHoodState());
  if (index + 1 < kHoodStateCount) {
    setHoodState(static_cast<HoodState>(index + 1));
  }
}

void Hood::previous() {
  const std::size_t index = indexOf(getHoodState());
  if (index > 0) {
    setHoodState(static_cast<HoodState>(index - 1));
  }
}

void Hood::cycle() {
  const std::size_t index = indexOf(getHoodState());
  setHoodState(static_cast<HoodState>((index + 1) % kHoodStateCount));
}

HoodOutput Hood::outputFor(HoodState state) const {
  const HoodState resolved = isDecodable(state) ? state : getHoodState();
  if (!isDecodable(resolved)) {
    return HoodOutput{};
  }
  return m_config.decode[indexOf(resolved)];
}

std::unique_ptr<Command> Hood::makeHoodStateCommand(HoodState state) {
  return runOnce([this, state]() { setHoodState(state); });
}

std::unique_ptr<Command> Hood::makeHoodStateForCommand(HoodState state,
                                                       QTime duration) {
  // makeStateForCommand caches the state directly, so an undecodable request
  // holds the current angle instead of parking on a state nothing can leave.
  return makeStateForCommand(isDecodable(state) ? state : getHoodState(), duration);
}

std::unique_ptr<Command> Hood::makeNextCommand() {
  return runOnce([this]() { next(); });
}

std::unique_ptr<Command> Hood::makePreviousCommand() {
  return runOnce([this]() { previous(); });
}

std::unique_ptr<Command> Hood::makeCycleCommand() {
  return runOnce([this]() { cycle(); });
}

void Hood::applyState(const HoodState& state) {
  if (!isDecodable(state)) {
    return;
  }

  const HoodOutput& output = m_config.decode[indexOf(state)];
  m_first.set_value(output.first);
  m_second.set_value(output.second);
}

bool Hood::isDecodable(HoodState state) {
  return indexOf(state) < kHoodStateCount;
}

std::size_t Hood::indexOf(HoodState state) {
  return static_cast<std::size_t>(state);
}

HoodState Hood::seedState(const HoodConfig& config) {
  return isDecodable(config.initial_state) ? config.initial_state : HoodState::Down;
}

}  // namespace mechanism
}  // namespace mclib
