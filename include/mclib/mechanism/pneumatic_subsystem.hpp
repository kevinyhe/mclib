// mclib
#pragma once

#include "mclib/device/pneumatic.hpp"
#include "mclib/mechanism/mechanism.hpp"

#include <initializer_list>
#include <memory>
#include <vector>

namespace mclib {
namespace mechanism {

class PneumaticSubsystem : public StateMechanism<bool> {
public:
  PneumaticSubsystem(std::initializer_list<char> adi_ports,
                     bool default_state = false,
                     bool extended_state = true);
  PneumaticSubsystem(std::vector<std::shared_ptr<device::Pneumatic>> pneumatics,
                     bool initial_state = false);

  void setExtended(bool extended);
  bool isExtended() const;
  void extend();
  void retract();
  void toggle();

  std::unique_ptr<Command> makeSetCommand(bool extended);
  std::unique_ptr<Command> makeExtendCommand();
  std::unique_ptr<Command> makeRetractCommand();
  std::unique_ptr<Command> makeToggleCommand();

protected:
  void applyState(const bool& extended) override;

private:
  static std::vector<std::shared_ptr<device::Pneumatic>> makePneumatics(
      std::initializer_list<char> adi_ports,
      bool default_state,
      bool extended_state);

  std::vector<std::shared_ptr<device::Pneumatic>> m_pneumatics;
  device::PneumaticGroup m_group;
};

}  // namespace mechanism
}  // namespace mclib
