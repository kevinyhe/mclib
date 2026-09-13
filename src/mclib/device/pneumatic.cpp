// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#include "mclib/device/pneumatic.hpp"

namespace mclib {
namespace device {

Pneumatic::Pneumatic(char adi_port, bool default_state, bool extended_state)
    : extender_port(0), adi_port(adi_port), default_state(default_state),
      extended_state(extended_state), current_state(default_state)
{
    const bool initial_raw_value = (default_state == extended_state);
    m_solenoid = std::make_shared<pros::adi::DigitalOut>(adi_port, initial_raw_value);
}

Pneumatic::Pneumatic(int extender_port, char adi_port, bool default_state,
                     bool extended_state)
    : extender_port(extender_port), adi_port(adi_port),
      default_state(default_state), extended_state(extended_state),
      current_state(default_state)
{
    const bool initial_raw_value = (default_state == extended_state);
    m_solenoid = std::make_shared<pros::adi::DigitalOut>(
        pros::adi::ext_adi_port_pair_t{static_cast<std::uint8_t>(extender_port),
                                       static_cast<std::uint8_t>(adi_port)},
        initial_raw_value);
}

void Pneumatic::set_value(bool value)
{
    current_state = value;
    if (m_solenoid)
    {
        const bool raw_value = (value == extended_state);
        m_solenoid->set_value(raw_value);
    }
}

void Pneumatic::extend()
{
    set_value(true);
}

void Pneumatic::retract()
{
    set_value(false);
}

void Pneumatic::toggle()
{
    set_value(!current_state);
}

bool Pneumatic::get_value()
{
    return current_state;
}

PneumaticGroup::PneumaticGroup(std::vector<std::shared_ptr<Pneumatic>> pneumatics)
    : pneumatics(std::move(pneumatics))
{
}

void PneumaticGroup::set_value(bool value)
{
    for (const auto &pneumatic : pneumatics)
    {
        if (pneumatic)
        {
            pneumatic->set_value(value);
        }
    }
}

void PneumaticGroup::extend()
{
    set_value(true);
}

void PneumaticGroup::retract()
{
    set_value(false);
}

void PneumaticGroup::toggle()
{
    set_value(!get_value());
}

bool PneumaticGroup::get_value()
{
    if (pneumatics.empty() || !pneumatics.front())
    {
        return false;
    }
    return pneumatics.front()->get_value();
}

std::vector<bool> PneumaticGroup::get_all_values()
{
    std::vector<bool> values;
    values.reserve(pneumatics.size());

    for (const auto &pneumatic : pneumatics)
    {
        values.push_back(pneumatic ? pneumatic->get_value() : false);
    }

    return values;
}

MockPneumatic::MockPneumatic(bool default_state) : current_state(default_state)
{
}

void MockPneumatic::set_value(bool value)
{
    current_state = value;
}

void MockPneumatic::extend()
{
    current_state = true;
}

void MockPneumatic::retract()
{
    current_state = false;
}

void MockPneumatic::toggle()
{
    current_state = !current_state;
}

bool MockPneumatic::get_value()
{
    return current_state;
}

}  // namespace device
}  // namespace mclib
