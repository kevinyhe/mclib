// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include "pros/adi.hpp"

#include <memory>
#include <vector>

namespace mclib {
namespace device {

/**
 * An abstract Pneumatic
 */
class IPneumatic
{
public:
  virtual void set_value(bool value) = 0;
  virtual void extend() = 0;
  virtual void retract() = 0;
  virtual void toggle() = 0;
  virtual bool get_value() = 0;
};

class Pneumatic : public IPneumatic
{
private:
  int extender_port;
  char adi_port;

  bool default_state;
  bool extended_state;
  bool current_state;

  std::shared_ptr<pros::adi::DigitalOut> m_solenoid;

public:
  /**
   * @brief Construct a new Pneumatic
   *
   * @param adi_port The 3 wire port the pneumatic is connected to
   * @param default_state The default state of the pneumatic
   * @param extended_state Whether the pneumatic is extended when the solenoid
   * is set to true
   */
  Pneumatic(char adi_port, bool default_state = false,
            bool extended_state = true);

  /**
   * @brief Construct a new Pneumatic
   *
   * @param extender_port The smart port the 3 wire expander is in
   * @param adi_port The 3 wire port the pneumatic is connected to
   * @param default_state The default state of the pneumatic
   * @param extended_state Whether the pneumatic is extended when the solenoid
   * is set to true
   */
  Pneumatic(int extender_port, char adi_port, bool default_state = false,
            bool extended_state = true);

  /**
   * @brief Set the value of the pneumatic
   *
   * @param value The value to set it to
   */
  void set_value(bool value);

  /**
   * @brief Extend the pneumatic
   *
   */
  void extend();

  /**
   * @brief Retract the pneumatic
   *
   */
  void retract();

  /**
   * @brief Toggle the value of the pneumatic
   *
   */
  void toggle();

  /**
   * @brief Get the value of the pneumatic
   *
   * @return true if the pneumatic is extended
   * @return false if the pneumatic is retracted
   */
  bool get_value();
};

class PneumaticGroup : public IPneumatic
{
private:
  std::vector<std::shared_ptr<Pneumatic>> pneumatics;

public:
  /**
   * @brief Construct a new Pneumatic Group
   *
   * @param pneumatics The pneumatics in the group
   */
  PneumaticGroup(std::vector<std::shared_ptr<Pneumatic>> pneumatics);

  /**
   * @brief Set the value of the pneumatics
   *
   * @param value The value to set it to
   */
  void set_value(bool value);

  /**
   * @brief Extend the pneumatics
   *
   */
  void extend();

  /**
   * @brief Retract the pneumatics
   *
   */
  void retract();

  /**
   * @brief Toggle the value of the pneumatics
   *
   */
  void toggle();

  /**
   * @brief Get the value of the first pneumatic
   *
   * @return true if the pneumatic is extended
   * @return false if the pneumatic is retracted
   */
  bool get_value();

  /**
   * @brief Get the values of all the pneumatics
   *
   * @return true if the pneumatics are extended
   * @return false if the pneumatics are retracted
   */
  std::vector<bool> get_all_values();
};

class MockPneumatic : public IPneumatic
{
private:
  bool current_state;

public:
  /**
   * @brief Construct a new Pneumatic
   *
   * @param default_state The default state of the pneumatic
   */
  MockPneumatic(bool default_state = false);

  /**
   * @brief Set the value of the pneumatic
   *
   * @param value The value to set it to
   */
  void set_value(bool value);

  /**
   * @brief Extend the pneumatic
   *
   */
  void extend();

  /**
   * @brief Retract the pneumatic
   *
   */
  void retract();

  /**
   * @brief Toggle the value of the pneumatic
   *
   */
  void toggle();

  /**
   * @brief Get the value of the pneumatic
   *
   * @return true if the pneumatic is extended
   * @return false if the pneumatic is retracted
   */
  bool get_value();
};

}  // namespace device
}  // namespace mclib
