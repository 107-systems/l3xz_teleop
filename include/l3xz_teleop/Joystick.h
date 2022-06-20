/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <optional>

#include "JoystickEvent.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Joystick
{
public:

   Joystick(std::string const & dev_node);
  ~Joystick();

  JoystickEvent update();

private:
  int _fd;
};

#endif /* JOYSTICK_H_ */
