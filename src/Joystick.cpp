/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_teleop/Joystick.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <stdexcept>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Joystick::Joystick(std::string const & dev_node)
: _fd{open(dev_node.c_str(), O_RDONLY | O_NONBLOCK)}
{

}

Joystick::~Joystick()
{
  close(_fd);
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<JoystickEvent> Joystick::update()
{
	JoystickEvent evt;
  
  if (auto const bytes_read = read(_fd, &evt, sizeof(JoystickEvent));
      bytes_read != sizeof(JoystickEvent))
  {
    if (errno != EAGAIN) {
      throw std::runtime_error("Error while reading from joystick: " + std::string(strerror(errno)));
    }
    return std::nullopt;
  }

  return evt;
}
