/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

#ifndef JOYSTICK_EVENT_H_
#define JOYSTICK_EVENT_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class JoystickEvent
{
public:

  static uint8_t constexpr EVENT_BUTTON  = 0x01;
  static uint8_t constexpr EVENT_AXIS    = 0x02;
  static uint8_t constexpr EVENT_INIT    = 0x80;

  uint32_t time;     /* event timestamp in milliseconds */
  int16_t  value;    /* value */
  uint8_t  type;      /* event type */
  uint8_t  number;    /* axis/button number */

  inline bool isButton() const { return ((type & EVENT_BUTTON) == EVENT_BUTTON); }
  inline bool isAxis  () const { return ((type & EVENT_AXIS)   == EVENT_AXIS); }
  inline bool isInit  () const { return ((type & EVENT_INIT)   == EVENT_INIT); }
};

#endif /* JOYSTICK_EVENT_H_ */
