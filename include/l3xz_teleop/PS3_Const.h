/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

#ifndef PS3_CONST_H_
#define PS3_CONST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum PS3_AxisId : uint8_t
{
  LEFT_STICK_HORIZONTAL = 0,
  LEFT_STICK_VERTICAL,
  LEFT_REAR_2,
  RIGHT_STICK_HORIZONTAL,
  RIGHT_STICK_VERTICAL,
  RIGHT_REAR_2,
};

enum PS3_ButtonId : uint8_t
{
  SELECT = 0,
  LEFT_JOYSTICK,
  RIGHT_JOYSTICK,
  START,
  DPAD_UP,
  DPAD_RIGHT,
  DPAD_DOWN,
  DPAD_LEFT,
  LEFT_TRIGGER,
  RIGHT_TRIGGER,
  LEFT_BUMPER,
  RIGHT_BUMPER,
  TRIANGLE,
  CIRCLE,
  X,
  SQUARE,
  PS3,
  PAD_UP = 13,
  PAD_DOWN = 14,
  PAD_RIGHT = 15,
  PAD_LEFT = 16,
};

#endif /* PS3_CONST_H_ */
