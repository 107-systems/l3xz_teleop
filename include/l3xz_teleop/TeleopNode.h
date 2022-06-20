/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "Joystick.h"
#include "PS3_Const.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TeleopNode : public rclcpp::Node
{
public:
   TeleopNode();
  ~TeleopNode();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _pub_timer;
  geometry_msgs::msg::Twist _twist_msg;

  std::shared_ptr<Joystick> _joystick;
  std::map<PS3_AxisId, float> _joystick_data;
  std::thread _joy_thread;
  std::atomic<bool> _joy_thread_active;

  void joystick_thread_func();
  void pub_timer_callback();
};
