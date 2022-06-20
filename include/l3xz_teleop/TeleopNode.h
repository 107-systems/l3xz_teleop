/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "Joystick.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode();
  void update();


private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  std::shared_ptr<Joystick> _joystick;
  geometry_msgs::msg::Twist _twist_msg;
};
