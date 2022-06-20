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

  void update();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  std::shared_ptr<Joystick> _joystick;
  geometry_msgs::msg::Twist _twist_msg;
  std::map<PS3_AxisId, float> _axis_data;

  void update_joystick();
  void update_ros();
};
