/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode();

private:
  rclcpp::TimerBase::SharedPtr _teleop_pub_timer;
 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
  sensor_msgs::msg::Joy _joy_msg;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _robot_pub;
  geometry_msgs::msg::Twist _robot_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _head_pub;
  geometry_msgs::msg::Twist _head_msg;

  void teleopPubFunc();
};
