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
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::teleop
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();

private:
  rclcpp::TimerBase::SharedPtr _teleop_pub_timer;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _robot_pub;
  geometry_msgs::msg::Twist _robot_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _head_pub;
  geometry_msgs::msg::Twist _head_msg;

  void updateRobotMessage(sensor_msgs::msg::Joy const & joy_msg);
  void updateHeadMessage (sensor_msgs::msg::Joy const & joy_msg);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::teleop */
