/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ros2_heartbeat/Publisher.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();

private:
  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  ::heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  rclcpp::TimerBase::SharedPtr _teleop_pub_timer;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;

  geometry_msgs::msg::Twist _robot_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _robot_pub;
  geometry_msgs::msg::Twist _head_msg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _head_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _robot_req_up_pub, _robot_req_down_pub;
  void init_pub();

  void updateRobotMessage(sensor_msgs::msg::Joy const & joy_msg);
  void updateHeadMessage (sensor_msgs::msg::Joy const & joy_msg);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
