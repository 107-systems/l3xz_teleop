/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_teleop/TeleopNode.h>

#include <chrono>
#include <limits>
#include <numeric>

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

TeleopNode::TeleopNode()
: Node("l3xz_teleop")
, _robot_msg{
    []()
    {
      geometry_msgs::msg::Twist msg;
      msg.linear.x  = 0.0;
      msg.linear.y  = 0.0;
      msg.linear.z  = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      return msg;
    } ()
  }
, _head_msg{
    []()
    {
      geometry_msgs::msg::Twist msg;
      msg.linear.x  = 0.0;
      msg.linear.y  = 0.0;
      msg.linear.z  = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      return msg;
    } ()
  }
{
  declare_parameter("joy_topic", "joy");
  declare_parameter("robot_topic", "cmd_vel_robot");
  declare_parameter("head_topic", "cmd_vel_head");

  _joy_sub = create_subscription<sensor_msgs::msg::Joy>
    (get_parameter("joy_topic").as_string(), 10, [this](sensor_msgs::msg::Joy::SharedPtr const msg)
                                                 {
                                                   updateRobotMessage(*msg);
                                                   updateHeadMessage (*msg);
                                                 });

  _robot_pub = create_publisher<geometry_msgs::msg::Twist>
    (get_parameter("robot_topic").as_string(), 10);
  
  _head_pub = create_publisher<geometry_msgs::msg::Twist>
    (get_parameter("head_topic").as_string(), 10);

  _teleop_pub_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->teleopPubFunc(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void TeleopNode::teleopPubFunc()
{
  _robot_pub->publish(_robot_msg);
  _head_pub->publish(_head_msg);
}

void TeleopNode::updateRobotMessage(sensor_msgs::msg::Joy const & joy_msg)
{
  _robot_msg.linear.x  = (-1.0f) * joy_msg.axes[1]; /* LEFT_STICK_VERTICAL   */
  _robot_msg.angular.z =           joy_msg.axes[0]; /* LEFT_STICK_HORIZONTAL */
}

void TeleopNode::updateHeadMessage(sensor_msgs::msg::Joy const & joy_msg)
{
  _head_msg.angular.y = (-1.0f) * joy_msg.axes[4]; /* RIGHT_STICK_VERTICAL   */
  _head_msg.angular.z =           joy_msg.axes[3]; /* RIGHT_STICK_HORIZONTAL */
}
