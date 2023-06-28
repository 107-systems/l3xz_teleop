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

#include <boost/sml.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ros2_heartbeat/publisher/Publisher.h>

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
  ~Node();

private:
  ::heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  rclcpp::QoS _joy_qos_profile;
  rclcpp::SubscriptionOptions _joy_sub_options;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
  void init_sub();
  void onJoyMsg(sensor_msgs::msg::Joy::SharedPtr const joy_msg);

  geometry_msgs::msg::Twist _robot_msg;
  static geometry_msgs::msg::Twist create_init_robot_msg();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _robot_pub;
  rclcpp::TimerBase::SharedPtr _robot_pub_timer;
  void init_robot_pub();

  geometry_msgs::msg::Twist _head_msg;
  static geometry_msgs::msg::Twist create_init_head_msg();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _head_pub;
  rclcpp::TimerBase::SharedPtr _head_pub_timer;
  void init_head_pub();

  std_msgs::msg::Bool _req_up_msg;
  static std_msgs::msg::Bool create_init_req_up_msg();
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _robot_req_up_pub;
  rclcpp::TimerBase::SharedPtr _robot_req_up_pub_timer;
  void init_robot_req_up_pub();

  std_msgs::msg::Bool _req_down_msg;
  static std_msgs::msg::Bool create_init_req_down_msg();
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _robot_req_down_pub;
  rclcpp::TimerBase::SharedPtr _robot_req_down_pub_timer;
  void init_robot_req_down_pub();

  void init_pub();

  struct liveliness_gained { };
  struct liveliness_lost { };

  struct FsmImpl {
    auto operator()() const noexcept {
      using namespace boost::sml;
      return make_transition_table(
        *"standby"_s + event<liveliness_gained> /
          [](Node & node)
          {
            RCLCPP_INFO(node.get_logger(), "liveliness gained for \"%s\"", node._joy_sub->get_topic_name());
          }
          = "active"_s
        ,"active"_s + event<liveliness_lost> /
          [](Node & node)
          {
            RCLCPP_WARN(node.get_logger(), "liveliness lost for \"%s\"", node._joy_sub->get_topic_name());
            /* Set all teleop messages to be at their initial value. */
            node._robot_msg    = Node::create_init_robot_msg();
            node._head_msg     = Node::create_init_head_msg();
            node._req_up_msg   = Node::create_init_req_up_msg();
            node._req_down_msg = Node::create_init_req_down_msg();
          }  = "standby"_s
      );
    }
  };

  std::unique_ptr<boost::sml::sm<FsmImpl>> _sm;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
