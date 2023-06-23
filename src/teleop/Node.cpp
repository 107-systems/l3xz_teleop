/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_teleop/Node.h>

#include <limits>
#include <numeric>
#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_teleop")
, _joy_qos_profile
{
  rclcpp::KeepLast(10),
  rmw_qos_profile_sensor_data
}
, _robot_msg
{
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
  }()
}
, _head_msg
{
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
  }()
}
, _req_up_msg
{
  []()
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    return msg;
  }()
}
, _req_down_msg
{
  []()
  {
    std_msgs::msg::Bool msg;
    msg.data = false;
    return msg;
  }()
}
{
  declare_parameter("joy_topic", "joy");
  declare_parameter("joy_topic_deadline_ms", 100);
  declare_parameter("robot_topic", "cmd_vel_robot");
  declare_parameter("head_topic", "cmd_vel_head");
  declare_parameter("robot_req_up_topic", "cmd_robot/req_up");
  declare_parameter("robot_req_down_topic", "cmd_robot/req_down");
  declare_parameter("pan_max_dps", 10.0f);
  declare_parameter("tilt_max_dps", 10.0f);

  init_heartbeat();
  init_sub();
  init_pub();

  _teleop_pub_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]()
    {
      _robot_pub->publish(_robot_msg);
      _head_pub->publish(_head_msg);
      _robot_req_up_pub->publish(_req_up_msg);
      _robot_req_down_pub->publish(_req_down_msg);
    });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str(), HEARTBEAT_LOOP_RATE);
}

void Node::init_sub()
{
  auto const joy_topic = get_parameter("joy_topic").as_string();
  auto const joy_topic_deadline = std::chrono::milliseconds(get_parameter("joy_topic_deadline_ms").as_int());

  _joy_qos_profile.deadline(joy_topic_deadline);

  _joy_sub_options.event_callbacks.deadline_callback =
    [this, joy_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR(get_logger(),
                   "Dead line configured for topic %s missed (total_count: %d, total_count_change: %d).",
                   joy_topic.c_str(),
                   event.total_count,
                   event.total_count_change);
    };

  _joy_sub = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic,
    _joy_qos_profile,
    [this](sensor_msgs::msg::Joy::SharedPtr const joy_msg)
    {
      updateRobotMessage       (joy_msg);
      updateHeadMessage        (joy_msg);
      updateRobotReqUpMessage  (joy_msg);
      updateRobotReqDownMessage(joy_msg);
    },
    _joy_sub_options);
}

void Node::init_pub()
{
  _robot_pub = create_publisher<geometry_msgs::msg::Twist>(
    get_parameter("robot_topic").as_string(), 10);

  _head_pub = create_publisher<geometry_msgs::msg::Twist>(
    get_parameter("head_topic").as_string(), 10);

  _robot_req_up_pub = create_publisher<std_msgs::msg::Bool>(
    get_parameter("robot_req_up_topic").as_string(), 1);

  _robot_req_down_pub = create_publisher<std_msgs::msg::Bool>(
    get_parameter("robot_req_down_topic").as_string(), 1);
}

void Node::updateRobotMessage(sensor_msgs::msg::Joy::SharedPtr const joy_msg)
{
  _robot_msg.linear.x  = (-1.0f) * joy_msg->axes[1]; /* LEFT_STICK_VERTICAL   */
  _robot_msg.angular.z =           joy_msg->axes[0]; /* LEFT_STICK_HORIZONTAL */
}

void Node::updateHeadMessage(sensor_msgs::msg::Joy::SharedPtr const joy_msg)
{
  float const pan_angular_velocity_dps  =           joy_msg->axes[3] * get_parameter("pan_max_dps").as_double();  /* RIGHT_STICK_HORIZONTAL */
  float const tilt_angular_velocity_dps = (-1.0f) * joy_msg->axes[4] * get_parameter("tilt_max_dps").as_double(); /* RIGHT_STICK_VERTICAL   */

  _head_msg.angular.z = pan_angular_velocity_dps  * M_PI / 180.0f;
  _head_msg.angular.y = tilt_angular_velocity_dps * M_PI / 180.0f;
}

void Node::updateRobotReqUpMessage(sensor_msgs::msg::Joy::SharedPtr const joy_msg)
{
  _req_up_msg.data = joy_msg->buttons[0] != 0;
}

void Node::updateRobotReqDownMessage(sensor_msgs::msg::Joy::SharedPtr const joy_msg)
{
  _req_down_msg.data = joy_msg->buttons[1] != 0;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
