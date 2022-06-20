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
: Node("teleop")
, _twist_msg{
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
, _joy_thread{}
, _joy_thread_active{false}
{
  declare_parameter("joy_dev_node", "/dev/input/js0");
  declare_parameter("topic_robot_velocity", "cmd_vel");
  _publisher = create_publisher<geometry_msgs::msg::Twist>(get_parameter("topic_robot_velocity").as_string(), 25);
  _pub_timer = create_wall_timer(std::chrono::milliseconds(50), [this]() { this->pub_timer_callback(); });

  _joystick = std::make_shared<Joystick>(get_parameter("joy_dev_node").as_string());
  _joy_thread = std::thread([this]() { this->joystick_thread_func(); });
}

TeleopNode::~TeleopNode()
{
  _joy_thread_active = false;
  _joy_thread.join();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void TeleopNode::joystick_thread_func()
{
  _joy_thread_active = true;

  while (_joy_thread_active)
  {
    JoystickEvent const evt = _joystick->update();

    if (evt.isInit())
      continue;

    if (evt.isAxis())
    {
      RCLCPP_INFO(get_logger(), "Axis %d: %d", evt.number, evt.value);

      PS3_AxisId const axis_id = static_cast<PS3_AxisId>(evt.number);
      float const axis_scaled_val = static_cast<float>(evt.value) / static_cast<float>(std::numeric_limits<int16_t>::max());
      _axis_data[axis_id] = axis_scaled_val;
    }

    if (evt.isButton()) {
      RCLCPP_INFO(get_logger(), "Button %d: %d", evt.number, evt.value);
    }
  }
}

void TeleopNode::pub_timer_callback()
{
  float linear_velocity_x = _twist_msg.linear.x;
  if (_axis_data.count(PS3_AxisId::LEFT_STICK_VERTICAL))
    linear_velocity_x = -1.0f * _axis_data[PS3_AxisId::LEFT_STICK_VERTICAL];

  float linear_velocity_y = _twist_msg.linear.y;
  if (_axis_data.count(PS3_AxisId::LEFT_STICK_HORIZONTAL))
    linear_velocity_y = _axis_data[PS3_AxisId::LEFT_STICK_HORIZONTAL];

  float angular_velocity_head_tilt = _twist_msg.angular.x;
  if (_axis_data.count(PS3_AxisId::RIGHT_STICK_VERTICAL))
    angular_velocity_head_tilt = -1.0f * _axis_data[PS3_AxisId::RIGHT_STICK_VERTICAL];

  float angular_velocity_head_pan = _twist_msg.angular.y;
  if (_axis_data.count(PS3_AxisId::RIGHT_STICK_HORIZONTAL))
    angular_velocity_head_pan = _axis_data[PS3_AxisId::RIGHT_STICK_HORIZONTAL];

  float angular_velocity_z = 0.0f;
  if (_axis_data.count(PS3_AxisId::LEFT_REAR_2))
    angular_velocity_z -= (_axis_data[PS3_AxisId::LEFT_REAR_2] + 1.0f) / 2.0f;
  if (_axis_data.count(PS3_AxisId::RIGHT_REAR_2))
    angular_velocity_z += (_axis_data[PS3_AxisId::RIGHT_REAR_2] + 1.0f) / 2.0f;

  _twist_msg.linear.x  = linear_velocity_x;
  _twist_msg.linear.y  = linear_velocity_y;
  _twist_msg.linear.z  = 0.0;

  _twist_msg.angular.x = angular_velocity_head_tilt;
  _twist_msg.angular.y = angular_velocity_head_pan;
  _twist_msg.angular.z = angular_velocity_z;

  _publisher->publish(_twist_msg);
}
