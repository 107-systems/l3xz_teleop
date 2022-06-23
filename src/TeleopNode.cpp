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
, _teleop_msg{
    []()
    {
      l3xz_teleop::msg::Teleop msg;
      msg.linear_velocity_x          = 0.0f;
      msg.linear_velocity_y          = 0.0f;
      msg.angular_velocity_z         = 0.0f;
      msg.angular_velocity_head_tilt = 0.0f;
      msg.angular_velocity_head_pan  = 0.0f;
      return msg;
    } ()
  }
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
, _joy_mtx{}
, _joy_thread{}
, _joy_thread_active{false}
{
  declare_parameter("joy_dev_node", "/dev/input/js0");
  declare_parameter("topic_robot_velocity", "cmd_vel");
  declare_parameter("use_twist_message", false);
  declare_parameter("x_maxspeed_m/sec", 3.0);
  declare_parameter("y_maxspeed_m/sec", 3.0);
  declare_parameter("angular_maxspeed_rad/sec", 1.0);

  _use_twist_msg = get_parameter("use_twist_message").as_bool();
  
  if (_use_twist_msg)
  {
    _teleop_twist_pub = create_publisher<geometry_msgs::msg::Twist>
      (get_parameter("topic_robot_velocity").as_string(), 10);
  }
  else
  {
    _teleop_pub = create_publisher<l3xz_teleop::msg::Teleop>
      (get_parameter("topic_robot_velocity").as_string(), 10);
  }

  _teleop_pub_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->teleopTimerCallback(); });

  _joystick   = std::make_shared<Joystick>(get_parameter("joy_dev_node").as_string());
  _joy_thread = std::thread([this]() { this->joystickThreadFunc(); });
}

TeleopNode::~TeleopNode()
{
  _joy_thread_active = false;
  _joy_thread.join();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void TeleopNode::joystickThreadFunc()
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

      std::lock_guard<std::mutex> lock(_joy_mtx);
      _joystick_data[axis_id] = axis_scaled_val;
    }

    if (evt.isButton()) {
      RCLCPP_INFO(get_logger(), "Button %d: %d", evt.number, evt.value);
    }
  }
}

void TeleopNode::publishTeleop()
{
  {
    std::lock_guard<std::mutex> lock(_joy_mtx);
  
    if (_joystick_data.count(PS3_AxisId::LEFT_STICK_VERTICAL))
      _teleop_msg.linear_velocity_x = -1.0f * _joystick_data[PS3_AxisId::LEFT_STICK_VERTICAL];

    if (_joystick_data.count(PS3_AxisId::LEFT_STICK_HORIZONTAL))
      _teleop_msg.linear_velocity_y = _joystick_data[PS3_AxisId::LEFT_STICK_HORIZONTAL];

    if (_joystick_data.count(PS3_AxisId::RIGHT_STICK_VERTICAL))
      _teleop_msg.angular_velocity_head_tilt = -1.0f * _joystick_data[PS3_AxisId::RIGHT_STICK_VERTICAL];

    if (_joystick_data.count(PS3_AxisId::RIGHT_STICK_HORIZONTAL))
      _teleop_msg.angular_velocity_head_pan = _joystick_data[PS3_AxisId::RIGHT_STICK_HORIZONTAL];

    if (_joystick_data.count(PS3_AxisId::LEFT_REAR_2))
      _teleop_msg.angular_velocity_z -= (_joystick_data[PS3_AxisId::LEFT_REAR_2] + 1.0f) / 2.0f;
    if (_joystick_data.count(PS3_AxisId::RIGHT_REAR_2))
      _teleop_msg.angular_velocity_z += (_joystick_data[PS3_AxisId::RIGHT_REAR_2] + 1.0f) / 2.0f;
  }

  _teleop_pub->publish(_teleop_msg);
}

void TeleopNode::publishTwist()
{
  {
    std::lock_guard<std::mutex> lock(_joy_mtx);
  
    if (_joystick_data.count(PS3_AxisId::LEFT_STICK_VERTICAL))
      _twist_msg.linear.x = -1.0f * get_parameter("x_maxspeed_m/sec").as_double() * _joystick_data[PS3_AxisId::LEFT_STICK_VERTICAL];

    if (_joystick_data.count(PS3_AxisId::LEFT_STICK_HORIZONTAL))
      _twist_msg.linear.y = get_parameter("y_maxspeed_m/sec").as_double() *  _joystick_data[PS3_AxisId::LEFT_STICK_HORIZONTAL];

    if (_joystick_data.count(PS3_AxisId::RIGHT_STICK_HORIZONTAL))
      _twist_msg.angular.z = get_parameter("angular_maxspeed_rad/sec").as_double() *  _joystick_data[PS3_AxisId::RIGHT_STICK_HORIZONTAL];

  }

  _teleop_twist_pub->publish(_twist_msg);
}

void TeleopNode::teleopTimerCallback()
{
  if(_use_twist_msg)
  {
    publishTwist();
  }
  else
  {
    publishTeleop();
  }
}
