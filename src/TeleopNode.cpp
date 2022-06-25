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
,_msg_stick{
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
, _msg_pad{
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
  declare_parameter("topic_stick", "cmd_vel");
  declare_parameter("topic_pad", "cmd_vel_pad");
  declare_parameter("x_max_stick", 3.0);
  declare_parameter("y_max_stick", 3.0);
  declare_parameter("angular_x_max_stick", 1.0);
  declare_parameter("angular_z_max_stick", 1.0);
  declare_parameter("x_pad", 1.0);
  declare_parameter("y_pad", 1.0);

  _teleop_stick_pub = create_publisher<geometry_msgs::msg::Twist>
    (get_parameter("topic_stick").as_string(), 10);
  
  _teleop_pad_pub = create_publisher<geometry_msgs::msg::Twist>
    (get_parameter("topic_pad").as_string(), 10);

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
      
      PS3_ButtonId const button_id = static_cast<PS3_ButtonId>(evt.number);
      std::lock_guard<std::mutex> lock(_joy_mtx);
      _button_data[button_id] = static_cast<bool>(evt.value);
    }
  }
}

void TeleopNode::teleopTimerCallback()
{
  {
    std::lock_guard<std::mutex> lock(_joy_mtx);
  
    if (_joystick_data.count(PS3_AxisId::LEFT_STICK_VERTICAL))
      _msg_stick.linear.x = -1.0f * get_parameter("x_max_stick").as_double() * _joystick_data[PS3_AxisId::LEFT_STICK_VERTICAL];

    if (_joystick_data.count(PS3_AxisId::LEFT_STICK_HORIZONTAL))
      _msg_stick.linear.y = get_parameter("y_max_stick").as_double() * _joystick_data[PS3_AxisId::LEFT_STICK_HORIZONTAL];

    if (_joystick_data.count(PS3_AxisId::RIGHT_STICK_VERTICAL))
      _msg_stick.angular.x = get_parameter("angular_x_max_stick").as_double() * _joystick_data[PS3_AxisId::RIGHT_STICK_VERTICAL];
    
    if (_joystick_data.count(PS3_AxisId::RIGHT_STICK_HORIZONTAL))
      _msg_stick.angular.z = get_parameter("angular_z_max_stick").as_double() * _joystick_data[PS3_AxisId::RIGHT_STICK_HORIZONTAL];

    if (_button_data.count(PS3_ButtonId::PAD_UP) && _button_data[PS3_ButtonId::PAD_UP])
    {
      _msg_pad.linear.x = get_parameter("x_pad").as_double() * static_cast<double>(_button_data[PS3_ButtonId::PAD_UP]);
    }
    else if (_button_data.count(PS3_ButtonId::PAD_DOWN) && _button_data[PS3_ButtonId::PAD_DOWN])
    {
      _msg_pad.linear.x = -1.0f * get_parameter("x_pad").as_double() * static_cast<double>(_button_data[PS3_ButtonId::PAD_DOWN]);
    }
    else
    {
      _msg_pad.linear.x = 0.0f;
    }

    if (_button_data.count(PS3_ButtonId::PAD_LEFT) && _button_data[PS3_ButtonId::PAD_LEFT])
    {
      _msg_pad.linear.y = get_parameter("y_pad").as_double() * static_cast<double>(_button_data[PS3_ButtonId::PAD_LEFT]);
    }
    else if (_button_data.count(PS3_ButtonId::PAD_RIGHT) && _button_data[PS3_ButtonId::PAD_RIGHT])
    {
      _msg_pad.linear.y = -1.0f * get_parameter("y_pad").as_double() * static_cast<double>(_button_data[PS3_ButtonId::PAD_RIGHT]);
    }
    else
    {
      _msg_pad.linear.y = 0.0f;
    }
  }

  _teleop_stick_pub->publish(_msg_stick);
  _teleop_pad_pub->publish(_msg_pad);
}
