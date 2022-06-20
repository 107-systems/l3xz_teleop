/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <string>
#include <chrono>
#include <memory>
#include <limits>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <l3xz_teleop/Joystick.h>
#include <l3xz_teleop/PS3_Const.h>

/**************************************************************************************
 * NODE IMPLEMENTATION 
 **************************************************************************************/
class TeleopNode : public rclcpp::Node
{
  public:
    TeleopNode()
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
    {
      declare_parameter("joy_dev_node", "/dev/input/js0");
      declare_parameter("topic_robot_velocity", "cmd_vel");
      _joystick = std::make_shared<Joystick>(get_parameter("joy_dev_node").as_string());
      _publisher = create_publisher<geometry_msgs::msg::Twist>(get_parameter("topic_robot_velocity").as_string(), 25);
    }

    void update()
    {
      static bool initialized = false;
      static auto prev = std::chrono::steady_clock::now();

      std::optional<JoystickEvent> const evt = _joystick->update();

      if (evt)
      {
        std::map<PS3_AxisId, float> axis_data;

        if (!initialized)
        {
          initialized = evt.value().isInit();
        }
        else if (evt.value().isAxis())
        {
          RCLCPP_INFO(get_logger(), "Axis %d: %d", evt.value().number, evt.value().value);

          PS3_AxisId const axis_id = static_cast<PS3_AxisId>(evt.value().number);
          float const axis_scaled_val = static_cast<float>(evt.value().value) / static_cast<float>(std::numeric_limits<int16_t>::max());
          axis_data[axis_id] = axis_scaled_val;
        }

        if (evt.value().isButton())
        {
          RCLCPP_INFO(get_logger(), "Button %d: %d", evt.value().number, evt.value().value);
        }

        auto const now = std::chrono::steady_clock::now();
        auto const elapsed = (now - prev);

        if (elapsed > std::chrono::milliseconds(50))
        {
          prev = now;
          float linear_velocity_x = _twist_msg.linear.x;
          if (axis_data.count(PS3_AxisId::LEFT_STICK_VERTICAL))
            linear_velocity_x = -1.0f * axis_data[PS3_AxisId::LEFT_STICK_VERTICAL];

          float linear_velocity_y = _twist_msg.linear.y;
          if (axis_data.count(PS3_AxisId::LEFT_STICK_HORIZONTAL))
            linear_velocity_y = axis_data[PS3_AxisId::LEFT_STICK_HORIZONTAL];

          float angular_velocity_head_tilt = _twist_msg.angular.x;
          if (axis_data.count(PS3_AxisId::RIGHT_STICK_VERTICAL))
            angular_velocity_head_tilt = -1.0f * axis_data[PS3_AxisId::RIGHT_STICK_VERTICAL];

          float angular_velocity_head_pan = _twist_msg.angular.y;
          if (axis_data.count(PS3_AxisId::RIGHT_STICK_HORIZONTAL))
            angular_velocity_head_pan = axis_data[PS3_AxisId::RIGHT_STICK_HORIZONTAL];

          float angular_velocity_z = 0.0f;
          if (axis_data.count(PS3_AxisId::LEFT_REAR_2))
            angular_velocity_z -= (axis_data[PS3_AxisId::LEFT_REAR_2] + 1.0f) / 2.0f;
          if (axis_data.count(PS3_AxisId::RIGHT_REAR_2))
            angular_velocity_z += (axis_data[PS3_AxisId::RIGHT_REAR_2] + 1.0f) / 2.0f;

          _twist_msg.linear.x  = linear_velocity_x;
          _twist_msg.linear.y  = linear_velocity_y;
          _twist_msg.linear.z  = 0.0;

          _twist_msg.angular.x = angular_velocity_head_tilt;
          _twist_msg.angular.y = angular_velocity_head_pan;
          _twist_msg.angular.z = angular_velocity_z;

          _publisher->publish(_twist_msg);
        }
      }
    }
  private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    std::shared_ptr<Joystick> _joystick;
    geometry_msgs::msg::Twist _twist_msg;
};

/**************************************************************************************
 * MAIN
 **************************************************************************************/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<TeleopNode> node = std::make_shared<TeleopNode>();

  try
  {
    while(rclcpp::ok())
    {
      node->update();
      rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error const & err)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
    return EXIT_FAILURE;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Unhandled exception caught.\nTerminating ...");
    return EXIT_FAILURE;
  }
}
