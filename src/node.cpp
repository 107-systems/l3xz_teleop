/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>

#include <l3xz_teleop/Joystick.h>
#include <l3xz_teleop/PS3_Const.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_node");

  ros::NodeHandle node_hdl;
  ros::NodeHandle node_hdl_private("~");

  std::string joy_dev_node, joy_topic;

  node_hdl_private.param<std::string>("joy_dev_node", joy_dev_node, "/dev/input/js0");
  node_hdl_private.param<std::string>("joy_topic", joy_topic, "cmd_vel");

  ROS_INFO("node config:\n  joy_dev_node: %s\n  joy_topic   : %s", joy_dev_node.c_str(), joy_topic.c_str());

  ros::Publisher cmd_vel_pub = node_hdl.advertise<geometry_msgs::Twist>(joy_topic, 10);

  try
  {
    Joystick joystick(joy_dev_node);

    for (auto prev = std::chrono::steady_clock::now(); ros::ok(); )
    {
      /* Read data from the joystick and pre-process it. */
      std::optional<JoystickEvent> const evt = joystick.update();

      if (evt)
      {
        if (evt.value().isAxis())
        {
          ROS_INFO("Axis %d: %d", evt.value().number, evt.value().value);
        }
      }

      /* Transmit control messages at a defined interval. */
      auto const now = std::chrono::steady_clock::now();
      auto const elapsed = (now - prev);

      if (elapsed > std::chrono::milliseconds(50))
      {
        prev = now;

        geometry_msgs::Twist msg;

        msg.linear.x  =  1.0;
        msg.linear.y  = -1.0;
        msg.linear.z  =  0.0;

        msg.angular.x =  1.0;
        msg.angular.y = -1.0;
        msg.angular.z =  0.0;

        cmd_vel_pub.publish(msg);
      }

      ros::spinOnce();
    }
  }
  catch (std::runtime_error const & err)
  {
    ROS_ERROR("%s", err.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
