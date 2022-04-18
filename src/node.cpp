/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>

#include <l3xz_teleop/Joystick.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  ros::init(argc, argv, "teleop_node");

  ros::NodeHandle node_hdl;
  ros::NodeHandle node_hdl_private("~");

  std::string joy_dev_node;

  node_hdl_private.param<std::string>("joy_dev_node", joy_dev_node, "/dev/input/js0");

  ROS_INFO("node config:\n  joy_dev_node: %s", joy_dev_node.c_str());

  Joystick joystick(joy_dev_node);

  while (ros::ok())
  {
    JoystickEvent const evt = joystick.update();

    if (evt.isAxis())
    {
      ROS_INFO("Axis %d: %d", evt.number, evt.value);
    }

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  ROS_ERROR("%s", err.what());
  return EXIT_FAILURE;
}
