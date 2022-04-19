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
#include <vector>
#include <limits>
#include <numeric>

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

    std::map<PS3_AxisId, std::vector<float>> axis_data_vect;

    for (auto prev = std::chrono::steady_clock::now(); ros::ok(); )
    {
      /* Read data from the joystick and pre-process it. */
      std::optional<JoystickEvent> const evt = joystick.update();

      /* Process the received event. */
      if (evt)
      {
        if (evt.value().isInit())
          continue;

        if (evt.value().isAxis())
        {
          ROS_INFO("Axis %d: %d", evt.value().number, evt.value().value);

          PS3_AxisId const axis_id = static_cast<PS3_AxisId>(evt.value().number);
          float const axis_scaled_val = static_cast<float>(evt.value().value) / static_cast<float>(std::numeric_limits<int16_t>::max());
          axis_data_vect[axis_id].push_back(axis_scaled_val);
        }

        if (evt.value().isButton())
        {
          ROS_INFO("Button %d: %d", evt.value().number, evt.value().value);
        }
      }

      /* Transmit control messages at a defined interval. */
      auto const now = std::chrono::steady_clock::now();
      auto const elapsed = (now - prev);

      if (elapsed > std::chrono::milliseconds(50))
      {
        prev = now;

        geometry_msgs::Twist msg;

        auto getAvgFn = [&axis_data_vect](PS3_AxisId const id) -> float
                        {
                          if (!axis_data_vect.count(id))
                            return 0.0f;

                          if (!axis_data_vect.at(id).size())
                            return 0.0f;

                          float const avg_axis_val = std::accumulate(axis_data_vect.at(id).begin(),
                                                                     axis_data_vect.at(id).end(),
                                                                     0.0f)
                                                     /
                                                     axis_data_vect.at(id).size();

                          axis_data_vect.at(id).clear();

                          return avg_axis_val;
                        };

        msg.linear.x  = getAvgFn(PS3_AxisId::LEFT_STICK_VERTICAL);
        msg.linear.y  = getAvgFn(PS3_AxisId::LEFT_STICK_HORIZONTAL);
        msg.linear.z  = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

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
