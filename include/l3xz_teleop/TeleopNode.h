/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <mutex>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <l3xz_teleop/msg/teleop.hpp>

#include "Joystick.h"
#include "PS3_Const.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TeleopNode : public rclcpp::Node
{
public:
   TeleopNode();
  ~TeleopNode();

private:
  rclcpp::Publisher<l3xz_teleop::msg::Teleop>::SharedPtr _teleop_pub;
  rclcpp::TimerBase::SharedPtr _teleop_pub_timer;
  l3xz_teleop::msg::Teleop _teleop_msg;

  std::shared_ptr<Joystick> _joystick;
  std::map<PS3_AxisId, float> _joystick_data;
  std::mutex _joy_mtx;
  std::thread _joy_thread;
  std::atomic<bool> _joy_thread_active;

  void joystickThreadFunc();
  void teleopTimerCallback();
};
