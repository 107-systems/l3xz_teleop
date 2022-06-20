/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_teleop/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_teleop/TeleopNode.h>

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
      rclcpp::spin(node);

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
