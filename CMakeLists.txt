#######################################################################################
cmake_minimum_required(VERSION 3.3.0)
#######################################################################################
project(l3xz_teleop)
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
#######################################################################################
include_directories(
  include
)
#######################################################################################
add_executable(${PROJECT_NAME}_node
  src/Joystick.cpp
  src/node.cpp
  src/TeleopNode.cpp
)
#######################################################################################
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs tf2)
#######################################################################################
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
