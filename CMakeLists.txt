#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_teleop)
set(L3XZ_TELEOP_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_heartbeat REQUIRED)
#######################################################################################
include_directories(
  include
  external/sml/current
)
#######################################################################################
add_executable(${L3XZ_TELEOP_TARGET}
  src/teleop/Node.cpp
  src/main.cpp
)
#######################################################################################
target_compile_features(${L3XZ_TELEOP_TARGET} PRIVATE cxx_std_17)
ament_target_dependencies(${L3XZ_TELEOP_TARGET} rclcpp std_msgs sensor_msgs geometry_msgs ros2_heartbeat)
#######################################################################################
install(TARGETS
  ${L3XZ_TELEOP_TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
