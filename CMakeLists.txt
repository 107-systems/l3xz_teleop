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
find_package(rosidl_default_generators REQUIRED)
#######################################################################################
include_directories(
  include
)
#######################################################################################
add_executable(${L3XZ_TELEOP_TARGET}
  src/Joystick.cpp
  src/main.cpp
  src/TeleopNode.cpp
)
#######################################################################################
target_compile_features(${L3XZ_TELEOP_TARGET} PRIVATE cxx_std_17)
ament_target_dependencies(${L3XZ_TELEOP_TARGET} rclcpp)
#######################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Teleop.msg"
)
#######################################################################################
rosidl_target_interfaces(${L3XZ_TELEOP_TARGET} ${PROJECT_NAME} "rosidl_typesupport_cpp")
#######################################################################################
install(TARGETS
  ${L3XZ_CTRL_TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
