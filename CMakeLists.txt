#######################################################################################
cmake_minimum_required(VERSION 3.0.2)
#######################################################################################
project(l3xz_teleop)
#######################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf2
)
#######################################################################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES l3xz_teleop
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)
#######################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#######################################################################################
add_executable(${PROJECT_NAME}_node
  src/Joystick.cpp
  src/node.cpp
)
#######################################################################################
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_node PRIVATE -std=c++17 -Wall -Werror -pedantic)
#######################################################################################
