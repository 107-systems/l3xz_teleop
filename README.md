<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_teleop`
[![Build Status](https://github.com/107-systems/l3xz_teleop/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_teleop/actions/workflows/ros2.yml)
===========================
Teleoperation for L3X-Z via PS3 joystick and ROS topics.

### How-to-build
```bash
# Clone this repository into catkin_ws/src.
git clone https://github.com/107-systems/l3xz_teleop
# Invoke catkin_make from the catkin workspace root.
source /opt/ros/noetic/setup.bash
catkin_make
```

### How-to-run
```bash
source devel/setup.bash
roslaunch l3xz_teleop joy.launch
```

### PS3 Control Description
Left joystick:
* Forward/Backward -> Velocity of L3X-Z stepping forward/backward.
* Left/Right -> Velocity of L3X-Z stepping left/right (sideways).

Left and right rear button/axis number #2 serve for controlling the angular velocity around the Z-axis. Fully pressing the left rear button #2 leads to L3X-Z turning left with maximum possible velocity, fully pressing the right rear button #2 leads to L3X-Z turning right with maximum possible velocity. Pressing both simultaneously they cancel each other out (think of pulling a horse left or right using the rein you hold with both hands).

Right joystick:
* Forward/Backward -> Pose of sensor head: up/down -90° to +90°.
* Left/Right -> Pose of sensor head: left/right -90° to +90°.

### Interface Documentation
#### Published Topics
| Default name | Type |
|:-:|:-:|
| `/cmd_vel` | [`geometry_msgs/Twist`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) |

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `joy_dev_node` | `/dev/input/js0` | Name of input device node under which joystick is registed in Linux. |
| `topic_robot_velocity` | `cmd_vel` | Name of topic for controlling L3X-Z foward/sideways/angular speed (linear.x/y, angular.z) as well as the sensor head. |
