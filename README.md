<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_teleop`
===========================
[![Build Status](https://github.com/107-systems/l3xz_teleop/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_teleop/actions/workflows/ros2.yml)

Teleoperation for L3X-Z via PS3 joystick and ROS topics.

### How-to-build
```bash
# Clone this repository.
git clone https://github.com/107-systems/l3xz_teleop
# Invoke 'colcon build' from repository root.
source /opt/ros/galactic/setup.bash
colcon build
```

### How-to-run
```bash
source install/setup.bash
ros2 launch l3xz_teleop joy.py
```
Check the content of the published message via:
```bash
ros2 topic list
/* ... */
ros2 topic echo /l3xz/cmd_vel_robot
```

### PS3 Control Description
| Axis | Button | Description | Mapping |
|:-:|:-:|-|:-:|
| Left Stick/Vertical | | Linear velocity of L3X-Z stepping forward/backward. | `linear.x` |
| Left Stick/Horizontal | | Linear velocity of L3X-Z stepping left/right (sideways). | `linear.y` |
| Right Stick/Vertical | | Angular velocity of L3X-Z around x-axis. | `angular.x` |
| Right Stick/Horizontal | | Angular velocity of L3X-Z around z-axis. | `angular.z` |


### Interface Documentation
#### Published Topics
| Default name | Type |
|:-:|:-:|
| `/cmd_vel_robot` | [`geometry_msgs/Twist`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) |
| `/cmd_vel_head` | [`geometry_msgs/Twist`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) |

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `joy_dev_node` | `/dev/input/js0` | Name of input device node under which joystick is registed in Linux. |
| `topic_robot_stick` | `cmd_vel_robot` | Name of topic for controlling L3X-Z foward/sideways/angular speed (linear.x/y, angular.z). |
| `topic_robot_pad` | `cmd_vel_head` | Name of topic for controlling L3X-Z sensor head. |
