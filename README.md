<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_teleop`
===========================
[![Build Status](https://github.com/107-systems/l3xz_teleop/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_teleop/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_teleop/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_teleop/actions/workflows/spell-check.yml)

Teleoperation for L3X-Z via PS3 joystick and ROS topics.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
colcon_ws/src$ git clone https://github.com/107-systems/l3xz_teleop
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build --packages-select l3xz_teleop
```

#### How-to-run
```bash
colcon_ws$ source install/setup.bash
colcon_ws$ ros2 launch l3xz_teleop teleop.py
```
Check the content of the published message via:
```bash
ros2 topic list
/* ... */
ros2 topic echo /l3xz/cmd_vel_robot
```

#### Interface Documentation
##### Subscribed Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/joy` | [`sensor_msgs/Joy`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) |

##### Published Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/cmd_vel_robot` | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |
| `/l3xz/cmd_vel_head` | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `joy_topic` | `joy` | Name of topic from where we are subscribing joystick messages. |
| `robot_topic` | `cmd_vel_robot` | Name of topic for controlling L3X-Z forward/angular speed (`linear.x`/, `angular.z`). |
| `head_topic` | `cmd_vel_head` | Name of topic for controlling L3X-Z sensor head (`angular.y`/, `angular.z`). |
