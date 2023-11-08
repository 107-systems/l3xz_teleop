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
* Install `gsl-lite`
```bash
git clone https://github.com/gsl-lite/gsl-lite && cd gsl-lite
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `Catch2`
```bash
git clone https://github.com/catchorg/Catch2 && cd Catch2
mkdir build && cd build
cmake .. && make -j8
sudo make install
```
* Install `fmt`
```bash
git clone https://github.com/fmtlib/fmt && cd fmt
mkdir build && cd build
cmake -DFMT_TEST=OFF ..
make -j8
sudo make install
```
* Install `mp-units`
```bash
git clone https://github.com/mpusz/mp-units && cd mp-units
mkdir build && cd build
cmake -DMP_UNITS_AS_SYSTEM_HEADERS=ON -DMP_UNITS_BUILD_LA=OFF ..
make -j8
sudo make install
```
* Build with `colcon`
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/l3xz_teleop
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select l3xz_teleop
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch l3xz_teleop teleop.py
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
|        Default name        |                                         Type                                          |
|:--------------------------:|:-------------------------------------------------------------------------------------:|
|   `/l3xz/cmd_vel_robot`    | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |
|    `/l3xz/cmd_vel_head`    | [`geometry_msgs/Twist`](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) |
|  `/l3xz/cmd_robot/req_up`  |       [`std_msgs/Bool`](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)       |
| `/l3xz/cmd_robot/req_down` |       [`std_msgs/Bool`](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)       |

##### Parameters
|                   Name                   |       Default        | Description                                                                                                  |
|:----------------------------------------:|:--------------------:|--------------------------------------------------------------------------------------------------------------|
|               `joy_topic`                |        `joy`         | Name of topic from where we are subscribing joystick messages.                                               |
|         `joy_topic_deadline_ms`          |         100          | Deadline in milliseconds within which a new joystick message is expected.                                    |
|  `joy_topic_liveliness_lease_duration`   |         1000         | The time within which the RMW node or publisher must show that it is alive.                                  | 
|              `robot_topic`               |   `cmd_vel_robot`    | Name of topic for controlling L3X-Z forward/angular speed (`linear.x`/, `angular.z`).                        |
|     `robot_topic_publish_period_ms`      |         100          | Publishing period for robot messages in milliseconds (ms).                                                   |
|               `head_topic`               |    `cmd_vel_head`    | Name of topic for controlling L3X-Z sensor head (`angular.y`/, `angular.z`) in degree per second (**dps**).  |
|      `head_topic_publish_period_ms`      |          50          | Publishing period for head messages in milliseconds (ms).                                                    |
|         `head_topic_deadline_ms`         |         100          | Deadline in milliseconds within which a new head message should be published.                                |
| `head_topic_liveliness_lease_duration`   |         1000         | The time within which the RMW node or publisher must show that it is alive.                                  |
|           `robot_req_up_topic`           |  `cmd_robot/req_up`  | Name of topic for requesting L3X-Z to stand up.                                                              |
|  `robot_req_up_topic_publish_period_ms`  |         250          | Publishing period for stand up request messages in milliseconds (ms).                                        |
|          `robot_req_down_topic`          | `cmd_robot/req_down` | Name of topic for requesting L3X-Z to sit down.                                                              |
| `robot_req_down_topic_publish_period_ms` |         250          | Publishing period for sit down request messages in milliseconds (ms).                                        |
|              `pan_max_dps`               |        10.0°         | Maximum target angular velocity for pan servo of the L3X-Z sensor head.                                      |
|              `tilt_max_dps`              |        10.0°         | Maximum target angular velocity for tilt servo of the L3X-Z sensor head.                                     |
