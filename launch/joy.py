from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_teleop',
      executable='l3xz_teleop_node',
      name='teleop_node',
      namespace='l3xz',
      output='screen',
      parameters=[
          {'joy_dev_node': '/dev/input/js0'},
          {'topic_robot_stick': 'cmd_vel_robot'},
          {'topic_robot_pad': 'cmd_vel_head'},
          {'robot_linear_x_max': 3.0},
          {'robot_linear_y_max': 3.0},
          {'robot_angular_x_max': 1.0},
          {'robot_angular_z_max': 1.0},
          {'head_linear_x_max': 3.0},
          {'head_linear_y_max': 3.0},
      ]
    )
  ])
