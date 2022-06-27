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
          {'x_max_stick': 3.0},
          {'y_max_stick': 3.0},
          {'angular_x_max_stick': 1.0},
          {'angular_z_max_stick': 1.0},
          {'x_pad': 3.0},
          {'y_pad': 3.0},
      ]
    )
  ])
