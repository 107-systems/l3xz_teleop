from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_teleop',
      executable='l3xz_teleop_node',
      name='teleop',
      output='screen',
      parameters=[
          {'joy_dev_node': '/dev/input/js0'},
          {'topic_robot_velocity': 'cmd_vel'},
      ]
    )
  ])
