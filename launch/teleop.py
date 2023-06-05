from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_teleop',
      executable='l3xz_teleop_node',
      name='l3xz_teleop',
      namespace='l3xz',
      output='screen',
      emulate_tty=True,
      parameters=[
          {'joy_topic': 'joy'},
          {'robot_topic': 'cmd_vel_robot'},
          {'head_topic': 'cmd_vel_head'},
          {'pan_max_dps' : 90.0},
          {'tilt_max_dps' : 90.0},
      ]
    )
  ])
