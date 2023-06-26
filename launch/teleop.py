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
          {'joy_topic_deadline_ms': 100},
          {'joy_topic_liveliness_lease_duration': 1000},
          {'robot_topic': 'cmd_vel_robot'},
          {'robot_topic_publish_period_ms': 100},
          {'head_topic': 'cmd_vel_head'},
          {'head_topic_publish_period_ms': 50},
          {'robot_req_up_topic': 'cmd_robot/req_up'},
          {'robot_req_up_topic_publish_period_ms': 250},
          {'robot_req_down_topic': 'cmd_robot/req_down'},
          {'robot_req_down_topic_publish_period_ms': 250},
          {'pan_max_dps' : 90.0},
          {'tilt_max_dps' : 90.0},
      ]
    )
  ])
