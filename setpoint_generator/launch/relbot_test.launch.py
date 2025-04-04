from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            parameters=[{'use_twist_cmd': False}]
        ),
        Node(
            package='setpoint_generator',
            executable='setpoint_generator'
        )
    ])