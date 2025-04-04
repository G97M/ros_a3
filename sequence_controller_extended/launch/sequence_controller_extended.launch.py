# sequence_controller_extended.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([


        # Launch the RELbot Simulator
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            parameters=[
                {'use_twist_cmd': False}  # or True if you prefer twist-based control
            ],
            output='screen'
        ),

        # Launch the ObjectDetector node (assuming the package is "object_detector_pkg")
        # Make sure that node receives your laptop camera on /image
        Node(
            package='color_object_detector',
            executable='color_object_detector_node',
            name='color_object_detector',
            parameters=[
              #  {'lower_hsv' : [110, 50, 50] } , # set threshold
              #  {'upper_hsv' : [130, 255, 255]},
                {'min_area': 50 } 
            ],
            output='screen'
        ),

        # Launch our extended sequence controller
        Node(
            package='sequence_controller_extended',
            executable='extended_sequence_controller',
            name='extended_sequence_controller',
            # Start in "sequence" mode or "follow" mode
            # You can change this parameter at runtime
            parameters=[{'control_mode': 'follow'}],
            output='screen'
        )
    ])
