from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="relbot_simulator",
                executable="relbot_simulator",
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            Node(
                package="image_tools",
                executable="cam2image",
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
                   Node(
            package='color_object_detector',
            executable='color_object_detector_node',
            name='color_object_detector',
            parameters=[
                #{'lower_hsv' : [110, 50, 50] } , # set threshold
                #{'upper_hsv' : [130, 255, 255]},
                {'min_area': 50 } 
            ],
            output='screen'
        ),
            Node(
                package="sequence_controller",
                executable="sequence_controller",
                remappings=[
                    ("camera_position", "/output/camera_position"),
                    ("left_motor_setpoint_vel", "/input/right_motor/setpoint_vel"),
                    ("right_motor_setpoint_vel", "/input/right_motor/setpoint_vel"),
                ],
            ),
        ]
    )
