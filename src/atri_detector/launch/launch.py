from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="atri_detector",
                executable="detector_node_exe",
                name="detector_node",
                output="screen",
            )
        ]
    )
