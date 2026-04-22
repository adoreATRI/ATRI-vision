import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("atri_serial_driver"), "config", "config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="atri_serial_driver",
                executable="atri_serial_driver_exe",
                name="atri_serial_driver_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
