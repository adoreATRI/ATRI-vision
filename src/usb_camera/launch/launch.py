from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('usb_camera'),
        'config',
        'config.yaml'
    )


    return LaunchDescription([
        Node(
            package='usb_camera',
            executable='usb_camera_node_exe',
            name='usb_camera_node',
            output='screen',
            parameters=[config]
        )
    ])