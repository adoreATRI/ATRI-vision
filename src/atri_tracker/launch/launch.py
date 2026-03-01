from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('atri_tracker'),
        'config',
        'config.yaml'
    )


    return LaunchDescription([
        Node(
            package='atri_tracker',
            executable='tracker_node_exe',
            name='tracker_node',
            output='screen',
            parameters=[config]
        )
    ])