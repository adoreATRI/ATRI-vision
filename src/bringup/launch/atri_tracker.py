from launch_ros.actions import Node

Node(
    package='atri_tracker',
    executable='tracker_node_exe',
    name='tracker_node',
)