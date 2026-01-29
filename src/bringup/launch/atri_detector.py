from launch_ros.actions import Node

Node(
    package='atri_detector',
    executable='detector_node_exe',
    name='detector_node',
)
