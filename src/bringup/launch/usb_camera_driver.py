from launch_ros.actions import Node

Node(
    package='usb_camera_driver',
    executable='camera_capture_node_exe',
    name='camera_capture_node',
)
