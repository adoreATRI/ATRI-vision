from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    camera_config = os.path.join(
      get_package_share_directory('usb_camera'),
        'config',
        'config.yaml'    
    )

    tracker_config = os.path.join(
        get_package_share_directory('atri_tracker'),
        'config',
        'config.yaml'
    )

    serial_config = os.path.join(
        get_package_share_directory('atri_serial_driver'),
        'config',
        'config.yaml'
    )

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_camera',
                plugin='usb_camera::USBCameraNode',
                name='usb_camera_node',
                parameters=[camera_config]
            ),

            ComposableNode(
                package='atri_detector',
                plugin='atri_detector::DetectorNode',
                name='detector_node',
            ),

            ComposableNode(
                package='atri_tracker',
                plugin='atri_tracker::TrackerNode',
                name='tracker_node',
                parameters=[tracker_config]
            ),

            ComposableNode(
                package='atri_serial_driver',
                plugin='atri_serial_driver::ATRISerialDriver',
                name='atri_serial_driver',
                parameters=[serial_config]
            ),
        ]
    )

    return LaunchDescription([
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base',
                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_camera',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
            ),
                       
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_to_optical',
                arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_optical_frame']
            ),
           
            container])
