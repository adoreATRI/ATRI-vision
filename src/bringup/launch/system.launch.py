from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('bringup'),
        'config',
        'camera_params.yaml'    
    )

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_camera_driver',
                plugin='usb_camera_driver::CameraCaptureNode',
                name='camera_capture_node',
                parameters=[config]
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
