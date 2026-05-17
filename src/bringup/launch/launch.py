import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile


def generate_launch_description():
    pkg_camera = get_package_share_directory("usb_camera")
    pkg_bringup = get_package_share_directory("bringup")
    pkg_tracker = get_package_share_directory("atri_tracker")
    pkg_serial_driver = get_package_share_directory("atri_serial_driver")

    # 声明参数
    start_rviz = LaunchConfiguration("start_rviz")

    camera_params_path = os.path.join(pkg_camera, "config", "usb_camera_config.yaml")
    camera_params = ParameterFile(camera_params_path, allow_substs=True)

    tracker_config = os.path.join(pkg_tracker, "config", "config.yaml")
    serial_config = os.path.join(pkg_serial_driver, "config", "config.yaml")
    rviz_config = os.path.join(pkg_bringup, "rviz", "debug.rviz")

    declare_start_rviz_cmd = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz with the debug display config",
    )
    declare_camera_path_cmd = DeclareLaunchArgument(
        "camera_path",
        default_value="/dev/v4l/by-id/usb-RYS_USB_Camera_200901010001-video-index0",
        description="V4L2 camera device path",
    )

    with open(serial_config, "r") as f:
        config_params = yaml.safe_load(f)
    serial_params = config_params["atri_serial_driver"]["ros__parameters"]

    container = ComposableNodeContainer(
        name="vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_camera",
                plugin="usb_camera::USBCameraNode",
                name="usb_camera_node",
                parameters=[camera_params],
            ),
            # debug
            # ComposableNode(
            #     package='camera_simulator',
            #     plugin='camera_simulator::CameraSimulatorNode',
            #     name='camera_simulator_node'
            # ),
            ComposableNode(
                package="atri_detector",
                plugin="atri_detector::DetectorNode",
                name="detector_node",
            ),
            ComposableNode(
                package="atri_tracker",
                plugin="atri_tracker::TrackerNode",
                name="tracker_node",
                parameters=[tracker_config],
            ),
            ComposableNode(
                package="atri_serial_driver",
                plugin="atri_serial_driver::ATRISerialDriver",
                name="atri_serial_driver",
                parameters=[serial_config],
            ),
        ],
    )

    odom_to_base_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    # gimbal_to_camera_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='gimbal_to_camera',
    #     arguments=[
    #         '0',
    #         str(serial_params['tf_offset']['camera_link']['y']),
    #         '0',
    #         '0',
    #         '0',
    #         '0',
    #         'gimbal_pitch',
    #         'camera_link',
    #     ],
    # )

    # Debug
    gimbal_to_camera_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gimbal_to_camera",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"],
    )

    camera_to_optical_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_optical",
        arguments=[
            "0",
            "0",
            "0",
            "-1.5708",
            "0",
            "-1.5708",
            "camera_link",
            "camera_optical_frame",
        ],
    )

    camera_to_laser_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_laser",
        arguments=[
            "0",
            str(serial_params["tf_offset"]["laser_link"]["y"]),
            "0",
            "0",
            "0",
            "0",
            "camera_link",
            "laser_link",
        ],
    )

    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()

    ld.add_action(declare_start_rviz_cmd)
    ld.add_action(declare_camera_path_cmd)

    ld.add_action(odom_to_base_node)
    ld.add_action(gimbal_to_camera_node)
    ld.add_action(camera_to_optical_node)
    ld.add_action(camera_to_laser_node)
    ld.add_action(rviz_node)
    ld.add_action(container)

    return ld
