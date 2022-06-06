import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    parameters = [
        {
            "frame_id": "camera_link",
            "subscribe_depth": True,
            "approx_sync": True,
        }
    ]
    remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
    ]
    return LaunchDescription(
        [
            Node(
                package="rtabmap_ros",
                executable="rgbd_odometry",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
            Node(
                package="rtabmap_ros",
                executable="rtabmap",
                output="screen",
                parameters=parameters,
                remappings=remappings,
                arguments=["-d"],
            ),
            Node(
                package="rtabmap_ros",
                executable="rtabmapviz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
            ComposableNodeContainer(
                name="container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::RGBDCamera",
                        name="camera",
                        parameters=[{"fps": 60.0}],
                    ),
                ],
                output="screen",
            ),
        ]
    )
