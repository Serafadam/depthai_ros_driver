import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    rviz_config = os.path.join(depthai_prefix, "config", "multicam.rviz")
    # put mx_ids here
    cams = {"cam_1": "", "cam_2": ""}
    tf_publishers = []
    nodes = []
    for cam_name, cam_mx_id in cams.items():
        tf_cam_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "description.launch.py")
            ),
            launch_arguments={"tf_prefix": cam_name}.items(),
        )
        base_to_cam = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"base_to_{cam_name}",
            arguments=[
                "0.0",
                "0",
                "0.0",
                "0",
                "0",
                "0",
                "base_camera_link",
                "{}_link".format(cam_name),
            ],
            output="screen",
        )
        tf_publishers.append(tf_cam_node)
        tf_publishers.append(base_to_cam)
        node = ComposableNode(
            package="depthai_ros_driver",
            plugin="depthai_ros_driver::RGBDCamera",
            name=cam_name,
            parameters=[
                {
                    "i_rgb_fps": 60.0,
                    "i_rgb_width": 1280,
                    "i_rgb_height": 720,
                    "i_camera_mxid": cam_mx_id,
                }
            ],
        )
        nodes.append(node)
    rviz = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )
    container = ComposableNodeContainer(
        name="container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    node_list = [rviz, container]
    node_list = node_list + tf_publishers
    return node_list


def generate_launch_description():
    declared_arguments = [DeclareLaunchArgument("use_rviz", default_value="False")]

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
