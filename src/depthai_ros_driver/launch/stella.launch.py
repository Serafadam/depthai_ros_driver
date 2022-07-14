import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    stella_params = os.path.join(depthai_prefix, "config", "stella_example.yaml")
    orb_vocab = os.path.join(depthai_prefix, "models", "orb_vocab.fbow")
    remapings = {("camera/image_raw", "camera/color/image_raw")}
    return LaunchDescription(
        [
            Node(
                package="stella_vslam_ros",
                executable="run_slam",
                output="screen",
                remappings=remapings,
                arguments=[
                    "-v",
                    orb_vocab,
                    "-c",
                    stella_params,
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(depthai_prefix, "launch", "description.launch.py")
                )
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
                        parameters=[
                            {
                                "i_rgb_fps": 60.0,
                                "i_rgb_width": 1280,
                                "i_rgb_height": 720,
                            }
                        ],
                    ),
                ],
                output="screen",
            ),
        ]
    )
