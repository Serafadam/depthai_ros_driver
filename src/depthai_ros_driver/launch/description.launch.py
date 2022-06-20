from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: Replace with robot state publisher in the future
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_color",
                arguments=[
                    "0.0",
                    "0",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "camera_link",
                    "color_frame",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_left",
                arguments=[
                    "0.0",
                    "0",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "camera_link",
                    "left_frame",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_right",
                arguments=[
                    "0.0",
                    "0",
                    "0.0",
                    "0",
                    "0",
                    "0",
                    "camera_link",
                    "right_frame",
                ],
                output="screen",
            ),
        ]
    )
