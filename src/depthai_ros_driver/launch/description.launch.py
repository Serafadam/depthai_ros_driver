from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # TODO: Replace with robot state publisher in the future
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context) + "_"
    return [
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
                "{}link".format(tf_prefix),
                "{}color_frame".format(tf_prefix),
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
                "{}link".format(tf_prefix),
                "{}left_frame".format(tf_prefix),
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
                "{}link".format(tf_prefix),
                "{}right_frame".format(tf_prefix),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("tf_prefix", default_value="camera"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
