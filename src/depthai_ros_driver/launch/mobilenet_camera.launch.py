import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    depthai_prefix = get_package_share_directory('depthai_ros_driver')
    rviz_config = os.path.join(depthai_prefix, 'config', 'mobilenet.rviz')
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='False'),

            Node(
                package='depthai_ros_driver',
                executable='mobilenet_camera',
                output='screen',
            ),
            Node(
                package='depthai_ros_driver',
                executable='obj_pub.py',
                output='screen',
            ),
            Node(
                condition=IfCondition(LaunchConfiguration('use_rviz')),
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config]
            )
        ]
    )
