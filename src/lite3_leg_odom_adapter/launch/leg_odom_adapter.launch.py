from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lite3_leg_odom_adapter"),
        "config",
        "adapter.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="lite3_leg_odom_adapter",
                executable="leg_odom_adapter",
                name="leg_odom_adapter",
                output="screen",
                parameters=[config],
            )
        ]
    )
