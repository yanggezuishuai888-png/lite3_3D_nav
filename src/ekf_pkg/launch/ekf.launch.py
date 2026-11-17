import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ekf_pkg")

    return LaunchDescription([
        Node(
            package="ekf_pkg",
            executable="ekf_input_adapter",
            name="ekf_input_adapter",
            output="screen",
            parameters=[os.path.join(package_share, "config", "input_adapter.yaml")],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(package_share, "config", "ekf.yaml")],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),
    ])
