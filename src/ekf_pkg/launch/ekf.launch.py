import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("ekf_pkg")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock during rosbag playback",
        ),
        Node(
            package="ekf_pkg",
            executable="ekf_input_adapter",
            name="ekf_input_adapter",
            output="screen",
            parameters=[
                os.path.join(package_share, "config", "input_adapter.yaml"),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                os.path.join(package_share, "config", "ekf.yaml"),
                {"use_sim_time": use_sim_time},
            ],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),
    ])
