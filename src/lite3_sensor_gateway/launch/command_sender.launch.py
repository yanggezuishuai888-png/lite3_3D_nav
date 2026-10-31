import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(
        get_package_share_directory('lite3_sensor_gateway'), 'config', 'command_sender.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='lite3_sensor_gateway', executable='command_sender',
             name='lite3_command_sender', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
