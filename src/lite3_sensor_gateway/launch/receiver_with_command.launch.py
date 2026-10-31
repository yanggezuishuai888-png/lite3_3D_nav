import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('lite3_sensor_gateway'), 'launch'
    )
    receiver_launch = os.path.join(launch_dir, 'receiver.launch.py')
    command_sender_launch = os.path.join(launch_dir, 'command_sender.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('receiver_config'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(receiver_launch),
            launch_arguments={
                'config': LaunchConfiguration('receiver_config'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(command_sender_launch),
        ),
    ])
