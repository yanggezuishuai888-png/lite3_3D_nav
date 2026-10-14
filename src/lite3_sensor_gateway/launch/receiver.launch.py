from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('lite3_sensor_gateway'), 'config', 'receiver.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='lite3_sensor_gateway', executable='receiver',
             name='lite3_gateway_receiver', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
