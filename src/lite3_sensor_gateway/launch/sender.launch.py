from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

"""

启动底盘+RGBD话题转发节点
source /opt/ros/foxy/setup.bash
source /home/ysc/workspace/lite3_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 launch lite3_sensor_gateway sender.launch.py

"""
def generate_launch_description():
    default = os.path.join(get_package_share_directory('lite3_sensor_gateway'), 'config', 'sender.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='lite3_sensor_gateway', executable='sender',
             name='lite3_gateway_sender', output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
