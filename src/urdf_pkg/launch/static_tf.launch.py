from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution([
        FindPackageShare('urdf_pkg'), 'Lite3_urdf', 'urdf', 'Lite3.urdf'
    ])
    description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', model]), value_type=str)
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='lite3_robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': False}],
            output='screen',
        ),
    ])
