from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_file = PathJoinSubstitution([
        FindPackageShare('urdf_pkg'), 'Lite3_urdf', 'urdf', 'Lite3.urdf'
        # FindPackageShare('urdf_pkg'), 'Lite3_urdf_new', 'urdf', 'Lite3.urdf'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('urdf_pkg'), 'rviz', 'lite3.rviz'
    ])
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', description_file
    ]), value_type=str)
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz to visualize the Lite3 model and TF tree.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
