import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParametersFromFile

CAMERA_CONFIG_PATH = "/home/ruming/ros2_ws/src/rtabmap_robot/config/camera.yaml"
RTABMAP_CONFIG_PATH = "/home/ruming/ros2_ws/src/rtabmap_robot/config/rtabmap.yaml"
RVIZ_CONFIG_PATH = "/home/ruming/ros2_ws/src/rtabmap_robot/rviz/rviz2.rviz"


def _load_ros_parameters(config_path, node_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    try:
        return config[node_name]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            f"{config_path} must contain {node_name}.ros__parameters"
        ) from error


def _launch_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def generate_launch_description():
    camera_parameters = _load_ros_parameters(CAMERA_CONFIG_PATH, "camera")
    rtabmap_parameters = _load_ros_parameters(
        RTABMAP_CONFIG_PATH, "/rtabmap/rtabmap"
    )

    rtabmap_launch_path = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py",
    )

    camera_info_republisher = Node(
        package="rtabmap_robot",
        executable="repeat_camera_info.py",
        name="camera_info_republisher",
        output="screen",
    )

    launch_arguments = {
        "rviz": LaunchConfiguration("rviz"),
        "rviz_cfg": LaunchConfiguration("rviz_cfg"),
        "rgb_topic": camera_parameters["rgb_topic"],
        "depth_topic": camera_parameters["depth_topic"],
        "camera_info_topic": camera_parameters["camera_info_topic"],
        "frame_id": camera_parameters["frame_id"],
        "approx_sync": _launch_value(camera_parameters["approx_sync"]),
        "approx_sync_max_interval": _launch_value(
            camera_parameters["approx_sync_max_interval"]
        ),
        "sync_queue_size": _launch_value(camera_parameters["queue_size"]),
        "subscribe_rgb": _launch_value(camera_parameters["subscribe_rgb"]),
        "depth": _launch_value(camera_parameters["subscribe_depth"]),
        "subscribe_rgbd": _launch_value(camera_parameters["subscribe_rgbd"]),
        "odom_frame_id": rtabmap_parameters["odom_frame_id"],
        "map_frame_id": rtabmap_parameters["map_frame_id"],
        "database_path": rtabmap_parameters["database_path"],
        "use_sim_time": "true",
        "odom_always_process_most_recent_frame": "false",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Launch RViz2 with the configured RViz file",
            ),
            DeclareLaunchArgument(
                "rviz_cfg",
                default_value=RVIZ_CONFIG_PATH,
                description="Absolute path to the RViz2 configuration",
            ),
            camera_info_republisher,
            GroupAction(
                actions=[
                    SetParametersFromFile(RTABMAP_CONFIG_PATH),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(rtabmap_launch_path),
                        launch_arguments=launch_arguments.items(),
                    ),
                ]
            )
        ]
    )


