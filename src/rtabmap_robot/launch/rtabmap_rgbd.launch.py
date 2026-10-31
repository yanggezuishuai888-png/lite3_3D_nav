
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParametersFromFile


############################
# 配置文件路径
############################
CAMERA_CONFIG_PATH = (
    "/home/ruming/ros2_ws/src/rtabmap_robot/config/camera.yaml"
)

RTABMAP_CONFIG_PATH = (
    "/home/ruming/ros2_ws/src/rtabmap_robot/config/rtabmap.yaml"
)

RVIZ_CONFIG_PATH = (
    "/home/ruming/ros2_ws/src/rtabmap_robot/rviz/rviz2.rviz"
)


############################
# 读取ROS参数
############################
def _load_ros_parameters(config_path, node_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    try:
        return config[node_name]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            f"{config_path} must contain "
            f"{node_name}.ros__parameters"
        ) from error


############################
# 参数转字符串
############################
def _launch_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


############################
# 主启动函数
############################
def generate_launch_description():

    ############################
    # 1. 读取配置文件
    ############################
    camera_parameters = _load_ros_parameters(
        CAMERA_CONFIG_PATH,
        "camera",
    )

    rtabmap_parameters = _load_ros_parameters(
        RTABMAP_CONFIG_PATH,
        "/rtabmap/rtabmap",
    )

    odometry_parameters = _load_ros_parameters(
        RTABMAP_CONFIG_PATH,
        "/rtabmap/rgbd_odometry",
    )

    ############################
    # 2. RTAB-Map官方Launch路径
    ############################
    rtabmap_launch_path = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py",
    )

    ############################
    # 3. urdf_pkg静态TF Launch路径
    ############################
    static_tf_launch_path = os.path.join(
        get_package_share_directory("urdf_pkg"),
        "launch",
        "static_tf.launch.py",
    )

    ############################
    # 4. RTAB-Map启动参数
    ############################
    launch_arguments = {

        # 可视化
        "rviz": LaunchConfiguration("rviz"),
        "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
        "rviz_cfg": LaunchConfiguration("rviz_cfg"),

        # RGB-D话题
        "rgb_topic": camera_parameters["rgb_topic"],
        "depth_topic": camera_parameters["depth_topic"],
        "camera_info_topic": camera_parameters["camera_info_topic"],
        "imu_topic": camera_parameters["imu_topic"],

        # 机器人坐标系
        "frame_id": camera_parameters["frame_id"],
        "odom_frame_id": rtabmap_parameters["odom_frame_id"],
        "vo_frame_id": odometry_parameters["odom_frame_id"],
        "map_frame_id": rtabmap_parameters["map_frame_id"],

        # TF发布
        "publish_tf_map": _launch_value(
            rtabmap_parameters.get("publish_tf", True)
        ),
        "publish_tf_odom": _launch_value(
            odometry_parameters.get("publish_tf", True)
        ),

        # 时间同步
        "approx_sync": _launch_value(
            camera_parameters["approx_sync"]
        ),
        "approx_sync_max_interval": _launch_value(
            camera_parameters["approx_sync_max_interval"]
        ),
        "sync_queue_size": _launch_value(
            camera_parameters["queue_size"]
        ),

        # Receiver publishes images and IMU with SensorDataQoS (Best Effort),
        # while CameraInfo is published Reliable.
        "qos_image": "2",
        "qos_odom": "2",
        "qos_camera_info": "1",
        "qos_imu": "2",

        # RGB-D输入模式
        "subscribe_rgb": _launch_value(
            camera_parameters["subscribe_rgb"]
        ),
        "depth": _launch_value(
            camera_parameters["subscribe_depth"]
        ),
        "subscribe_rgbd": _launch_value(
            camera_parameters["subscribe_rgbd"]
        ),

        # 启用RGB-D视觉里程计
        "visual_odometry": "true",

        # RTAB-Map数据库
        "database_path": LaunchConfiguration("database_path"),

        # Live hardware uses system time; rosbag playback can override this.
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "wait_imu_to_init": "true",

        # Real-time operation: prefer the newest frame when processing falls behind.
        "odom_always_process_most_recent_frame": "true",
    }

    ############################
    # 5. 创建LaunchDescription
    ############################
    return LaunchDescription([

        ############################
        # 启动参数声明
        ############################

        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="是否启动RViz2",
        ),

        DeclareLaunchArgument(
            "rtabmap_viz",
            default_value="true",
            description="是否启动RTAB-Map图形界面",
        ),

        DeclareLaunchArgument(
            "rviz_cfg",
            default_value=RVIZ_CONFIG_PATH,
            description="RViz配置文件路径",
        ),

        DeclareLaunchArgument(
            "database_path",
            default_value=rtabmap_parameters["database_path"],
            description="RTAB-Map database path (defaults to rtabmap.yaml)",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock for rosbag playback; false for live hardware",
        ),

        DeclareLaunchArgument(
            "use_static_tf",
            default_value="true",
            description="是否启动urdf_pkg的静态TF",
        ),

        ############################
        # 6. 启动urdf_pkg静态TF
        ############################
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                static_tf_launch_path
            ),
            condition=IfCondition(
                LaunchConfiguration("use_static_tf")
            ),
        ),

        ############################
        # 7. 启动RTAB-Map
        ############################
        GroupAction(
            actions=[

                # 加载RTAB-Map YAML参数
                SetParametersFromFile(
                    RTABMAP_CONFIG_PATH
                ),

                # 启动官方RTAB-Map Launch
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        rtabmap_launch_path
                    ),
                    launch_arguments=launch_arguments.items(),
                ),
            ]
        ),
    ])
