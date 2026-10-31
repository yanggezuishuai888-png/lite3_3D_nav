1. `pc_ros_env.bash`：刷新 PC 端 ROS 2 环境。
2. `run_receiver.sh`：运行 `source /home/ruming/ros2_ws/scripts/run_receiver.sh`，同时启动 UDP receiver 和速度命令 sender。
3. 前台按 Ctrl+C，或在另一终端运行 `bash /home/ruming/ros2_ws/scripts/stop_receiver.sh`，停止整个进程组。

注意：command sender 会在 `/lite3/gateway/time_valid` 为 true 时转发 `/cmd_vel` 到狗端。启动前确保没有非预期的 `/cmd_vel` 发布者。
