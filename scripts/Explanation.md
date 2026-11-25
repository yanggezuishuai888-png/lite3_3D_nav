1. `pc_ros_env.bash`：加载 PC 端 ROS 2 Jazzy 工作空间，并设置 ROS Domain 和 CycloneDDS 环境。
2. `run_receiver.sh`：启动 UDP Receiver 和速度命令 Sender；仅在网关时间合格时转发 `/cmd_vel`。
3. `stop_receiver.sh`：安全停止 `run_receiver.sh` 启动的整个网关进程组。
4. `lite3_chrony_network_refresh.sh`：网络恢复后立即刷新 Chrony 时间源，并在时间合格后启动 PC PTP Master。
5. `NetworkManager/dispatcher.d/90-lite3-chrony-refresh`：监听 NetworkManager 联网事件并触发 Chrony 刷新服务。
6. `systemd/lite3-chrony-network-refresh.service`：以 systemd 一次性服务运行 Chrony 网络恢复逻辑。
7. `systemd/lite3-ptp4l-pc.service.d/20-recovery.conf`：设置 PC `ptp4l` 的时间资格检查、自动重试及 `phc2sys` 联动恢复。
8. `systemd/lite3-phc2sys-pc.service.d/20-recovery.conf`：确保 PC `phc2sys` 异常退出后自动重启。
