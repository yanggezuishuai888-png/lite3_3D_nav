# 验证记录

## RGB-D 扩展（2026-10-14）

- 狗端 Foxy Release 编译成功；`protocol_test`、`codec_test`、`gateway_integration` 全部通过。
- PC 端 Jazzy 编译成功，三项相同测试全部通过。两端协议和节点源码已同步到 PC `/home/ruming/ros2_ws/src/lite3_sensor_gateway`。
- 新的跨域 loopback 集成测试验证 RGB、对齐深度、各自内参及原始深度，12 路数据和状态失效检查通过；640×480 RGB 编解码逐字节往返通过。
- 默认配置改为 RGB/对齐深度各最多 15 Hz，关闭冗余原始深度；`image_max_mbps` 保留用户设置的 1000 Mbit/s。相机源时间戳在 PC 保留，供 RGB-D 配对。
- 这些测试不等于真实 10.42.0.x 链路的 640×480、15 Hz 持续吞吐验收；仍须观察 PC 的图像实际 Hz、`sequence_gaps` 和 `incomplete_expired`。
- 双机千兆链路（10.42.0.2 → 10.42.0.1）另用隔离 ROS 域 81 发布 640×480、约 14.3 Hz 的随机 RGB 与随机 16UC1 深度，共 215 对。随机数据接近无法压缩的上限情形，不代表真实场景压缩比。
- 第一轮逐包等待限速时，PC 收到 174 帧 RGB、56 帧对齐深度，发送端覆盖 155 帧。改为小批量限速并轮流取两路后，PC 收到 149 帧 RGB、146 帧对齐深度，发送端覆盖 1 帧；两轮接收端 `sequence_gaps=0`、`incomplete_expired=0`、`invalid_packets=0`。瓶颈从发送队列改善，但这次模拟源到 PC 仍只有约 10 FPS，尚未证明实际相机可稳定达到 15 FPS。
- 复测时狗端 `realsense_ros2` 为 inactive，故没有真实相机图像验收。PC 端底盘遥测通过真实链路，IMU 约 200 Hz，诊断显示 IMU/odom 序号缺口均为 0。

验证设备：当前 Lite3 Jetson，ROS 2 Foxy、GCC 9、C++17。本机未安装 Jazzy。

## 已完成

- 最终 Release 编译成功，`-Wall -Wextra -Wpedantic` 无编译警告。
- `colcon test`：3/3 通过（protocol_test、codec_test、gateway_integration），总测试时间约 11.65 秒。
- 协议测试：字段字节序、分片乱序/重复、CRC 损坏、截断/超长头、超时清理、组装数量上限、压缩往返与无效压缩输入。
- 编解码测试：IMU/里程计协方差、CameraInfo 的 D/K/R/P/ROI、带行填充及 bigendian 标记的深度图、非有限值及无效布局拒绝。
- 集成测试：本机虚拟数据在 ROS 域 81 → UDP loopback → ROS 域 82 转发，检查 IMU、里程计、超声波、遥控状态、203520 字节深度图、内参、尺度、存活 Bool，以及输入停止后 Bool 转 false。图像每个字节均核对。
- 包清单、YAML 参数和 Python launch/test 文件解析通过。

## 真实底盘只读探测

`test/probe_live.py` 从现有域 0 订阅，经 UDP loopback 在独立域 84 重发并接收；运行 12 秒（含发现和首个心跳等待）。

| 输出 | 收到消息数 |
|---|---:|
| /lite3/imu/data | 2177 |
| /lite3/leg_odom2 | 2176 |
| /lite3/ultrasound_distance | 2176 |
| /lite3/handle_state | 2175 |
| /lite3/sensor_status/imu_isalive | 25 |

最后一次网关诊断：invalid_packets=0，重组过期=0，IMU 和 odom 已接受序列之间的缺口均为 0。`old_packets` 包含第一次心跳到来前丢弃的数据，本轮为 820，不能将其直接理解为运行中的网络丢包。

实测厂家 IMU frame_id 为 `base_link`，里程计 frame_id 为空；本包保持原字段。源 IMU/里程计时间为单调时钟，接收端输出时间为本机接收时间；这验证了时间基准不一致的待办确实存在。

测试后已停止本次启动的 sender、receiver 和测试进程。厂家服务前后均为：transfer_ros2 active、realsense_ros2 inactive、voa_ros2 inactive；没有启停这些服务。

详细记录位于新工作区：`build/lite3_sensor_gateway/Testing/`、`integration-logs/` 和 `live-probe-logs/result.json`。

## 尚未验证 / 后续工作

- PC 上 Jazzy 的真实 10.42.0.x 双机图像运行（本次已完成编译及 loopback 集成测试）。
- 实际 10.42.0.2 → 10.42.0.1 网线链路的持续带宽、丢包、负载和断网恢复。
- 真实 RGBD 图像（本轮相机服务未运行，图像链路使用模拟 424×240 深度图验证）。
- URDF/外参、完整 TF 树、测量时间同步、SLAM/定位/导航算法接入。

人工复测真实底盘可在 Foxy 环境下执行：

```bash
cd /home/ysc/workspace/lite3_ws
source /opt/ros/foxy/setup.bash
python3 src/lite3_sensor_gateway/test/probe_live.py build/lite3_sensor_gateway
```

人工探测会短暂加入本机域 0 作为只读订阅者；请确保测试域 84 未被其他程序使用。它不会发布控制命令，也不会触碰厂家 UDP 端口。
