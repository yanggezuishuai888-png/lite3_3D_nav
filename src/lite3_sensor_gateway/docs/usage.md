# 第一阶段：只读传感器网关

目标：保留 Lite3 本机 Foxy 和厂家程序，PC 使用 Jazzy、独立 ROS 域，通过明确的 UDP 协议接收传感器。此阶段完成数据接收、检查和记录，不接入 Nav2 控制，不代表已经具备可靠建图定位条件。

## 数据链路

```text
Lite3（10.42.0.2，本机 ROS_DOMAIN_ID 保持原值，当前为 0）
├─ 运动主机 → 厂家 jetson2motion → 本机传感器话题 ─┐
├─ sensor_checker → 本机传感器存活 Bool ──────────┤
└─ RGBD USB → 厂家 realsense_ros2 → RGB/对齐深度/内参 ┤
                                                 ↓ 只订阅
                               lite3_gateway_sender（C++）
                                  ├─ 底盘/状态/内参 → UDP 45001
                                  └─ RGB/深度图    → UDP 45003
                                                 ↓ 10.42.0.x
PC（10.42.0.1，Jazzy，ROS_DOMAIN_ID=74）
└─ lite3_gateway_receiver（C++）
   ├─ 校验版本、源 IP、会话、序号、CRC；图像分片重组
   ├─ 发布 /lite3/... 标准 ROS 2 消息
   └─ /lite3/gateway/status：连接、数据年龄、原始时间、错误和丢帧诊断
```

接收端绑定 PC 的 45001/45003；发送端绑定 Jetson 地址及系统分配的临时源端口。不会重复绑定厂家的 43893/43897/43899，不直接接收运动主机 UDP，也不直接打开相机。多个 ROS 订阅者是正常用法，与两个 UDP 程序抢同一个端口不同。

域隔离由各端 ROS_DOMAIN_ID 完成。更改 PC 后应关闭旧域上的 RViz、导航、Python 节点等，再用域 74 启动；仅修改一个新终端不会改变旧进程。狗的域与厂家服务保持不变。协议逐字段编码，不透传 Foxy DDS/CDR，不要求 Foxy 和 Jazzy 相互发现。

## 第一阶段的话题

| Jetson 输入 | PC 输出 | 类型 / 用途 |
|---|---|---|
| /imu/data | /lite3/imu/data | sensor_msgs/Imu：姿态、角速度、加速度、协方差 |
| /leg_odom2 | /lite3/leg_odom2 | nav_msgs/Odometry：原始底盘里程计、速度、协方差 |
| /us_publisher/ultrasound_distance | /lite3/ultrasound_distance | std_msgs/Float64：原样距离值，源单位待厂家确认 |
| /handle_state | /lite3/handle_state | geometry_msgs/Twist：仅观测遥控摇杆，非仲裁结果、非实际速度 |
| /camera/color/image_raw | /lite3/camera/color/image_raw | sensor_msgs/Image：原始 rgb8/bgr8 彩图 |
| /camera/color/camera_info | /lite3/camera/color/camera_info | sensor_msgs/CameraInfo：彩色内参 |
| /camera/aligned_depth_to_color/image_raw | /lite3/camera/aligned_depth_to_color/image_raw | sensor_msgs/Image：16UC1 对齐深度 |
| /camera/aligned_depth_to_color/camera_info | /lite3/camera/aligned_depth_to_color/camera_info | sensor_msgs/CameraInfo：对齐深度内参 |
| /camera/depth/image_rect_raw | /lite3/camera/depth/image_rect_raw | 可选：16UC1 原始深度，默认关闭 |
| /camera/depth/camera_info | /lite3/camera/depth/camera_info | 可选：原始深度内参，默认关闭 |
| /sensor_status/*_isalive | /lite3/sensor_status/*_isalive | std_msgs/Bool：imu、odom、odom2、joint、ultrasound、realsense、lidar |
| 参数 depth_scale_m | /lite3/camera/depth/scale_m | std_msgs/Float64：原始深度值乘此值得到米；默认 0.001 |
| 网关自身统计 | /lite3/gateway/status | diagnostic_msgs/DiagnosticArray：每 0.5 秒 |

默认转发建图基础数据：RGB、对齐深度及各自内参；原始深度可通过 `enable_depth: true` 打开。不转发厂家点云、雷达、关节原始数据和冗余 `/leg_odom`。相机必须由既有驱动运行，网关不会启停厂家服务。默认 RGB/对齐深度各最多 15 Hz，深度 zlib 无损压缩，RGB 原样发送。`image_max_mbps: 1000` 是软件上限，不是实际链路吞吐。640×480、15 Hz 的 RGB 加 16UC1 深度原始负载约 184 Mbit/s，分片和网络头使实际流量更高；第三路原始深度会继续增加负载。要观察 PC 诊断中的 `sequence_gaps`、`incomplete_expired`、实际 Hz 和 CPU，而非仅凭千兆协商速率判断稳定。

双机随机图像压力测试约收到每路 10 FPS，低于配置上限 15 Hz；该数据几乎不可压缩，而且狗端真实相机服务当时未运行。实际 RGB-D 频率需在相机启动后重新测量。若只有 10 Hz 左右，应检查狗端图像输入 Hz、sender 日志 `overwritten`、两端 CPU 和 PC 诊断；可将 `color_max_hz`/`depth_max_hz` 同时调到可持续速率，或在后续加入 JPEG 等压缩方案。保留两路相近的输出频率更利于按时间戳配对。

传感器话题采用 best effort、volatile；PC 订阅也应允许 best effort。状态、内参、深度尺度与诊断采用 reliable、volatile。内参按 1 Hz 重发，匹配图像尺寸和 frame_id 时随对应图像再发布相同时间戳。缓存内参的重发不表示相机仍在采集；判断相机数据存活要看图像接收年龄。PC 默认保留相机源时间戳，供 RGB 与对齐深度按 header.stamp 近似配对；底盘仍使用 PC 接收时间。

## 编译

狗端（已在此工作区完成编译验证）：

```bash
cd /home/ysc/workspace/lite3_ws
source /opt/ros/foxy/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select lite3_sensor_gateway --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

PC 端只复制 `src/lite3_sensor_gateway` 源码目录到现有 `~/ros2_ws/src`，不复制 Jetson 的 build/install/log。本次已经同步并编译；以后更新两端时需保持协议/节点源码一致：

```bash
# 在 PC 执行；如果用户名或 SSH 地址不同，修改此命令
mkdir -p ~/ros2_ws/src
scp -r ysc@10.42.0.2:/home/ysc/workspace/lite3_ws/src/lite3_sensor_gateway ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
# 若缺依赖，安装：build-essential cmake python3-colcon-common-extensions
# zlib1g-dev ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-nav-msgs
# ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-diagnostic-msgs ros-jazzy-launch-ros
colcon build --packages-select lite3_sensor_gateway --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

生产节点和协议均为 C++。launch 和测试编排使用 Python，仅负责启动，不处理生产消息。

## 手动启动

先启动 PC 接收端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/scripts/pc_ros_env.bash
ros2 launch lite3_sensor_gateway receiver.launch.py
```

狗端独立终端启动发送端，保持当前厂家域（本设备核对为 0）：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/workspace/lite3_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 launch lite3_sensor_gateway sender.launch.py
```

默认地址为狗 10.42.0.2、PC 10.42.0.1；应先核对网卡。接收端绑定不存在的地址会明确报错退出。PC 如启用防火墙，应允许来自 10.42.0.2 的 UDP 45001/45003。无需配置端口转发。

更改参数可复制 config 中 YAML 到新工作区自己的位置：

```bash
ros2 launch lite3_sensor_gateway sender.launch.py config:=/绝对路径/sender.yaml
ros2 launch lite3_sensor_gateway receiver.launch.py config:=/绝对路径/receiver.yaml
```

停止网关用 Ctrl+C，不影响厂家服务。这里没有配置开机启动，先完成 PC 实机联调再部署。

## PC 验收

所有命令先在同一终端设置 ROS_DOMAIN_ID=74 并 source PC 的工作区。

```bash
ros2 topic list
ros2 topic echo /lite3/gateway/status
ros2 topic echo /lite3/imu/data --qos-reliability best_effort --once
ros2 topic echo /lite3/leg_odom2 --qos-reliability best_effort --once
ros2 topic hz /lite3/camera/color/image_raw
ros2 topic hz /lite3/camera/aligned_depth_to_color/image_raw
```

验收顺序：底盘数据、RGB/对齐深度及内参、两路图像尺寸与时间戳、数据频率、停发后的失效状态、重新启动 sender 后恢复。相机服务未运行时图像缺数据是预期行为。首次接收等到一秒一次的心跳建立会话，通常需要约 1–2 秒。

发送端每十秒日志包含送出消息数、队列覆盖数和发送/编码错误数。送出只表示本机尝试发送，不能证明 PC 收到；以 PC 诊断为准。接收端诊断中的 sequence_gaps 统计已完成消息之间的序号缺口；incomplete_expired 统计重组超时。`old_packets` 还包含首次心跳建立会话前被忽略的数据，启动时非零是正常的。默认 2.5 秒未收到某 Bool 的更新即发布 false；不会让旧 true 永久保留。某 Bool 报 false 并不禁止原始数据发布，请同时观察对应数据年龄。

UDP 无重传、无可靠交付承诺；图像损坏或缺分片即整帧丢弃，避免向下游发布半帧。只保留每种待发送数据的最新样本；接收内存与待重组数量有界。版本和 CRC 用于兼容/损坏检测，源 IP 白名单不等同于加密认证。本实现用于当前有线局域网。

## 时间戳与 TF：明确待办

- 厂家 IMU/里程计源码使用 `RCL_STEADY_TIME`，不能与 PC Unix 时间直接相减。默认 `timestamp_policy: receive` 为 IMU/里程计写入 PC 接收时间；`camera_timestamp_policy: source` 保留相机源时间以配对 RGB-D。两种时间尚未统一，不应直接拿去验证高精度融合。
- UDP 信封保留源时间及声明时钟域；诊断保留最近一条 `source_stamp_ns`、`source_clock_domain`、`sender_system_ns`。0=未知，1=单调时钟，2=系统时钟。原消息全部字段除默认输出时间戳外按原样保留。相机时钟默认未知。
- `timestamp_policy: source` 仅用于已核对相同时间基准的后续阶段。它会直接保留源 stamp，不做换算；当前厂家单调时钟下不应切换用于导航。需制定 Jetson↔PC 校时、单调时钟映射、相机硬件时钟映射和重启后重建策略；评估 IMU/深度/里程计相对延迟、漂移和抖动。单靠 NTP 不会自动修正消息内的单调时间戳。
- 保留原始 frame_id/child_frame_id，包括空值。拿到 URDF 和外参后补 `base_link→imu_link/camera_link/...` 静态变换，核对相机 optical frame，指定 `odom→base_link` 的唯一发布者和 SLAM/定位 `map→odom` 的唯一发布者。本包当前不发 `/tf` 或 `/tf_static`，不编造 frame，也不把 odometry 自动转 TF。
- 向厂家确认 IMU/里程计坐标轴和单位、超声波量纲/无效值、odom 定义与复位行为、协方差可用性、传感器安装外参。URDF 到位后再验证整条坐标树；拿到这些之前可以验通数据，但不能认定导航链路完成。
- 速度仲裁、遥控优先、网络断连停车、cmd_vel 回传均属于后续控制阶段。本包不创建任何命令发布者，不收控制 UDP。

## 测试

```bash
cd /home/ysc/workspace/lite3_ws
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
colcon test --packages-select lite3_sensor_gateway --event-handlers console_direct+
colcon test-result --verbose
```

协议/编解码单测覆盖乱序、重复、CRC、分片长度、超时、内存淘汰、压缩、字段往返与畸形数据。集成测试在本机独立 ROS 域 81/82、仅 loopback 上运行虚拟传感器、网关和检查器，验证原始深度、RGB、对齐深度、内参、底盘消息和状态失效，不接触厂家服务。详见 test/run_integration.py；避免在其他业务已使用 81/82 域时运行。PC Jazzy 实机编译、10.42 链路丢包/带宽、真实相机及长时间运行验收仍需执行。

## sender 启动出现 exit code -11

本设备曾因 PC 残留的域 0 Jazzy CLI daemon 发现流量，在 Foxy CycloneDDS 初始化期间段错误。PC 接收节点在域 74 并不代表其他 PC 进程也已离开域 0。详见 [定位与跨机测试记录](dds_crash_smoke.md)。

PC 检查 `pgrep -af ros2-daemon`；确认残留域 0 daemon 后，可用 `ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 daemon stop` 精确停止它。后续每个 ROS 终端先加载 PC 域 74 环境。不要因此关闭正常的域 74 UDP 接收节点，也不要未经检查替换 DDS 或修改厂家服务。
