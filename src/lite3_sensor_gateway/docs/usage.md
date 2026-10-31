# 第一阶段：只读传感器网关

目标：保留 Lite3 本机 Foxy 和厂家程序，PC 使用 Jazzy、独立 ROS 域，通过明确的 UDP 协议接收传感器。此阶段完成数据接收、检查和记录，不接入 Nav2 控制，不代表已经具备可靠建图定位条件。

## 数据链路

```text
Lite3（10.42.0.2，本机 ROS_DOMAIN_ID 保持原值，当前为 0）
├─ 运动主机 → 厂家 jetson2motion → 本机传感器话题 ─┐
├─ sensor_checker → 本机传感器存活 Bool ──────────┤
└─ RGBD USB → 厂家 realsense_ros2 → RGB/对齐深度/内参/内部 TF ┤
                                                 ↓ 只订阅
                               lite3_gateway_sender（C++）
                                  ├─ 底盘/状态/内参/相机内部 TF → UDP 45001
                                  └─ RGB/深度图    → UDP 45003
                                                 ↓ 10.42.0.x
PC（10.42.0.1，Jazzy，ROS_DOMAIN_ID=74）
└─ lite3_gateway_receiver（C++）
   ├─ 校验版本、源 IP、会话、序号、CRC；图像分片重组
   ├─ 回答狗端时钟探测；校时合格后发布融合用 ROS 2 消息
   ├─ 将相机内部静态变换发布到 /tf_static
   ├─ /lite3/gateway/time_valid：时钟是否合格
   └─ /lite3/gateway/status：连接、时钟偏差、原始时间、错误和丢帧诊断
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
| /tf_static 中五条相机内部变换 | PC /tf_static | 深度、彩色及对齐深度帧到相应 optical frame；只转发当前驱动实际发布的五条边 |
| /sensor_status/*_isalive | /lite3/sensor_status/*_isalive | std_msgs/Bool：imu、odom、odom2、joint、ultrasound、realsense、lidar |
| 参数 depth_scale_m | /lite3/camera/depth/scale_m | std_msgs/Float64：原始深度值乘此值得到米；默认 0.001 |
| 网关自身统计 | /lite3/gateway/status | diagnostic_msgs/DiagnosticArray：每 0.5 秒 |
| 时钟质量门控 | /lite3/gateway/time_valid | std_msgs/Bool：false 时融合输入不发布 |

默认转发建图基础数据：RGB、对齐深度及各自内参；原始深度可通过 `enable_depth: true` 打开。不转发厂家点云、雷达、关节原始数据和冗余 `/leg_odom`。相机必须由既有驱动运行，网关不会启停厂家服务。默认 RGB/对齐深度各最多 15 Hz，深度 zlib 无损压缩，RGB 原样发送。`image_max_mbps: 1000` 是软件上限，不是实际链路吞吐。640×480、15 Hz 的 RGB 加 16UC1 深度原始负载约 184 Mbit/s，分片和网络头使实际流量更高；第三路原始深度会继续增加负载。要观察 PC 诊断中的 `sequence_gaps`、`incomplete_expired`、实际 Hz 和 CPU，而非仅凭千兆协商速率判断稳定。

双机随机图像压力测试约收到每路 10 FPS，低于配置上限 15 Hz；该数据几乎不可压缩。真实相机启动后，PC C++ SensorDataQoS 订阅实测 RGB 12.1 Hz、对齐深度 12.7 Hz，完全相同时间戳的图像对约 10.4 对/秒。若建图需要更高配对频率，应检查狗端图像输入 Hz、sender 日志 `overwritten`、两端 CPU 和 PC 诊断；可在后续加入 JPEG 等压缩方案，或调低两路输出频率以匹配可持续吞吐。保留两路相近的输出频率更利于按时间戳配对。

传感器话题采用 best effort、volatile；PC 订阅也应允许 best effort。状态、内参、深度尺度与诊断采用 reliable、volatile。内参按 1 Hz 重发，匹配图像尺寸和 frame_id 时随对应图像再发布相同时间戳。缓存内参的重发不表示相机仍在采集；判断相机数据存活要看图像接收年龄。PC 保留相机源时间戳供 RGB-D 配对；底盘单调时间戳在狗端转换成系统时间后保留。

## 编译

狗端（已在此工作区完成编译验证）：

```bash
cd /home/ysc/workspace/lite3_ws
source /opt/ros/foxy/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select lite3_sensor_gateway --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash
```

PC 端只复制 `src/lite3_sensor_gateway` 源码目录到现有 `~/ros2_ws/src`，不复制 Jetson 的 build/install/log。协议变更后必须同步更新并编译 PC；不要覆盖 PC 自己的接收配置：

```bash
# 在 PC 执行；如果用户名或 SSH 地址不同，修改此命令
mkdir -p ~/ros2_ws/src
scp -r ysc@10.42.0.2:/home/ysc/workspace/lite3_ws/src/lite3_sensor_gateway ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
# 若缺依赖，安装：build-essential cmake python3-colcon-common-extensions
# zlib1g-dev ros-jazzy-rclcpp ros-jazzy-sensor-msgs ros-jazzy-nav-msgs
# ros-jazzy-geometry-msgs ros-jazzy-std-msgs ros-jazzy-diagnostic-msgs ros-jazzy-tf2-msgs
# ros-jazzy-tf2-ros ros-jazzy-launch-ros
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

默认地址为狗 10.42.0.2、PC 10.42.0.1；应先核对网卡。接收端绑定不存在的地址会明确报错退出。PC 如启用防火墙，应允许来自 10.42.0.2 的 UDP 45001/45003，以及 NTP UDP 123；狗端需允许来自 PC 45001 的探测应答返回临时源端口。无需配置端口转发。

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
ros2 topic echo /lite3/gateway/time_valid
ros2 topic echo /lite3/imu/data --qos-reliability best_effort --once
ros2 topic echo /lite3/leg_odom2 --qos-reliability best_effort --once
ros2 topic hz /lite3/camera/color/image_raw
ros2 topic hz /lite3/camera/aligned_depth_to_color/image_raw
python3 ~/ros2_ws/src/lite3_sensor_gateway/test/probe_rgbd.py --seconds 15
ros2 topic echo /tf_static tf2_msgs/msg/TFMessage --qos-reliability reliable --qos-durability transient_local
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
# 更准确的大图双路测量：BUILD_TESTING=ON 时运行
~/ros2_ws/build/lite3_sensor_gateway/integration_fixture --ros-args -p mode:=probe
```

验收顺序：先确认 `/lite3/gateway/time_valid` 为 true，再检查底盘数据、RGB/对齐深度及内参、两路图像尺寸与时间戳、数据频率、停发后的失效状态、重新启动 sender 后恢复。false 时没有融合话题消息是预期的保护行为。相机服务未运行时图像缺数据是预期行为。首次接收需要建立会话并取得至少三个合格探测样本，通常需要数秒。

发送端每十秒日志包含送出消息数、队列覆盖数和发送/编码错误数。送出只表示本机尝试发送，不能证明 PC 收到；以 PC 诊断为准。接收端诊断中的 sequence_gaps 统计已完成消息之间的序号缺口；incomplete_expired 统计重组超时。`old_packets` 还包含首次心跳建立会话前被忽略的数据，启动时非零是正常的。默认 2.5 秒未收到某 Bool 的更新即发布 false；不会让旧 true 永久保留。某 Bool 报 false 并不禁止原始数据发布，请同时观察对应数据年龄。

UDP 无重传、无可靠交付承诺；图像损坏或缺分片即整帧丢弃，避免向下游发布半帧。只保留每种待发送数据的最新样本；接收内存与待重组数量有界。版本和 CRC 用于兼容/损坏检测，源 IP 白名单不等同于加密认证。本实现用于当前有线局域网。

## 时间戳、校时与 TF

先使 PC 与狗的系统时钟持续同步。推荐在 PC 上配置 chrony 为 10.42.0.x 网段的时间服务器（`allow 10.42.0.2`），狗端配置 `server 10.42.0.1 iburst`，并关闭与 chrony 冲突的其他校时服务。配置应在各自系统服务中完成；网关不修改厂家服务。用两端的 `chronyc tracking`、`chronyc sources -v` 检查来源、状态与偏差。PC 的系统时间还应由可靠上游校准；只让狗与一个错误的 PC 时间一致不能保证绝对时间正确。

两端安装 chrony 后，在 PC 的 `/etc/chrony/chrony.conf` 保留上游时间源并增加 `allow 10.42.0.2`；在狗端同一文件增加 `server 10.42.0.1 iburst`，且取消其他上游 `server`/`pool`，使狗只向 PC 校时。先停用两端的 `systemd-timesyncd`，再启动 chrony。PC 防火墙允许狗访问 UDP 123。确认狗端 `chronyc sources -v` 选中的来源是 `10.42.0.1`，`chronyc tracking` 不再显示未同步；配置前先停相机及网关，待校时稳定后重启相机和网关。这里没有自动改动系统时钟服务。

网关另通过 10.42.0.x 的 UDP 45001 做持续的四时间戳探测，狗端按往返时间计算跨机时钟偏差上界。连续三个样本的 `abs(offset)+RTT/2 <= 5 ms`、最近合格样本不超过 2.5 秒，并且狗端单调→系统时钟映射新鲜，才开始发送 IMU、里程计及 RGB-D。PC 还要求最近 1.5 秒收到合格状态、心跳在线、本机时钟映射正常，才发布这些融合话题；`/lite3/gateway/time_valid=false` 时下游不得融合旧缓存。校时是持续过程，不是开机只执行一次。链路中断、偏差超限或时钟跳变会关闭门控；系统时钟发生跳变后必须重新启动相机和对应网关节点，以重建相机驱动的时间锚点。启动顺序建议先校时、再启动相机、PC receiver、狗端 sender。

厂家 IMU/里程计原戳是 `RCL_STEADY_TIME`。狗端在本机近同时采样单调与系统时钟，转换原戳，并把原始单调时间保留在协议 `raw_source_ns`；PC 输出消息的 header.stamp 是转换后的系统时间。相机 RGB、对齐深度与内参保留驱动源戳，接收端验证其与信封一致；图像戳偏离狗端当前系统时间超过 250 ms 时被丢弃。诊断包含跨机偏差、上界、RTT、映射不确定度和最近探测年龄。时钟门控只证明当前两机时间可比，不能证明相机硬件时间锚点或所有传感器的采样延迟已经准确；建图前仍需测 RGB-D 与 IMU/里程计的实际相对延迟、抖动。`timestamp_policy` 与 `camera_timestamp_policy` 必须为 `source`。

门控对 IMU、里程计、图像和内参生效；超声波、遥控状态和存活 Bool 仍作为诊断数据传输，不能在 `time_valid=false` 时把它们当作融合输入。由于现有 Bool 没有测量时间戳，本阶段不将它们纳入视觉惯性融合。
- 相机和 IMU 保留原始 frame_id；PC 接收端对厂家 `/leg_odom2` 缺失的 `header.frame_id`、`child_frame_id` 分别补 `odom`、`base_link`，非空字段不覆盖。狗端订阅相机驱动的 `/tf_static`，严格筛选五条内部变换；对齐深度模式下，`camera_color_optical_frame` 的实际父帧为 `camera_aligned_depth_to_color_frame`。PC 网关在 `/tf_static` 重新发布。静态变换不需要测量时刻配对，因此独立于 `time_valid` 门控；动态传感器仍须在校时合格后使用。PC 的 URDF 单独发布本体和安装链，不要重复发布相机内部边。用 `tf2_echo base_link camera_color_optical_frame` 检查完整静态链。本包不发布 `odom→base_link` 或 `map→odom`；RTAB-Map 的 TF 发布需单独配置且每条边只有一个发布者。
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

协议/编解码单测覆盖乱序、重复、CRC、分片长度、超时、内存淘汰、压缩、字段往返与畸形数据。时间单测覆盖单调/系统转换、时钟跳变、偏差超限与过期。集成测试在本机独立 ROS 域 81/82、仅 loopback 上运行虚拟传感器、网关和检查器，验证校时合格后底盘与 RGB-D 发布、无校时回包时阻止融合数据，不接触厂家服务。详见 test/run_integration.py；避免在其他业务已使用 81/82 域时运行。真实 10.42 链路已验证时钟门控保护；完成直连校时后的真实相机及长时间运行验收仍需执行。

## sender 启动出现 exit code -11

本设备曾因 PC 残留的域 0 Jazzy CLI daemon 发现流量，在 Foxy CycloneDDS 初始化期间段错误。PC 接收节点在域 74 并不代表其他 PC 进程也已离开域 0。详见 [定位与跨机测试记录](dds_crash_smoke.md)。

PC 检查 `pgrep -af ros2-daemon`；确认残留域 0 daemon 后，可用 `ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 daemon stop` 精确停止它。后续每个 ROS 终端先加载 PC 域 74 环境。不要因此关闭正常的域 74 UDP 接收节点，也不要未经检查替换 DDS 或修改厂家服务。
