# Lite3 PC 网关验收记录

日期：2026-10-13

## 目标环境

- PC：`ruming-X15-AT-23`，Ubuntu 24.04.5，x86_64
- ROS：Jazzy，PC ROS domain `74`
- PC 有线地址：`10.42.0.1`（`enp0s31f6`）
- 狗端地址：`10.42.0.2`，Foxy，ROS domain `0`
- UDP：`45001`（状态/底盘/内参）、`45003`（深度图）
- 源码：`src/lite3_sensor_gateway`，来自狗端工作区的已复制源码

## PC 侧入口

- 配置：`config/receiver.pc.yaml`
- ROS 环境：`scripts/pc_ros_env.bash`
- receiver：`scripts/run_receiver.sh`
- 启动：`source scripts/run_receiver.sh` 或 `bash scripts/run_receiver.sh`
- 停止：在启动终端按 `Ctrl-C`；也可在另一终端运行 `bash scripts/stop_receiver.sh`

专用环境会清除继承的 `ROS_DISCOVERY_SERVER`、`CYCLONEDDS_URI`、`ROS_LOCALHOST_ONLY` 和
`RMW_IMPLEMENTATION`，然后加载 Jazzy、本工作区和 domain 74。它不修改全局 shell 配置。

## 当前结果

- [x] Jazzy Release 编译通过
- [x] `protocol_test` 通过
- [x] `codec_test` 通过
- [x] `gateway_integration` 通过（loopback，ROS domains 81/82）
- [x] 真实 `10.42.0.2 -> 10.42.0.1` UDP 底盘数据样本
- [ ] 真实深度图、CameraInfo 和深度尺度
- [ ] sender 停止/恢复后的真实链路失效恢复

## 2026-10-13 实机冒烟

- 狗端已有 sender 实例，未重复启动；PC receiver 本次由 `scripts/run_receiver.sh` 启动。
- receiver 实际绑定：`10.42.0.1:45001`、`10.42.0.1:45003`。
- `/lite3/imu/data`、`/lite3/leg_odom2`、`/lite3/ultrasound_distance` 均收到真实样本，
  `ros2 topic hz` 有界采样约为 200 Hz。
- `/lite3/imu/data` 和 `/lite3/leg_odom2` 各有真实样本；IMU frame 为 `base_link`，里程计
  frame_id/child_frame_id 当前为空，网关保持原值。
- `/lite3/gateway/status`：UDP online，`invalid_packets=0`，`incomplete_expired=0`，
  IMU/odom `sequence_gaps=0`；启动早期 `old_packets=399`，按协议说明不作为运行中丢包率。
- 深度和 CameraInfo 5 秒采样无消息；狗端只读状态为 `realsense_ros2 inactive`，故标记为
  相机未运行，不判为 UDP 故障。
- 当前 PC receiver 已停止；狗端用户已有 sender 保持运行，未执行停止/恢复测试。

`ros2 topic echo` 的 QoS 参数只影响 receiver 发布后的 ROS 订阅，不影响狗端到 PC 的 UDP
接收。当前 Jazzy 的 `ros2 topic hz` 不支持 `--qos-reliability` 选项，频率测试使用其实际
支持的语法完成。

集成测试使用模拟传感器，不代表狗端真实链路或相机已经验收。真实验证完成后，应在此补充
样本数量、频率、`/lite3/gateway/status` 诊断计数和相机是否运行。

## 保留事项

当前使用 `timestamp_policy: receive`；输出时间是 PC 接收时间，不是已完成时钟同步的测量时间。
本包不发布 TF，不提供 RGB、点云、原始激光或运动控制，也不代表 Nav2/SLAM 已就绪。
