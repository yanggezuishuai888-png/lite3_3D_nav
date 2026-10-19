# Lite3 只读传感器网关

C++17 / ament_cmake 包，包含 `sender`（Jetson/Foxy）和 `receiver`（PC/Jazzy）两个节点。通过自定义 UDP 协议转发 IMU、里程计、超声波、遥控状态、传感器存活状态、RGB、对齐深度和内参；原始深度可选。

- [编译、启动、话题映射、时钟门控及 TF 待办](docs/usage.md)
- [UDP 协议定义](docs/protocol.md)
- [本机验证记录](docs/validation.md)
- 默认：狗 10.42.0.2 → PC 10.42.0.1，UDP 45001/45003；狗 ROS 域保持不变，PC 使用 74。
- 生产数据处理全部 C++；无控制输出、无厂家服务管理、无相机硬件独占。
- 网关已按跨机时钟探测结果门控 IMU、里程计与 RGB-D；校时合格前不发布融合数据。TF 和外参仍待配置，不等于已经可用于可靠导航。
