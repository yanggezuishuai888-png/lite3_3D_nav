# ekf_pkg

Lite3 足式里程计与全姿态 IMU 融合：

```text
/lite3/leg_odom2 ------------------------+
                                         +--> ekf_input_adapter
/lite3/imu/data -------------------------+          |
                                                   +--> robot_localization EKF
                                                        |
                                                        +--> /odometry/filtered
                                                        +--> ekf_odom -> base_link
```

- 直接订阅原始足式里程计；旧的 `lite3_leg_odom_adapter` 不启动。
- 足式里程计融合 `x/y`；实测中厂家 `z` 为不随贴伏/站立变化的常量，因此不使用原值。
- 适配器额外提供局部 `z=0` 伪观测，仅用于在保留 roll/pitch 时防止 EKF 垂直状态发散；它不表示真实机身高度。
- IMU 融合完整 `roll/pitch/yaw` 姿态和三轴角速度。
- 不融合含重力的线加速度，不宣称 EKF 能单独观测高度。
- 输入适配器保留源时间戳，检查帧和新鲜度，并将原始全零协方差替换为有限初值。
- EKF 是 `ekf_odom -> base_link` 的唯一 TF 发布者。

协方差目前是工程初值，正式导航前应通过静止噪声和已知距离/角度运动标定。
