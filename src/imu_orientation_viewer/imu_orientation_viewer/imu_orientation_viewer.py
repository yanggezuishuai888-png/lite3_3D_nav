
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class ImuOrientationViewer(Node):

    def __init__(self):
        super().__init__('imu_orientation_viewer')

        # 参数设置
        self.declare_parameter('imu_topic', '/lite3/imu/data')
        self.declare_parameter('world_frame', 'imu_world')
        self.declare_parameter(
            'child_frame', 'imu_orientation_check'
        )

        self.imu_topic = self.get_parameter(
            'imu_topic'
        ).value

        self.world_frame = self.get_parameter(
            'world_frame'
        ).value

        self.child_frame = self.get_parameter(
            'child_frame'
        ).value

        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅IMU
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.last_log_time = 0

        self.get_logger().info(
            f'IMU Viewer started. Topic: {self.imu_topic}'
        )

    # 四元数转换为欧拉角
    def quaternion_to_rpy(self, x, y, z, w):

        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y)
        )

        sin_pitch = 2.0 * (w * y - z * x)

        sin_pitch = max(-1.0, min(1.0, sin_pitch))

        pitch = math.asin(sin_pitch)

        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw)
        )

    # IMU回调函数
    def imu_callback(self, msg):

        # 获取四元数
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # 检查四元数是否合法
        norm = math.sqrt(x*x + y*y + z*z + w*w)

        if not math.isfinite(norm) or norm < 1e-8:
            self.get_logger().warning(
                'Invalid IMU quaternion!'
            )
            return

        # 归一化四元数
        x /= norm
        y /= norm
        z /= norm
        w /= norm

        # 转换为欧拉角
        roll, pitch, yaw = self.quaternion_to_rpy(
            x, y, z, w
        )

        # 创建TF变换
        transform = TransformStamped()

        # 保留IMU原始时间戳
        transform.header.stamp = msg.header.stamp

        # 虚拟世界坐标系
        transform.header.frame_id = self.world_frame

        # IMU姿态显示坐标系
        transform.child_frame_id = self.child_frame

        # 坐标原点
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # 姿态四元数
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w

        # 发布TF
        self.tf_broadcaster.sendTransform(transform)

        # 每秒打印一次数据，避免刷屏
        now = self.get_clock().now().nanoseconds

        if now - self.last_log_time >= 1_000_000_000:

            self.last_log_time = now

            self.get_logger().info(
                '\n'
                '========== IMU Orientation ==========\n'
                f'Roll  : {roll:8.3f} deg\n'
                f'Pitch : {pitch:8.3f} deg\n'
                f'Yaw   : {yaw:8.3f} deg\n'
                '-------------------------------------\n'
                f'Gyro X: {msg.angular_velocity.x: .5f}\n'
                f'Gyro Y: {msg.angular_velocity.y: .5f}\n'
                f'Gyro Z: {msg.angular_velocity.z: .5f}\n'
                '-------------------------------------\n'
                f'Acc X : {msg.linear_acceleration.x: .4f}\n'
                f'Acc Y : {msg.linear_acceleration.y: .4f}\n'
                f'Acc Z : {msg.linear_acceleration.z: .4f}\n'
                '====================================='
            )


def main(args=None):

    rclpy.init(args=args)

    node = ImuOrientationViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
