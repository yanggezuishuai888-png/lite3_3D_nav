
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)


DEPTH_TOPIC = (
    '/lite3/camera/aligned_depth_to_color/image_raw'
)
INFO_TOPIC = (
    '/lite3/camera/aligned_depth_to_color/camera_info'
)

TARGET_FRAME = 'base_link'


def rotate_vector(q, v):
    qv = np.array([q.x, q.y, q.z], dtype=float)
    return v + 2.0 * np.cross(
        qv, np.cross(qv, v) + q.w * v
    )


class WallCheck(Node):

    def __init__(self):
        super().__init__('wall_extrinsic_check')

        self.bridge = CvBridge()
        self.camera_info = None
        self.last_info_stamp = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            INFO_TOPIC,
            self.info_callback,
            sensor_qos
        )

        self.depth_sub = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            sensor_qos
        )

        self.get_logger().info(
            'Waiting for aligned depth and CameraInfo...'
        )

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):

        if self.camera_info is None:
            return

        if msg.header.frame_id != 'camera_color_optical_frame':
            self.get_logger().error(
                'Unexpected depth frame: ' + msg.header.frame_id
            )
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(str(e))
            return

        if msg.encoding == '16UC1':
            depth = depth.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            depth = depth.astype(np.float32)
        else:
            self.get_logger().error(
                'Unsupported encoding: ' + msg.encoding
            )
            return

        h, w = depth.shape

        # Central 40% x 40% region.
        u0, u1 = int(w * 0.30), int(w * 0.70)
        v0, v1 = int(h * 0.30), int(h * 0.70)

        roi = depth[v0:v1, u0:u1]

        vv, uu = np.where(
            np.isfinite(roi) &
            (roi > 0.20) &
            (roi < 4.0)
        )

        valid_count = len(uu)

        if valid_count < 100:
            self.get_logger().warning(
                f'Insufficient valid points: {valid_count}'
            )
            return

        u = uu + u0
        v = vv + v0

        # Limit to at most approximately 3000 points.
        step = max(1, len(u) // 3000)
        u = u[::step]
        v = v[::step]

        z = depth[v, u]

        # Intrinsics from aligned-depth CameraInfo.
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        points_camera = np.column_stack((x, y, z))

        try:
            tf = self.tf_buffer.lookup_transform(
                TARGET_FRAME,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp)
            )
        except Exception as e:
            self.get_logger().warning(
                f'TF lookup failed: {e}'
            )
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        translation = np.array([t.x, t.y, t.z])

        points_base = np.array([
            rotate_vector(q, p) + translation
            for p in points_camera
        ])

        # Robustly reject points far from the dominant plane.
        center0 = np.median(points_base, axis=0)
        distances = np.linalg.norm(
            points_base - center0, axis=1
        )

        keep = distances < np.percentile(distances, 80)
        points_base = points_base[keep]

        center = np.mean(points_base, axis=0)
        centered = points_base - center

        _, _, vh = np.linalg.svd(
            centered, full_matrices=False
        )

        normal = vh[-1]
        normal /= np.linalg.norm(normal)

        # Make normal point generally toward +X.
        if normal[0] < 0:
            normal = -normal

        x_values = points_base[:, 0]
        y_values = points_base[:, 1]
        z_values = points_base[:, 2]

        self.get_logger().info(
            '\n'
            f'Frame: {msg.header.frame_id}\n'
            f'Valid points before filtering: {valid_count}\n'
            f'Points used: {len(points_base)}\n'
            f'Wall center in base_link [m]:\n'
            f'  X={center[0]:.4f}\n'
            f'  Y={center[1]:.4f}\n'
            f'  Z={center[2]:.4f}\n'
            f'Wall X: mean={np.mean(x_values):.4f}, '
            f'std={np.std(x_values):.4f} m\n'
            f'Wall Y range: '
            f'{np.min(y_values):.3f} to {np.max(y_values):.3f} m\n'
            f'Wall Z range: '
            f'{np.min(z_values):.3f} to {np.max(z_values):.3f} m\n'
            f'Plane normal in base_link:\n'
            f'  [{normal[0]:.4f}, '
            f'{normal[1]:.4f}, {normal[2]:.4f}]\n'
            f'Normal alignment with X: {abs(normal[0]):.4f}'
        )


def main():
    rclpy.init()
    node = WallCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
