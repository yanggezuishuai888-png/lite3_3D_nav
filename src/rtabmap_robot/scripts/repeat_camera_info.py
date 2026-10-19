#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoRepublisher(Node):
    def __init__(self):
        super().__init__("camera_info_republisher")
        qos = QoSProfile(depth=10)
        self._camera_info = None
        self._warned_missing_camera_info = False
        self._publisher = self.create_publisher(
            CameraInfo, "/openloris/color/camera_info", qos
        )
        self.create_subscription(
            CameraInfo, "/d400/color/camera_info", self._cache_camera_info, qos
        )
        self.create_subscription(
            Image, "/d400/color/image_raw", self._publish_for_image, qos
        )

    def _cache_camera_info(self, message):
        self._camera_info = message
        self.get_logger().info(
            f"Cached {message.width}x{message.height} color camera calibration"
        )

    def _publish_for_image(self, image):
        if self._camera_info is None:
            if not self._warned_missing_camera_info:
                self.get_logger().warning(
                    "RGB image arrived before CameraInfo; waiting for calibration"
                )
                self._warned_missing_camera_info = True
            return

        camera_info = copy.copy(self._camera_info)
        camera_info.header = copy.copy(image.header)
        self._publisher.publish(camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
