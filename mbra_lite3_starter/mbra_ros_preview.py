#!/usr/bin/env python3
"""ROS 2 Jazzy: RGB images -> UCLA MBRA-PG -> /mbra/local_path.

Visualization ONLY. No /cmd_vel, no movement/actuation, no Nav2 required.
The goal is a robot-FIXED local test goal in the output frame (not a map goal).
"""

from __future__ import annotations

from collections import deque
import importlib.util
import math
from pathlib import Path
import time

import numpy as np
import torch  # Import before ONNX Runtime session creation for CUDA libraries.
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage


def load_official_model(model_dir: Path):
    inference_py = model_dir / "inference.py"
    model_path = model_dir / "mbra.onnx"
    info_path = model_dir / "model_info.yaml"
    for f in (inference_py, model_path, info_path):
        if not f.is_file():
            raise FileNotFoundError(f"Official MBRA model file missing: {f}")
    spec = importlib.util.spec_from_file_location("mbra_pg_ros_official", inference_py)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import {inference_py}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.MBRAPGNavigator(
        onnx_path=str(model_path),
        device="cuda" if torch.cuda.is_available() else "cpu",
    )


class MBRAPreview(Node):
    def __init__(self):
        super().__init__("mbra_ros_preview")
        default_model_dir = str(Path(__file__).resolve().parent / "MBRA_PG_Official")
        self.declare_parameter("model_dir", default_model_dir)
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("goal_x", 3.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("sample_hz", 5.0)
        self.declare_parameter("infer_hz", 2.0)
        self.declare_parameter("max_image_age_sec", 1.5)
        self.declare_parameter("require_gpu", True)

        def param(name):
            return self.get_parameter(name).value

        self.frame_id = str(param("frame_id"))
        self.image_topic = str(param("image_topic"))
        sample_hz = float(param("sample_hz"))
        infer_hz = float(param("infer_hz"))
        self.max_image_age = float(param("max_image_age_sec"))
        self.require_gpu = bool(param("require_gpu"))
        if not self.frame_id or not self.image_topic:
            raise ValueError("frame_id and image_topic must be nonempty")
        if not all(math.isfinite(v) and v > 0 for v in
                   (sample_hz, infer_hz, self.max_image_age)):
            raise ValueError("sample_hz, infer_hz and max_image_age_sec must be positive")
        if not math.isclose(sample_hz, 5.0, abs_tol=0.01):
            self.get_logger().warning(
                "Official MBRA model_info.yaml specifies 5 Hz image sampling; "
                "different sampling may degrade prediction quality."
            )

        self.bridge = CvBridge()
        self.history = deque(maxlen=6)
        self.latest_image_monotonic = 0.0
        self.last_sample_monotonic = float("-inf")
        self.last_error_log = float("-inf")
        self.sample_period = 1.0 / sample_hz
        self.tick_count = 0
        self.navigator = load_official_model(Path(str(param("model_dir"))).expanduser().resolve())
        providers = self.navigator._session.get_providers()
        self.get_logger().info(f"torch.cuda.is_available(): {torch.cuda.is_available()}")
        self.get_logger().info(f"ORT available providers: {ort.get_available_providers()}")
        self.get_logger().info(f"MBRA session providers: {providers}")
        if self.require_gpu and (
            not torch.cuda.is_available() or "CUDAExecutionProvider" not in providers
        ):
            raise RuntimeError(
                "MBRA session lacks CUDAExecutionProvider. "
                "Fix CUDA libraries or run with -p require_gpu:=false for CPU debugging only."
            )

        self.publisher = self.create_publisher(RosPath, "/mbra/local_path", 10)
        self.subscription = self.create_subscription(
            RosImage, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / infer_hz, self.on_infer_timer)
        self.get_logger().info(
            f"VISUALIZATION ONLY: {self.image_topic} -> /mbra/local_path, "
            f"frame={self.frame_id}, sampling={sample_hz:g}Hz, inference={infer_hz:g}Hz"
        )

    def warn_sparingly(self, message: str):
        now = time.monotonic()
        if now - self.last_error_log >= 5.0:
            self.get_logger().warning(message)
            self.last_error_log = now

    def on_image(self, msg: RosImage):
        now = time.monotonic()
        if now - self.last_sample_monotonic < self.sample_period:
            return
        try:
            # cv_bridge handles bgr8/rgb8/mono8 conversions to true RGB order.
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            if rgb.ndim != 3 or rgb.shape[2] != 3:
                raise ValueError(f"Unexpected image shape {rgb.shape}")
            if rgb.dtype != np.uint8:
                raise ValueError(f"Expected uint8 RGB image, got {rgb.dtype}")
            # The official inference.py later handles 96x96 resize and
            # ImageNet normalization; do NOT normalize twice here.
            chw = np.ascontiguousarray(
                np.moveaxis(rgb, -1, 0), dtype=np.float32
            ) / 255.0
        except (CvBridgeError, ValueError, TypeError) as exc:
            self.warn_sparingly(f"RGB conversion failed: {exc}")
            return
        self.history.append(chw)
        self.last_sample_monotonic = now
        self.latest_image_monotonic = now

    def on_infer_timer(self):
        if len(self.history) < 6:
            if self.tick_count % 10 == 0:
                self.get_logger().info(f"Waiting for 6 frames: {len(self.history)}/6")
            self.tick_count += 1
            return
        age = time.monotonic() - self.latest_image_monotonic
        if age > self.max_image_age:
            self.warn_sparingly(f"RGB image stale ({age:.2f}s); not publishing a path")
            return

        # These ROS parameters can be changed while the node runs:
        # ros2 param set /mbra_ros_preview goal_y 1.0
        gx = float(self.get_parameter("goal_x").value)
        gy = float(self.get_parameter("goal_y").value)
        if not (math.isfinite(gx) and math.isfinite(gy)):
            self.warn_sparingly("Goal has NaN/Inf; not running inference")
            return
        obs = np.stack(tuple(self.history), axis=0)[None, ...].astype(np.float32)
        goal_xy = np.array([gx, gy], dtype=np.float32)
        try:
            start = time.perf_counter()
            trajectory, _ = self.navigator.inference_trajectory(obs, goal_xy=goal_xy)
            latency_ms = (time.perf_counter() - start) * 1000.0
            traj = np.asarray(trajectory)
            if traj.shape != (1, 1, 8, 2):
                raise ValueError(f"Unexpected trajectory shape: {traj.shape}")
            if not np.isfinite(traj).all():
                raise ValueError("Trajectory contains NaN/Inf")
        except Exception as exc:
            self.warn_sparingly(f"MBRA inference failed: {type(exc).__name__}: {exc}")
            return

        stamp = self.get_clock().now().to_msg()
        path = RosPath()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        for x, y in traj[0, 0]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Valid identity quaternion.
            path.poses.append(pose)
        self.publisher.publish(path)
        self.tick_count += 1
        if self.tick_count % 10 == 0:
            self.get_logger().info(
                f"path={len(path.poses)} poses, goal=({gx:.2f},{gy:.2f})m, "
                f"latest image age={age:.2f}s, inference={latency_ms:.1f}ms"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MBRAPreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
