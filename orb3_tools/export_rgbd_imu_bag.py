#!/usr/bin/env python3

import argparse
from pathlib import Path

import cv2
import numpy as np

from rclpy.serialization import deserialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageFilter,
    StorageOptions,
)
from sensor_msgs.msg import Image, Imu


RGB_TOPIC = "/lite3/camera/color/image_raw"
DEPTH_TOPIC = "/lite3/camera/aligned_depth_to_color/image_raw"
IMU_TOPIC = "/lite3/imu/data"


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def stamp_text(ns):
    return f"{ns // 1_000_000_000}.{ns % 1_000_000_000:09d}"


def rgb_to_bgr(msg):
    if msg.encoding != "rgb8":
        raise RuntimeError(f"Unexpected RGB encoding: {msg.encoding}")

    image = np.frombuffer(msg.data, dtype=np.uint8)
    image = image.reshape(msg.height, msg.step)
    image = image[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


def depth_to_u16(msg):
    if msg.encoding not in ("16UC1", "mono16"):
        raise RuntimeError(f"Unexpected depth encoding: {msg.encoding}")

    endian = ">u2" if msg.is_bigendian else "<u2"
    image = np.frombuffer(msg.data, dtype=endian)
    image = image.reshape(msg.height, msg.step // 2)
    return image[:, : msg.width].astype(np.uint16, copy=False)


def main():
    parser = argparse.ArgumentParser(
        description="Export a ROS 2 bag for offline ORB-SLAM3 RGB-D-Inertial."
    )
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--start", type=float, default=0.0,
                        help="Start offset from the first camera frame, seconds")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Duration in seconds; 0 means to the end")
    parser.add_argument(
        "--camera-time-offset",
        type=float,
        default=0.100,
        help=(
            "Seconds added to RGB-D header timestamps. The Lite3 first-pass "
            "calibration is +0.100 s (physical image time = stamp + offset)."
        ),
    )
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {output}")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )

    available_topics = {entry.name for entry in reader.get_all_topics_and_types()}
    required_topics = {RGB_TOPIC, DEPTH_TOPIC, IMU_TOPIC}
    missing_topics = sorted(required_topics - available_topics)
    if missing_topics:
        raise RuntimeError(
            "Bag is missing required topic(s): " + ", ".join(missing_topics)
        )

    reader.set_filter(
        StorageFilter(topics=[RGB_TOPIC, DEPTH_TOPIC, IMU_TOPIC])
    )

    rgb_dir = output / "rgb"
    depth_dir = output / "depth"
    rgb_dir.mkdir(parents=True)
    depth_dir.mkdir(parents=True)

    rgb_files = {}
    depth_files = {}
    imu_samples = {}
    imu_frames = set()
    first_camera_stamp = None
    selected_messages = 0

    while reader.has_next():
        topic, serialized, _ = reader.read_next()

        if topic == IMU_TOPIC:
            msg = deserialize_message(serialized, Imu)
            ns = stamp_ns(msg)
            imu_samples[ns] = (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
            imu_frames.add(msg.header.frame_id)
            continue

        msg = deserialize_message(serialized, Image)
        ns = stamp_ns(msg)
        if first_camera_stamp is None:
            first_camera_stamp = ns

        relative_time = (ns - first_camera_stamp) / 1e9
        if relative_time < args.start:
            continue
        if args.duration > 0 and relative_time > args.start + args.duration:
            continue

        filename = f"{ns}.png"
        if topic == RGB_TOPIC:
            relative_path = f"rgb/{filename}"
            ok = cv2.imwrite(
                str(output / relative_path),
                rgb_to_bgr(msg),
                [cv2.IMWRITE_PNG_COMPRESSION, 3],
            )
            if not ok:
                raise RuntimeError(f"Failed to write {relative_path}")
            rgb_files[ns] = relative_path
        elif topic == DEPTH_TOPIC:
            relative_path = f"depth/{filename}"
            ok = cv2.imwrite(
                str(output / relative_path),
                depth_to_u16(msg),
                [cv2.IMWRITE_PNG_COMPRESSION, 1],
            )
            if not ok:
                raise RuntimeError(f"Failed to write {relative_path}")
            depth_files[ns] = relative_path

        selected_messages += 1
        if selected_messages % 500 == 0:
            print(
                f"camera_messages={selected_messages}, "
                f"rgb={len(rgb_files)}, depth={len(depth_files)}, "
                f"imu={len(imu_samples)}"
            )

    paired = sorted(set(rgb_files) & set(depth_files))
    if not paired:
        raise RuntimeError("No synchronized RGB-D pairs found")
    if not imu_samples:
        raise RuntimeError("No IMU samples found")

    offset_ns = round(args.camera_time_offset * 1_000_000_000)
    imu_stamps = sorted(imu_samples)
    paired = [
        ns for ns in paired
        if imu_stamps[0] <= ns + offset_ns <= imu_stamps[-1]
    ]
    if not paired:
        raise RuntimeError(
            "No RGB-D pairs remain inside the IMU time range after applying "
            f"camera offset {args.camera_time_offset:+.6f} s"
        )

    first_corrected = paired[0] + offset_ns
    last_corrected = paired[-1] + offset_ns
    imu_margin_ns = 100_000_000
    selected_imu_stamps = [
        ns for ns in imu_stamps
        if first_corrected - imu_margin_ns <= ns <= last_corrected + imu_margin_ns
    ]

    with (output / "associations.txt").open("w") as association_file, \
         (output / "rgb.txt").open("w") as rgb_file, \
         (output / "depth.txt").open("w") as depth_file:

        association_file.write(
            f"# RGB-D timestamps include camera_time_offset="
            f"{args.camera_time_offset:+.9f} s\n"
        )
        rgb_file.write("# timestamp filename\n")
        depth_file.write("# timestamp filename\n")

        for ns in paired:
            corrected_timestamp = stamp_text(ns + offset_ns)
            rgb_path = rgb_files[ns]
            depth_path = depth_files[ns]
            association_file.write(
                f"{corrected_timestamp} {rgb_path} "
                f"{corrected_timestamp} {depth_path}\n"
            )
            rgb_file.write(f"{corrected_timestamp} {rgb_path}\n")
            depth_file.write(f"{corrected_timestamp} {depth_path}\n")

    with (output / "imu.txt").open("w") as imu_file:
        imu_file.write("# timestamp gx gy gz ax ay az\n")
        imu_file.write("# angular velocity: rad/s; linear acceleration: m/s^2\n")
        for ns in selected_imu_stamps:
            gx, gy, gz, ax, ay, az = imu_samples[ns]
            imu_file.write(
                f"{stamp_text(ns)} "
                f"{gx:.12g} {gy:.12g} {gz:.12g} "
                f"{ax:.12g} {ay:.12g} {az:.12g}\n"
            )

    max_rgbd_gap = max(
        ((b - a) / 1e9 for a, b in zip(paired, paired[1:])),
        default=0.0,
    )
    max_imu_gap = max(
        ((b - a) / 1e9
         for a, b in zip(selected_imu_stamps, selected_imu_stamps[1:])),
        default=0.0,
    )

    print(f"Finished: {len(paired)} synchronized RGB-D pairs")
    print(f"IMU samples: {len(selected_imu_stamps)}; frame_id={sorted(imu_frames)}")
    print(f"Camera timestamp correction: {args.camera_time_offset:+.6f} s")
    print(f"Maximum RGB-D gap: {max_rgbd_gap:.6f} s")
    print(f"Maximum IMU gap: {max_imu_gap:.6f} s")
    if max_rgbd_gap > 0.5:
        print("WARNING: RGB-D contains a gap longer than 0.5 s")
    if max_imu_gap > 0.05:
        print("WARNING: IMU contains a gap longer than 0.05 s")
    print(f"Output: {output}")


if __name__ == "__main__":
    main()
