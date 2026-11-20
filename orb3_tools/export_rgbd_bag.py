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
from sensor_msgs.msg import Image


RGB_TOPIC = "/lite3/camera/color/image_raw"
DEPTH_TOPIC = "/lite3/camera/aligned_depth_to_color/image_raw"


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
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    parser.add_argument("--start", type=float, default=0.0,
                        help="Start offset in seconds")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Duration in seconds; 0 means to the end")
    args = parser.parse_args()

    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Output directory is not empty: {output}")

    rgb_dir = output / "rgb"
    depth_dir = output / "depth"
    rgb_dir.mkdir(parents=True)
    depth_dir.mkdir(parents=True)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(StorageFilter(topics=[RGB_TOPIC, DEPTH_TOPIC]))

    rgb_files = {}
    depth_files = {}
    first_stamp = None
    written = 0

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        msg = deserialize_message(serialized, Image)
        ns = stamp_ns(msg)

        if first_stamp is None:
            first_stamp = ns

        relative_time = (ns - first_stamp) / 1e9

        if relative_time < args.start:
            continue

        if args.duration > 0 and relative_time > args.start + args.duration:
            break

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

        written += 1
        if written % 500 == 0:
            print(
                f"messages={written}, "
                f"rgb={len(rgb_files)}, depth={len(depth_files)}"
            )

    paired = sorted(set(rgb_files) & set(depth_files))
    if not paired:
        raise RuntimeError("No synchronized RGB-D pairs found")

    with (output / "associations.txt").open("w") as association_file, \
         (output / "rgb.txt").open("w") as rgb_file, \
         (output / "depth.txt").open("w") as depth_file:

        rgb_file.write("# timestamp filename\n")
        depth_file.write("# timestamp filename\n")

        for ns in paired:
            timestamp = stamp_text(ns)
            rgb_path = rgb_files[ns]
            depth_path = depth_files[ns]

            association_file.write(
                f"{timestamp} {rgb_path} "
                f"{timestamp} {depth_path}\n"
            )
            rgb_file.write(f"{timestamp} {rgb_path}\n")
            depth_file.write(f"{timestamp} {depth_path}\n")

    print(f"Finished: {len(paired)} synchronized RGB-D pairs")
    print(f"Output: {output}")


if __name__ == "__main__":
    main()