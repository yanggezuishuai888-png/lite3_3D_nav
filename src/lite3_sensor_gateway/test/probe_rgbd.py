#!/usr/bin/env python3
"""Measure received RGB-D rates and one-to-one source-timestamp pairing."""

import argparse
from collections import deque
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=12.0)
    parser.add_argument('--prefix', default='/lite3')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('lite3_rgbd_probe')
    waiting = [deque(), deque()]
    counts = [0, 0]
    formats = [None, None]
    deltas_ms = []

    def receive(index, message):
        counts[index] += 1
        formats[index] = (message.width, message.height, message.encoding,
                          message.header.frame_id)
        stamp = stamp_ns(message)
        other = waiting[1 - index]
        if other:
            nearest = min(other, key=lambda candidate: abs(candidate - stamp))
            delta_ms = abs(nearest - stamp) / 1e6
            if delta_ms <= 50:
                other.remove(nearest)
                deltas_ms.append(delta_ms)
                return
        waiting[index].append(stamp)
        if len(waiting[index]) > 100:
            waiting[index].popleft()

    topics = (args.prefix + '/camera/color/image_raw',
              args.prefix + '/camera/aligned_depth_to_color/image_raw')
    subscriptions = [
        node.create_subscription(Image, topic,
                                 lambda message, index=index: receive(index, message),
                                 qos_profile_sensor_data)
        for index, topic in enumerate(topics)
    ]
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        elapsed = time.monotonic() - start
        print('elapsed_s={:.1f} color={} ({:.1f} Hz) aligned={} ({:.1f} Hz)'.format(
            elapsed, counts[0], counts[0] / elapsed,
            counts[1], counts[1] / elapsed))
        print('color={} aligned={}'.format(formats[0], formats[1]))
        print('paired_within_50ms={} exact={} max_delta_ms={}'.format(
            len(deltas_ms), sum(delta == 0 for delta in deltas_ms),
            max(deltas_ms) if deltas_ms else 'n/a'))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
