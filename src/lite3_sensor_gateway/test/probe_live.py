#!/usr/bin/env python3
"""Manual read-only probe on the dog. Never invoked by colcon test.

Uses existing domain-0 vendor sensors and a loopback receiver in domain 84.
Does not start vendor services, publish commands, or open sensor hardware.
Run from a terminal sourced with Foxy. Output is JSON; all children exit afterward.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '84'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from diagnostic_msgs.msg import DiagnosticArray

build = Path(sys.argv[1]).resolve()
logs = build / 'live-probe-logs'
logs.mkdir(exist_ok=True)
ports = []
reserved = []
for _ in range(2):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    ports.append(sock.getsockname()[1])
    reserved.append(sock)
for sock in reserved:
    sock.close()
processes, files = [], []
counts, samples, diagnostics = {}, {}, {}


def start(binary, domain, peer_key):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='0' if domain == 0 else '1')
    stream = open(logs / (binary + '.log'), 'w')
    files.append(stream)
    args = [str(build / binary), '--ros-args', '-p', 'bind_ip:=127.0.0.1',
            '-p', peer_key + ':=127.0.0.1', '-p', 'telemetry_port:=' + str(ports[0]),
            '-p', 'image_port:=' + str(ports[1])]
    process = subprocess.Popen(args, env=env, stdout=stream, stderr=subprocess.STDOUT)
    processes.append(process)


def receive(name, message):
    counts[name] = counts.get(name, 0) + 1
    if hasattr(message, 'header'):
        samples[name] = {'frame_id': message.header.frame_id,
                         'stamp_sec': message.header.stamp.sec}
    elif hasattr(message, 'data'):
        samples[name] = message.data


def diagnostic(message):
    for status in message.status:
        diagnostics[status.name] = {'level': int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else int(status.level), 'message': status.message,
                                    'values': {v.key: v.value for v in status.values}}


rclpy.init()
node = rclpy.create_node('lite3_gateway_live_probe')
subscriptions = []
try:
    for name, cls in [('imu/data', Imu), ('leg_odom2', Odometry), ('handle_state', Twist),
                      ('ultrasound_distance', Float64), ('sensor_status/imu_isalive', Bool)]:
        subscriptions.append(node.create_subscription(
            cls, '/lite3/' + name, lambda msg, key=name: receive(key, msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        DiagnosticArray, '/lite3/gateway/status', diagnostic, 10))
    start('receiver', 84, 'sender_ip')
    start('sender', 0, 'pc_ip')
    end = time.monotonic() + 12
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('gateway exited; inspect ' + str(logs))
    result = {'duration_seconds': 12, 'counts': counts, 'samples': samples,
              'diagnostics': diagnostics,
              'test_scope': 'real domain-0 chassis -> localhost UDP -> isolated domain 84; no PC test'}
    (logs / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, default=lambda x: x.decode("utf-8", errors="replace") if isinstance(x, bytes) else str(x)))
    print(json.dumps(result, indent=2, ensure_ascii=False, default=lambda x: x.decode("utf-8", errors="replace") if isinstance(x, bytes) else str(x)))
    if any(counts.get(key, 0) == 0 for key in ['imu/data', 'leg_odom2', 'ultrasound_distance']):
        raise RuntimeError('expected chassis streams missing')
finally:
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    for stream in files:
        stream.close()
    node.destroy_node()
    rclpy.shutdown()
