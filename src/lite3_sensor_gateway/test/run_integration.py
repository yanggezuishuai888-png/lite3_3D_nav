#!/usr/bin/env python3
"""Exercise two independent ROS domains without touching vendor nodes or services."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys

build = Path(sys.argv[1]).resolve()
blocked = len(sys.argv) > 2 and sys.argv[2] == 'blocked'
logs = build / 'integration-logs'
logs.mkdir(exist_ok=True)
processes = []
files = []
sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2)]
for sock in sockets:
    sock.bind(('127.0.0.1', 0))
ports = [sock.getsockname()[1] for sock in sockets]
for sock in sockets:
    sock.close()


def start(binary, name, domain, params):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1')
    command = [str(build / binary), '--ros-args']
    for key, value in params.items():
        command += ['-p', '{}:={}'.format(key, value)]
    stream = open(logs / (name + '.log'), 'w')
    files.append(stream)
    process = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT)
    processes.append(process)
    return process


try:
    shared = {'bind_ip': '127.0.0.1', 'telemetry_port': ports[0], 'image_port': ports[1]}
    receiver = start('receiver', 'receiver', 82,
                     dict(shared, sender_ip='127.0.0.1', reply_time_probes='false' if blocked else 'true'))
    sender = start('sender', 'sender', 81,
                   dict(shared, pc_ip='127.0.0.1', enable_color='true',
                        enable_aligned_depth='true', image_max_mbps=200.0))
    checker = start('integration_fixture', 'checker', 82, {'mode': 'blocked' if blocked else 'check'})
    publisher = start('integration_fixture', 'publisher', 81, {'mode': 'publish'})
    result = checker.wait(timeout=25)
    if sender.poll() is not None or receiver.poll() is not None:
        raise RuntimeError('gateway terminated unexpectedly')
    print((logs / 'checker.log').read_text())
    if result != 0:
        raise RuntimeError('integration check failed; see ' + str(logs))
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
