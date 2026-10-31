#!/usr/bin/env bash

pid_file="/tmp/lite3_sensor_gateway_receiver_${UID}.pid"
if [[ ! -f "${pid_file}" ]]; then
  echo "Lite3 gateway PID file not found; it may already be stopped."
  exit 0
fi

pid="$(<"${pid_file}")"
if [[ ! "${pid}" =~ ^[0-9]+$ ]]; then
  echo "Invalid PID file: ${pid_file}" >&2
  exit 1
fi

pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ')"
args="$(ps -o args= -p "${pid}" 2>/dev/null || true)"
if [[ "${pgid}" != "${pid}" || ( "${args}" != *"ros2 launch lite3_sensor_gateway receiver.launch.py"* && "${args}" != *"ros2 launch lite3_sensor_gateway receiver_with_command.launch.py"* ) ]]; then
  if ! kill -0 "${pid}" 2>/dev/null; then
    rm -f -- "${pid_file}"
    echo "Lite3 receiver is already stopped."
    exit 0
  fi
  echo "PID file does not identify a Lite3 gateway process group; leaving it untouched." >&2
  exit 1
fi

kill -INT -- "-${pid}" 2>/dev/null || true
for ((i=0; i<50; i++)); do
  kill -0 -- "-${pid}" 2>/dev/null || break
  sleep 0.1
done
if kill -0 -- "-${pid}" 2>/dev/null; then
  kill -TERM -- "-${pid}" 2>/dev/null || true
  for ((i=0; i<20; i++)); do
    kill -0 -- "-${pid}" 2>/dev/null || break
    sleep 0.1
  done
fi
if kill -0 -- "-${pid}" 2>/dev/null; then
  kill -KILL -- "-${pid}" 2>/dev/null || true
fi

if [[ "$(<"${pid_file}" 2>/dev/null)" == "${pid}" ]]; then
  rm -f -- "${pid_file}"
fi
echo "Stopped Lite3 gateway process group ${pid}."
