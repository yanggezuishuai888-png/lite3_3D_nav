#!/usr/bin/env bash

_lite3_receiver_run() {
  local script_dir workspace pid_file pid wrapper_pid existing existing_pgid existing_args status old_int old_term i
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" || return 1
  workspace="$(cd "${script_dir}/.." && pwd)" || return 1
  source "${script_dir}/pc_ros_env.bash" || return 1

  pid_file="/tmp/lite3_sensor_gateway_receiver_${UID}.pid"
  if [[ -f "${pid_file}" ]]; then
    existing="$(<"${pid_file}")"
    if [[ "${existing}" =~ ^[0-9]+$ ]] && kill -0 "${existing}" 2>/dev/null; then
      existing_pgid="$(ps -o pgid= -p "${existing}" 2>/dev/null | tr -d ' ')"
      existing_args="$(ps -o args= -p "${existing}" 2>/dev/null || true)"
      if [[ "${existing_pgid}" == "${existing}" && "${existing_args}" == *"ros2 launch lite3_sensor_gateway receiver.launch.py"* ]]; then
        printf 'Lite3 receiver is already running (PID %s).\n' "${existing}"
        return 1
      fi
      printf 'PID file points to a live process that is not this receiver: %s\n' "${existing}" >&2
      return 1
    fi
    rm -f -- "${pid_file}"
  fi

  rm -f -- "${pid_file}"
  setsid --wait bash -c '
    pid_file="$1"
    shift
    printf "%s\n" "$$" > "${pid_file}" || exit 1
    exec "$@"
  ' _ "${pid_file}" ros2 launch lite3_sensor_gateway receiver.launch.py \
    "config:=${workspace}/config/receiver.pc.yaml" &
  wrapper_pid=$!

  for ((i=0; i<100; i++)); do
    if [[ -s "${pid_file}" ]]; then
      pid="$(<"${pid_file}")"
      [[ "${pid}" =~ ^[0-9]+$ ]] && break
    fi
    if ! kill -0 "${wrapper_pid}" 2>/dev/null; then
      wait "${wrapper_pid}" 2>/dev/null || true
      rm -f -- "${pid_file}"
      printf 'Receiver process failed before recording its PID.\n' >&2
      return 1
    fi
    sleep 0.05
  done
  if [[ ! "${pid:-}" =~ ^[0-9]+$ ]]; then
    kill -INT "${wrapper_pid}" 2>/dev/null || true
    wait "${wrapper_pid}" 2>/dev/null || true
    rm -f -- "${pid_file}"
    printf 'Timed out waiting for receiver process PID.\n' >&2
    return 1
  fi

  _lite3_receiver_stop_group() {
    local group_pid="$1" sig="$2" i
    kill -"${sig}" -- "-${group_pid}" 2>/dev/null || true
    for ((i=0; i<50; i++)); do
      kill -0 -- "-${group_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-${group_pid}" 2>/dev/null; then
      kill -TERM -- "-${group_pid}" 2>/dev/null || true
      for ((i=0; i<20; i++)); do
        kill -0 -- "-${group_pid}" 2>/dev/null || break
        sleep 0.1
      done
    fi
    if kill -0 -- "-${group_pid}" 2>/dev/null; then
      kill -KILL -- "-${group_pid}" 2>/dev/null || true
    fi
    if [[ -f "${pid_file}" && "$(<"${pid_file}")" == "${group_pid}" ]]; then
      rm -f -- "${pid_file}"
    fi
  }
  old_int="$(trap -p INT)"
  old_term="$(trap -p TERM)"
  trap "_lite3_receiver_stop_group '${pid}' INT" INT
  trap "_lite3_receiver_stop_group '${pid}' TERM" TERM

  printf 'Lite3 receiver process group started (PID/PGID %s). Press Ctrl+C to stop it.\n' "${pid}"
  wait "${wrapper_pid}"
  status=$?

  if [[ -n "${old_int}" ]]; then eval "${old_int}"; else trap - INT; fi
  if [[ -n "${old_term}" ]]; then eval "${old_term}"; else trap - TERM; fi
  if [[ -f "${pid_file}" && "$(<"${pid_file}")" == "${pid}" ]]; then
    rm -f -- "${pid_file}"
  fi
  if kill -0 -- "-${pid}" 2>/dev/null; then
    _lite3_receiver_stop_group "${pid}" INT
  fi
  unset -f _lite3_receiver_stop_group
  return "${status}"
}

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  _lite3_receiver_source_entry() {
    local result
    _lite3_receiver_run
    result=$?
    unset -f _lite3_receiver_run _lite3_receiver_source_entry
    return "${result}"
  }
  _lite3_receiver_source_entry
  return $?
fi
_lite3_receiver_run
_lite3_receiver_status=$?
unset -f _lite3_receiver_run
exit "${_lite3_receiver_status}"
