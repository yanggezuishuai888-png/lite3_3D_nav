#!/usr/bin/env bash

_lite3_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_lite3_workspace="$(cd "${_lite3_script_dir}/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy setup not found: /opt/ros/jazzy/setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi
if [[ ! -f "${_lite3_workspace}/install/local_setup.bash" ]]; then
  echo "Workspace is not built: ${_lite3_workspace}/install/local_setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi

# Keep dog-side discovery and CycloneDDS settings out of the PC session.
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO
source /opt/ros/jazzy/setup.bash
source "${_lite3_workspace}/install/local_setup.bash"
export ROS_DOMAIN_ID=74

export LITE3_WORKSPACE="${_lite3_workspace}"
unset _lite3_script_dir _lite3_workspace
