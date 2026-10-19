#!/usr/bin/env bash
set -euo pipefail

bag_path="${1:-/home/ruming/bag/market1-1_ros2}"
rate="${RATE:-0.5}"
package_prefix="$(ros2 pkg prefix rtabmap_robot)"
qos_file="${package_prefix}/share/rtabmap_robot/config/market1_qos_overrides.yaml"

if [[ ! -d "${bag_path}" ]]; then
  printf 'Bag directory does not exist: %s\n' "${bag_path}" >&2
  exit 1
fi

exec ros2 bag play "${bag_path}" \
  --clock \
  --rate "${rate}" \
  --qos-profile-overrides-path "${qos_file}" \
  --topics \
    /d400/color/image_raw \
    /d400/aligned_depth_to_color/image_raw \
    /d400/color/camera_info \
    /tf_static
