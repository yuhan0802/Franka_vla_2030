#!/usr/bin/env bash
set -euo pipefail

if (( $# < 3 )); then
  echo "Usage: $0 <merged_dataset_name> <input_dataset_1> <input_dataset_2> [...]" >&2
  exit 2
fi

docker compose run --rm --no-TTY franka_vr bash -lc '
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  colcon build --symlink-install --packages-select franka_vr
  source install/setup.bash
  ros2 run franka_vr merge_lerobot_datasets.py "$@"
' _ "$@"
