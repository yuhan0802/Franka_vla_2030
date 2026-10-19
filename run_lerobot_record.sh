#!/usr/bin/env bash
set -euo pipefail
DATASET_NAME="${1:-fr3_teleop}"
DURATION="${2:-30}"
TASK="${3:-Move the FR3 end effector along a small smooth random trajectory}"
docker compose run --rm franka_vr bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  if [[ ! -f install/setup.bash ]]; then
    echo "franka_vr has not been built. Run this once:" >&2
    echo "  docker compose run --rm franka_vr bash -lc '\''source /opt/ros/humble/setup.bash; cd /ws_franka_vr; colcon build --symlink-install --packages-select franka_vr'\''" >&2
    exit 1
  fi
  source install/setup.bash
  exec ros2 launch franka_vr lerobot_recording.launch.py \
    dataset_name:="'"${DATASET_NAME}"'" duration:="'"${DURATION}"'" task:="'"${TASK}"'"
'
