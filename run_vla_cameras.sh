#!/usr/bin/env bash
set -euo pipefail

# Three VLA image streams only: left/right C922 external cameras and the
# RealSense D435i wrist camera. This never creates a robot controller or
# records an episode.
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$root_dir/scripts/runtime_log_root.sh"
camera_mode="${VLA_CAMERA_MODE:-calibrated}"
case "$camera_mode" in
  raw|calibrated) ;;
  *) echo "VLA_CAMERA_MODE must be raw or calibrated" >&2; exit 2 ;;
esac
echo "Camera mode: $camera_mode"
docker compose run --rm -e "VLA_CAMERA_MODE=$camera_mode" franka_vr bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  flock /ws_franka_vr/.colcon-build.lock \
    colcon build --symlink-install --cmake-force-configure --packages-select franka_vr
  source install/setup.bash
  exec ros2 launch franka_vr vla_cameras.launch.py
'
