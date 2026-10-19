#!/usr/bin/env bash
set -euo pipefail

# Three VLA image streams only: left/right C922 external cameras and the
# RealSense D435i wrist camera. This never creates a robot controller or
# records an episode.
docker compose run --rm franka_vr bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  flock /ws_franka_vr/.colcon-build.lock \
    colcon build --symlink-install --cmake-force-configure --packages-select franka_vr
  source install/setup.bash
  exec ros2 launch franka_vr vla_cameras.launch.py
'
