#!/usr/bin/env bash
set -euo pipefail
docker compose run --rm franka_vr bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  colcon build --symlink-install --cmake-force-configure --packages-select franka_vr
  source install/setup.bash
  ros2 launch franka_vr franka_twist.launch.py \
    use_fake_hardware:=true load_gripper:="'"${LOAD_GRIPPER:-true}"'" use_rviz:="'"${USE_RVIZ:-true}"'"
'
