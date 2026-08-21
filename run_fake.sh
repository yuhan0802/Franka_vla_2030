#!/usr/bin/env bash
set -euo pipefail
docker compose run --rm franka_vr ros2 launch franka_vr franka_twist.launch.py \
  use_fake_hardware:=true load_gripper:="${LOAD_GRIPPER:-true}" use_rviz:="${USE_RVIZ:-true}"
