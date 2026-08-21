#!/usr/bin/env bash
set -euo pipefail

# Start Gazebo physics, MoveIt Servo, and the trajectory controller that the
# Quest bridge uses.  Run ./run_vr.sh in a second terminal afterwards.
docker compose run --rm franka_vr \
  ros2 launch franka_vr gazebo_vr.launch.py \
  load_gripper:=true use_rviz:="${USE_RVIZ:-true}"
