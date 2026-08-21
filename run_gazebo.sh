#!/usr/bin/env bash
set -euo pipefail

# Gazebo Sim runs a physics simulation entirely in the container.  The launch
# also starts MoveIt so RViz can plan and execute a dragged end-effector goal.
# It never connects to the FR3 FCI and does not require ROBOT_IP.
docker compose run --rm franka_vr \
  ros2 launch franka_vr gazebo_vr.launch.py \
  load_gripper:="${LOAD_GRIPPER:-true}" use_rviz:="${USE_RVIZ:-true}"
