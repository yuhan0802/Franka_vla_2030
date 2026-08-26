#!/usr/bin/env bash
set -euo pipefail

# Run this in a second terminal after ./run_gazebo.sh.  It joins the existing
# simulation container so its keyboard commands use the same ROS graph.
mapfile -t manual_containers < <(
  docker ps --format '{{.ID}} {{.Names}}' |
    awk '$2 ~ /^franka_vr_docker-franka_vr-run-/ { print $1 }'
)

if (( ${#manual_containers[@]} != 1 )); then
  echo "Expected exactly one running Gazebo container; found ${#manual_containers[@]}." >&2
  echo "Start ./run_gazebo.sh first, and stop any duplicate run containers." >&2
  exit 1
fi

exec docker exec -it "${manual_containers[0]}" bash -lc '
  source /opt/ros/humble/setup.bash
  source /ws_franka_vr/install/setup.bash
  ros2 run franka_vr keyboard_manual_control.py --ros-args -p use_sim_time:=true
'
