#!/usr/bin/env bash
set -euo pipefail

# Start the Meta Quest bridge after a robot control stack (fake hardware,
# Gazebo, or real robot) is already running in another terminal.
docker compose run --rm franka_vr \
  /ws_franka_vr/src/franka_vr/franka_vr/oculus_reader/start_vr.sh
