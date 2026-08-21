#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f /ws_franka_vr/install/setup.bash ]]; then
  source /ws_franka_vr/install/setup.bash
fi
exec "$@"
