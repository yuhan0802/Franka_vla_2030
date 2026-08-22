#!/usr/bin/env bash
set -euo pipefail

# Start Gazebo physics, MoveIt Servo, and the trajectory controller that the
# Quest bridge uses.  Run ./run_vr.sh in a second terminal afterwards.
docker compose run --rm franka_vr \
  bash -lc '
    set -e
    cd /ws_franka_vr
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select \
      franka_vr franka_gazebo_bringup franka_fr3_moveit_config
    source /ws_franka_vr/install/setup.bash
    ros2 launch franka_vr gazebo_vr.launch.py \
      load_gripper:=true use_rviz:="'"${USE_RVIZ:-true}"'"
  '
