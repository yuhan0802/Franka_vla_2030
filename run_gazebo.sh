#!/usr/bin/env bash
set -euo pipefail

# Gazebo Sim runs a physics simulation entirely in the container.  The launch
# also starts MoveIt so RViz can plan and execute a dragged end-effector goal.
# It never connects to the FR3 FCI and does not require ROBOT_IP.
docker compose run --rm franka_vr \
  bash -lc '
    set -e
    cd /ws_franka_vr
    source /opt/ros/humble/setup.bash
    # Source files are bind-mounted while the build/install trees live in the
    # image. Force CMake to reconfigure so newly installed RViz and launch
    # assets (including the MoveIt end-effector marker configuration) are not
    # silently shadowed by the stale install manifest in the image.
    colcon build --symlink-install --cmake-force-configure --packages-select \
      franka_vr franka_gazebo_bringup franka_fr3_moveit_config
    source /ws_franka_vr/install/setup.bash
    ros2 launch franka_vr gazebo_vr.launch.py \
      load_gripper:="'"${LOAD_GRIPPER:-true}"'" use_rviz:="'"${USE_RVIZ:-true}"'"
  '
