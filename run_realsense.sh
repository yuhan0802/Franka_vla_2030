#!/usr/bin/env bash
set -euo pipefail

# Start the directly connected Intel RealSense D435i.  The compose service is
# privileged and mounts /dev, so the camera's V4L2 and HID devices are visible
# inside this short-lived ROS container.
#
# Override CAMERA_NAME when another ROS camera namespace is required, and
# REALSENSE_SERIAL when more than one RealSense device is connected.
CAMERA_NAME="${CAMERA_NAME:-camera}"
REALSENSE_SERIAL="${REALSENSE_SERIAL:-}"

serial_arg=()
if [[ -n "${REALSENSE_SERIAL}" ]]; then
  serial_arg=(serial_no:="${REALSENSE_SERIAL}")
fi

docker compose run --rm franka_vr bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  exec ros2 launch realsense2_camera rs_launch.py \
    camera_name:="'"${CAMERA_NAME}"'" \
    enable_color:=true enable_depth:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 \
    align_depth.enable:=true pointcloud.enable:=true \
    '"${serial_arg[*]}"'
'
