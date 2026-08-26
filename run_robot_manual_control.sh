#!/usr/bin/env bash
set -euo pipefail

# Run this in a second terminal after ./run_robot.sh.  This command never
# starts a new ros2_control hardware instance: it only joins the already
# running real-robot Servo stack and sends its bounded MoveIt Servo targets.
# Set ROBOT_CONTAINER explicitly if automatic discovery is not appropriate.
robot_container="${ROBOT_CONTAINER:-}"

if [[ -z "$robot_container" ]]; then
  mapfile -t robot_containers < <(
    while read -r container_id container_name; do
      command_line="$(docker inspect --format '{{range .Config.Cmd}}{{printf "%s " .}}{{end}}' "$container_id")"
      if [[ "$command_line" == *"franka_twist.launch.py"* ]]; then
        printf '%s\n' "$container_id"
      fi
    done < <(docker ps --format '{{.ID}} {{.Names}}')
  )

  if (( ${#robot_containers[@]} != 1 )); then
    echo "Expected exactly one running ./run_robot.sh container; found ${#robot_containers[@]}." >&2
    echo "Start ./run_robot.sh first, stop duplicate robot stacks, or set ROBOT_CONTAINER." >&2
    exit 1
  fi
  robot_container="${robot_containers[0]}"
fi

if ! docker inspect "$robot_container" >/dev/null 2>&1; then
  echo "ROBOT_CONTAINER '$robot_container' does not identify a running container." >&2
  exit 1
fi

echo "Connecting keyboard control to real-robot Servo container: $robot_container"
echo "Keep the emergency stop available and verify the workspace is clear."

exec docker exec -it "$robot_container" bash -lc '
  source /opt/ros/humble/setup.bash
  source /ws_franka_vr/install/setup.bash
  ros2 run franka_vr keyboard_manual_control.py --ros-args -p use_sim_time:=false
'
