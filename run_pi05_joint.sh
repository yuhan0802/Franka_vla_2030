#!/usr/bin/env bash
# Independent pi0.5-DROID JointJog workflow. Does not invoke the EE-pose bridge.
set -euo pipefail

mode="${1:-dry-run}"
case "$mode" in
  server|probe|dry-run|execute|status|stop) ;;
  *) echo "Usage: $0 {server|probe|dry-run|execute|status|stop}" >&2; exit 2 ;;
esac

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_session="franka_vr_pi05_joint_robot"
camera_session="franka_vr_pi05_joint_cameras"
instruction="${VLA_INSTRUCTION:-put the red test tube into the red basket}"
server_host="${VLA_REMOTE_HOST:-10.24.116.66}"
server_port="${VLA_SERVER_PORT:-8094}"
max_velocity="${PI05_MAX_JOINT_VELOCITY:-0.15}"
max_acceleration="${PI05_MAX_JOINT_ACCELERATION:-0.5}"
max_actions="${PI05_MAX_ACTIONS:-16}"
gripper_force="${PI05_GRIPPER_FORCE:-10.0}"
camera_mode="${VLA_CAMERA_MODE:-calibrated}"
case "$camera_mode" in
  raw) image_suffix=image_raw ;;
  calibrated) image_suffix=image_rect ;;
  *) echo "VLA_CAMERA_MODE must be raw or calibrated" >&2; exit 2 ;;
esac

start_session() {
  local session="$1" command="$2"
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "Reusing tmux session: $session"
  else
    tmux new-session -d -s "$session" -e "VLA_CAMERA_MODE=$camera_mode" "cd '$root_dir' && VLA_CAMERA_MODE='$camera_mode' exec $command"
    echo "Started tmux session: $session"
  fi
}

session_status() {
  local session="$1"
  printf '%s: ' "$session"
  if tmux has-session -t "$session" 2>/dev/null; then echo running; else echo stopped; fi
}

stop_session() {
  local session="$1" attempt
  tmux has-session -t "$session" 2>/dev/null || return 0
  echo "Sending SIGINT through tmux: $session"
  tmux send-keys -t "$session" C-c
  for attempt in $(seq 1 150); do
    tmux has-session -t "$session" 2>/dev/null || return 0
    sleep 0.1
  done
  echo "$session is still shutting down; leaving it intact (no SIGKILL/docker stop)." >&2
  return 1
}

run_adapter() {
  local adapter_mode="$1" enable_gripper="" allow_unconfirmed_grasp=""
  [[ "${PI05_ENABLE_GRIPPER:-false}" == true ]] && enable_gripper="--enable-gripper"
  [[ "${PI05_ALLOW_UNCONFIRMED_GRASP:-false}" == true ]] && allow_unconfirmed_grasp="--allow-unconfirmed-grasp"
  docker compose run --rm \
    -e "VLA_INSTRUCTION=$instruction" \
    -e "VLA_REMOTE_HOST=$server_host" \
    -e "VLA_SERVER_PORT=$server_port" \
    -e "PI05_MAX_JOINT_VELOCITY=$max_velocity" \
    -e "PI05_MAX_JOINT_ACCELERATION=$max_acceleration" \
    -e "PI05_MAX_ACTIONS=$max_actions" \
    -e "PI05_GRIPPER_FORCE=$gripper_force" \
    -e "PI05_ADAPTER_MODE=$adapter_mode" \
    -e "PI05_GRIPPER_FLAG=$enable_gripper" \
    -e "PI05_UNCONFIRMED_GRASP_FLAG=$allow_unconfirmed_grasp" \
    -e "PI05_IMAGE_SUFFIX=$image_suffix" \
    franka_vr bash -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      cd /ws_franka_vr
      source install/setup.bash
      exec python3 /ws_franka_vr/src/franka_vr/franka_vr/scripts/pi05_joint_velocity_adapter.py \
        --server-host "$VLA_REMOTE_HOST" --server-port "$VLA_SERVER_PORT" \
        --instruction "$VLA_INSTRUCTION" \
        --left-image-topic "/camera_left/$PI05_IMAGE_SUFFIX" \
        --right-image-topic "/camera_right/$PI05_IMAGE_SUFFIX" \
        --max-joint-velocity "$PI05_MAX_JOINT_VELOCITY" \
        --max-joint-acceleration "$PI05_MAX_JOINT_ACCELERATION" \
        --max-actions "$PI05_MAX_ACTIONS" \
        --gripper-force "$PI05_GRIPPER_FORCE" \
        $PI05_GRIPPER_FLAG $PI05_UNCONFIRMED_GRASP_FLAG $PI05_ADAPTER_MODE
    '
}

case "$mode" in
  server|probe)
    exec "$root_dir/run_vla_pi05_droid_66.sh" "$mode"
    ;;
  status)
    "$root_dir/run_vla_pi05_droid_66.sh" status || true
    session_status "$robot_session"
    session_status "$camera_session"
    exit 0
    ;;
  stop)
    failed=0
    stop_session "$robot_session" || failed=1
    stop_session "$camera_session" || failed=1
    "$root_dir/run_vla_pi05_droid_66.sh" stop || failed=1
    exit "$failed"
    ;;
esac

if [[ "$mode" == execute && "${PI05_ENABLE_REAL_ROBOT_EXECUTION:-}" != "I_UNDERSTAND" ]]; then
  echo "REFUSED before robot/camera startup: real JointJog execution requires:" >&2
  echo "  PI05_ENABLE_REAL_ROBOT_EXECUTION=I_UNDERSTAND" >&2
  echo "Run dry-run, simulation, replay, and unloaded low-speed commissioning first." >&2
  exit 3
fi

if tmux has-session -t "$camera_session" 2>/dev/null; then
  active_camera_mode="$(tmux show-environment -t "$camera_session" VLA_CAMERA_MODE 2>/dev/null || true)"
  if [[ "$active_camera_mode" != "VLA_CAMERA_MODE=$camera_mode" ]]; then
    echo "Camera session mode is unknown or differs from $camera_mode. Stop the camera session before retrying." >&2
    exit 1
  fi
fi
echo "Camera mode: $camera_mode; left image: /camera_left/$image_suffix"
"$root_dir/run_vla_pi05_droid_66.sh" server
start_session "$robot_session" "./run_robot_pi05_joint.sh"
start_session "$camera_session" "./run_vla_cameras.sh"

if [[ "$mode" == dry-run ]]; then
  echo "dry-run does not send robot motion commands"
  run_adapter ""
  exit 0
fi

echo "REAL ROBOT commissioning mode: at most $max_actions actions, max ${max_velocity} rad/s."
run_adapter "--execute --confirm-real-robot JOINT_VELOCITY_PI05"
