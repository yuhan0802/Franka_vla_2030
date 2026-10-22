#!/usr/bin/env bash
# End-effector pose control option for 15x7 EEF policies (see
# franka_eef_pose_control_adapter.md). Independent of the pi0.5 direct joint
# velocity workflow: it uses the MoveIt Servo pose stack (run_robot.sh,
# /set_target_pose) and its own tmux sessions and log directory.
#
#   dry-run    policy + cameras + robot state; no target is sent
#   axis-test  one hand-built small motion along EEF_TEST_AXIS; no policy
#   execute    serial 15-step chunks on the real robot
#   status | stop
set -euo pipefail

mode="${1:-dry-run}"
case "$mode" in dry-run|axis-test|execute|status|stop) ;; *)
  echo "Usage: $0 {dry-run|axis-test|execute|status|stop}" >&2; exit 2;; esac
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"
robot_session=franka_vr_eef_robot
camera_session=franka_vr_eef_cameras
host="${EEF_SERVER_HOST:-10.24.116.66}"; port="${EEF_SERVER_PORT:-8095}"
instruction="${VLA_INSTRUCTION:-place the test tube rack in the black tray}"
# Training recorded /camera_left/image_rect, so default to calibrated images.
camera_mode="${VLA_CAMERA_MODE:-calibrated}"
case "$camera_mode" in raw) suffix=image_raw;; calibrated) suffix=image_rect;;
  *) echo "VLA_CAMERA_MODE must be raw or calibrated" >&2; exit 2;; esac
exterior_camera="${EEF_EXTERIOR_CAMERA:-left}"
case "$exterior_camera" in left|right) ;; *) echo "EEF_EXTERIOR_CAMERA must be left or right" >&2; exit 2;; esac

start_session() {
  local name="$1" command="$2"
  if tmux has-session -t "$name" 2>/dev/null; then echo "Reusing tmux: $name"
  else tmux new-session -d -s "$name" "cd '$root_dir' && $command"; echo "Started tmux: $name"; fi
}
stop_session() {
  local name="$1" attempt
  tmux has-session -t "$name" 2>/dev/null || return 0
  tmux send-keys -t "$name" C-c
  for attempt in $(seq 1 150); do tmux has-session -t "$name" 2>/dev/null || return 0; sleep .1; done
  echo "$name did not exit after SIGINT; left intact (no SIGKILL)" >&2; return 1
}
status_session() { if tmux has-session -t "$1" 2>/dev/null; then echo "$1: running"; else echo "$1: stopped"; fi; }
running_commands() {
  local id
  while read -r id; do
    [[ -n "$id" ]] && docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir" \
    --filter "label=com.docker.compose.oneoff=True")
}

case "$mode" in
  status)
    status_session "$robot_session"; status_session "$camera_session"
    echo "Latest logs:"; ls -dt "$root_dir"/logs/eef_pose_* 2>/dev/null | head -3 || true
    exit 0;;
  stop)
    failed=0; stop_session "$camera_session" || failed=1; stop_session "$robot_session" || failed=1
    exit "$failed";;
esac

commands="$(running_commands)"
if [[ "$commands" == *franka_pi05_direct_velocity.launch.py* || "$commands" == *franka_pi05_joint_velocity.launch.py* ]]; then
  echo "REFUSED: a pi0.5 joint/direct velocity stack owns the robot. Stop it first (./run_pi05_direct.sh stop)." >&2
  exit 1
fi

run_stamp="$(date +%Y%m%d_%H%M%S)_$$"; log_dir="${EEF_LOG_DIR:-$root_dir/logs/eef_pose_$run_stamp}"
mkdir -p "$log_dir"; log_dir="$(cd "$log_dir" && pwd)"

if [[ "$commands" == *franka_twist.launch.py* ]]; then
  echo "Reusing the running Servo pose stack (franka_twist.launch.py)."
else
  start_session "$robot_session" "ROBOT_IP='${ROBOT_IP:-172.16.0.2}' LOAD_GRIPPER=true USE_RVIZ=false ./run_robot.sh"
fi
if [[ "$mode" != axis-test ]]; then
  if [[ "$commands" == *vla_cameras.launch.py* ]]; then
    echo "Reusing running VLA cameras; make sure they publish $camera_mode images."
  else
    start_session "$camera_session" "VLA_CAMERA_MODE='$camera_mode' ./run_vla_cameras.sh"
  fi
fi

adapter_flags=(--server-host "$host" --server-port "$port" --instruction "$instruction"
  --exterior-image-topic "/camera_${exterior_camera}/$suffix"
  --bounds-config /ws_franka_vr/src/franka_vr/franka_vr/config/vla_calibration.yaml
  --chunk-start "${EEF_CHUNK_START:-measured}"
  --gripper-state-source "${EEF_GRIPPER_STATE_SOURCE:-measured}"
  --max-translation "${EEF_MAX_TRANSLATION:-0.006}" --max-rotation "${EEF_MAX_ROTATION:-0.035}"
  --startup-timeout "${EEF_STARTUP_TIMEOUT:-180}" --jsonl-log /eef_logs/policy.jsonl)
[[ -n "${EEF_ROTATION_FRAME:-}" ]] && adapter_flags+=(--rotation-frame "$EEF_ROTATION_FRAME")

case "$mode" in
  dry-run)
    adapter_flags+=(--max-chunks "${EEF_MAX_CHUNKS:-1}" --max-actions 2000);;
  axis-test|execute)
    [[ "${EEF_ENABLE_REAL_ROBOT:-}" == I_UNDERSTAND_EEF_POSE ]] || {
      echo "REFUSED: set EEF_ENABLE_REAL_ROBOT=I_UNDERSTAND_EEF_POSE only after the on-site safety check." >&2; exit 3; }
    adapter_flags+=(--execute --confirm-real-robot EEF_POSE_TARGET)
    if [[ "$mode" == axis-test ]]; then
      [[ -n "${EEF_TEST_AXIS:-}" && -n "${EEF_ROTATION_FRAME:-}" ]] || {
        echo "axis-test requires EEF_TEST_AXIS (+x..-rz) and EEF_ROTATION_FRAME (world|tool)" >&2; exit 2; }
      adapter_flags+=(--test-axis "$EEF_TEST_AXIS" --test-magnitude "${EEF_TEST_MAGNITUDE:-0.04}"
        --test-steps "${EEF_TEST_STEPS:-5}" --max-chunks 1 --max-actions 15)
    else
      adapter_flags+=(--max-chunks "${EEF_MAX_CHUNKS:-1}" --max-actions "${EEF_MAX_ACTIONS:-15}"
        --execute-horizon "${EEF_EXECUTE_HORIZON:-15}")
      [[ "${EEF_ENABLE_GRIPPER:-false}" == true ]] && adapter_flags+=(--enable-gripper)
    fi;;
esac

printf -v quoted_flags '%q ' "${adapter_flags[@]}"
echo "EEF POSE $mode: server=$host:$port camera=$exterior_camera/$camera_mode logs=$log_dir"
set +e
docker compose run --rm -v "$log_dir:/eef_logs" franka_vr bash -lc '
  set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
  exec python3 src/franka_vr/franka_vr/scripts/eef_pose_policy_adapter.py '"$quoted_flags"'
' 2>&1 | tee "$log_dir/adapter.log"
status=${PIPESTATUS[0]}
set -e
echo "Adapter exited with $status. The Servo stack keeps holding; stop it with: $0 stop. Logs: $log_dir"
exit "$status"
