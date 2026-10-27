#!/usr/bin/env bash
# End-effector pose policies (15x7 EEF) WITHOUT MoveIt Servo. Same policy adapter
# logic as run_eef_pose.sh, but targets are tracked by eef_pose_direct_adapter.py
# (Servo-equivalent P tracker + DLS IK, 100 Hz) and executed by the pi0.5 direct
# joint velocity stack (run_robot_pi05_direct.sh, with gripper).
#
#   dry-run    policy + cameras + robot state; no JointJog publisher
#   axis-test  one hand-built small motion along EEF_TEST_AXIS; no policy
#   home       move to the Task0 training start pose (one chunk, <=4 mm/step); no policy
#   execute    serial 15-step chunks on the real robot
#   status | stop
#   server | server-status | server-stop   delegated to run_eef_pose.sh (same model server)
#
# The robot stack is started ARMED for axis-test/home/execute and for dry-run when
# EEF_DIRECT_STACK=armed (so a following execute reuses it); otherwise disarmed.
# Arming requires EEF_DIRECT_ENABLE_REAL_ROBOT=I_UNDERSTAND_DIRECT_EEF.
set -euo pipefail

mode="${1:-dry-run}"
case "$mode" in dry-run|axis-test|home|execute|status|stop|server|server-status|server-stop) ;; *)
  echo "Usage: $0 {dry-run|axis-test|home|execute|status|stop|server|server-status|server-stop}" >&2; exit 2;; esac
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$root_dir/scripts/runtime_log_root.sh"
cd "$root_dir"
case "$mode" in server|server-status|server-stop) exec "$root_dir/run_eef_pose.sh" "$mode";; esac
[[ "$mode" == home ]] && { mode=axis-test; export EEF_TEST_AXIS=home EEF_ROTATION_FRAME=tool; }

robot_session=franka_vr_eef_direct_robot
camera_session=franka_vr_eef_direct_cameras
stack_record="$FRANKA_LOG_ROOT/.eef_direct_stack_log_dir"
host="${EEF_SERVER_HOST:-10.24.116.20}"; port="${EEF_SERVER_PORT:-8095}"
checkpoint="${EEF_CHECKPOINT:-}"
actioncodec="${EEF_ACTIONCODEC:-}"
state_stats="${EEF_STATE_STATS:-}"
instruction="${VLA_INSTRUCTION:-place the test tube rack in the black tray}"
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
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
}
direct_containers() {
  local id
  while read -r id; do
    [[ -n "$id" ]] || continue
    [[ "$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null)" == *franka_pi05_direct_velocity.launch.py* ]] && echo "$id"
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
}
request_zero_velocity() {
  local id
  for id in $(direct_containers); do
    echo "Requesting zero joint velocity in direct launch container: $id"
    timeout --signal=TERM --kill-after=2s 5s docker exec "$id" bash -lc '
      source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
      ros2 topic pub -r 15 --times 4 /pi05/direct_joint_velocity_cmd control_msgs/msg/JointJog \
        "{joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration: 0.0666667}"
    ' >/dev/null 2>&1 || true
  done
}

case "$mode" in
  status)
    status_session "$robot_session"; status_session "$camera_session"
    [[ -f "$stack_record" ]] && echo "Robot stack log: $(cat "$stack_record")/robot.log"
    echo "Latest logs:"; ls -dt "$FRANKA_LOG_ROOT"/eef_direct_* 2>/dev/null | head -3 || true
    exit 0;;
  stop)
    failed=0; request_zero_velocity
    stop_session "$camera_session" || failed=1; stop_session "$robot_session" || failed=1
    for id in $(direct_containers); do echo "Forwarding SIGINT to direct launch container: $id"; docker kill --signal=SIGINT "$id" >/dev/null || true; done
    rm -f "$stack_record"
    exit "$failed";;
esac

commands="$(running_commands)"
if [[ "$commands" == *franka_twist.launch.py* ]]; then
  echo "REFUSED: the MoveIt Servo stack (franka_twist.launch.py) owns the robot. Stop it first (./run_eef_pose.sh stop)." >&2
  exit 1
fi
if [[ "$commands" == *franka_pi05_joint_velocity.launch.py* ]]; then
  echo "REFUSED: the pi0.5 joint velocity stack owns the robot. Stop it first." >&2; exit 1
fi

stack_mode=disarmed
[[ "$mode" != dry-run || "${EEF_DIRECT_STACK:-disarmed}" == armed ]] && stack_mode=armed
if [[ "$stack_mode" == armed && "${EEF_DIRECT_ENABLE_REAL_ROBOT:-}" != I_UNDERSTAND_DIRECT_EEF ]]; then
  echo "REFUSED: arming the direct stack requires EEF_DIRECT_ENABLE_REAL_ROBOT=I_UNDERSTAND_DIRECT_EEF (on-site safety check)." >&2
  exit 3
fi

run_stamp="$(date +%Y%m%d_%H%M%S)_$$"; log_dir="${EEF_LOG_DIR:-$FRANKA_LOG_ROOT/eef_direct_$run_stamp}"
mkdir -p "$log_dir"; log_dir="$(cd "$log_dir" && pwd)"

if tmux has-session -t "$robot_session" 2>/dev/null; then
  stack_log="$(cat "$stack_record" 2>/dev/null || true)/robot.log"
  [[ -f "$stack_log" ]] || { echo "REFUSED: $robot_session is running but its log is unknown; run: $0 stop" >&2; exit 1; }
  if [[ "$stack_mode" == armed ]] && ! grep -q 'direct executor is ARMED' "$stack_log"; then
    echo "REFUSED: the running direct stack is DISARMED; run '$0 stop' and start again armed." >&2; exit 1
  fi
  echo "Reusing direct robot stack (log: $stack_log)"
else
  mkdir -p "$(dirname "$stack_record")"; echo "$log_dir" > "$stack_record"; stack_log="$log_dir/robot.log"
  start_session "$robot_session" "PI05_DIRECT_LOG_DIR='$log_dir' PI05_DIRECT_LOAD_GRIPPER=true \
PI05_DIRECT_MAX_VELOCITY='${EEF_DIRECT_EXECUTOR_MAX_VELOCITY:-0.6}' \
PI05_DIRECT_MAX_ACCELERATION='${EEF_DIRECT_EXECUTOR_MAX_ACCELERATION:-2.0}' ./run_robot_pi05_direct.sh '$stack_mode'"
fi
if [[ "$mode" != axis-test ]]; then
  if [[ "$commands" == *vla_cameras.launch.py* ]]; then
    echo "Reusing running VLA cameras; make sure they publish $camera_mode images."
  else
    start_session "$camera_session" "VLA_CAMERA_MODE='$camera_mode' VLA_EXTERIOR_CAMERA='$exterior_camera' ./run_vla_cameras.sh"
  fi
fi

echo "Waiting for the direct robot stack ($stack_mode) to become ready (up to 120 seconds)..."
ready=false
for _ in $(seq 1 120); do
  tmux has-session -t "$robot_session" 2>/dev/null || { echo "Direct robot stack exited; inspect $stack_log" >&2; exit 1; }
  if [[ -f "$stack_log" ]]; then
    grep -q 'FAULT LATCHED' "$stack_log" && { echo "Direct executor faulted; inspect $stack_log (then: $0 stop)" >&2; exit 1; }
    if [[ "$stack_mode" == armed ]]; then
      grep -q 'armed dependencies ready' "$stack_log" &&
        grep -q 'Configured and activated.*fr3_direct_velocity_controller' "$stack_log" && { ready=true; break; }
    else
      grep -q 'direct executor is DISARMED' "$stack_log" &&
        grep -q 'Configured and activated.*joint_state_broadcaster' "$stack_log" && { ready=true; break; }
    fi
  fi
  sleep 1
done
[[ "$ready" == true ]] || { echo "Direct robot startup timed out; adapter not started. Inspect $stack_log" >&2; exit 1; }

capture_sessions() {
  local name
  for name in "$robot_session" "$camera_session"; do
    tmux has-session -t "$name" 2>/dev/null && tmux capture-pane -p -J -S - -t "$name" > "$log_dir/${name#franka_vr_eef_direct_}_tmux.log" || true
  done
}

adapter_flags=(--server-host "$host" --server-port "$port" --instruction "$instruction"
  --exterior-image-topic "/camera_${exterior_camera}/$suffix"
  --bounds-config "${EEF_BOUNDS_CONFIG:-/ws_franka_vr/src/franka_vr/franka_vr/config/eef_workspace_bounds.yaml}"
  --chunk-start "${EEF_CHUNK_START:-measured}"
  --gripper-state-source "${EEF_GRIPPER_STATE_SOURCE:-measured}"
  --max-translation "${EEF_MAX_TRANSLATION:-0.006}" --max-rotation "${EEF_MAX_ROTATION:-0.035}"
  --max-tf-age "${EEF_MAX_TF_AGE:-0.15}"
  --startup-timeout "${EEF_STARTUP_TIMEOUT:-180}" --jsonl-log /eef_logs/policy.jsonl
  --max-joint-velocity "${EEF_DIRECT_MAX_JOINT_VELOCITY:-0.5}"
  --max-joint-acceleration "${EEF_DIRECT_MAX_JOINT_ACCELERATION:-1.8}")
[[ -n "${EEF_ROTATION_FRAME:-}" ]] && adapter_flags+=(--rotation-frame "$EEF_ROTATION_FRAME")
if [[ -n "${EEF_HOME_POSITION:-}" ]]; then
  read -r -a home_position <<< "$EEF_HOME_POSITION"
  adapter_flags+=(--home-position "${home_position[@]}")
fi
if [[ -n "${EEF_HOME_QUATERNION:-}" ]]; then
  read -r -a home_quaternion <<< "$EEF_HOME_QUATERNION"
  adapter_flags+=(--home-quaternion "${home_quaternion[@]}")
fi
if [[ -n "${EEF_ACTION_GAIN:-}" ]]; then
  read -r -a action_gain <<< "$EEF_ACTION_GAIN"
  adapter_flags+=(--action-gain "${action_gain[@]}")
fi
if [[ "$mode" != axis-test ]]; then
  [[ -n "${EEF_EXPECTED_ACTION_NORMALIZATION:-}" ]] && adapter_flags+=(--expected-action-normalization "$EEF_EXPECTED_ACTION_NORMALIZATION")
  [[ "${EEF_EXPECTED_ACTION_MULAW_MU:-0}" != 0 ]] && adapter_flags+=(--expected-action-mulaw-mu "$EEF_EXPECTED_ACTION_MULAW_MU")
  [[ -n "$checkpoint" ]] && adapter_flags+=(--expected-checkpoint "$checkpoint")
  [[ -n "$actioncodec" ]] && adapter_flags+=(--expected-actioncodec "$actioncodec")
  [[ -n "$state_stats" ]] && adapter_flags+=(--expected-state-stats "$state_stats")
  [[ -n "${EEF_MODEL_IMAGE_SIZE:-}" ]] && adapter_flags+=(--expected-model-image-size "$EEF_MODEL_IMAGE_SIZE")
  [[ "${EEF_SAVE_IMAGES:-true}" == true ]] && adapter_flags+=(--save-observation-dir /eef_logs/observations)
fi

case "$mode" in
  dry-run)
    adapter_flags+=(--max-chunks "${EEF_MAX_CHUNKS:-1}" --max-actions 2000);;
  axis-test|execute)
    adapter_flags+=(--execute --confirm-real-robot EEF_POSE_TARGET --confirm-direct DIRECT_EEF_VELOCITY)
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
echo "EEF DIRECT $mode (no MoveIt Servo): server=$host:$port camera=$exterior_camera/$camera_mode logs=$log_dir"
set +e
docker compose run --rm -v "$log_dir:/eef_logs" franka_vr bash -lc '
  set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
  exec python3 src/franka_vr/franka_vr/scripts/eef_pose_direct_adapter.py '"$quoted_flags"'
' 2>&1 | tee "$log_dir/adapter.log"
status=${PIPESTATUS[0]}
set -e
capture_sessions
[[ -f "$stack_log" ]] && cp "$stack_log" "$log_dir/robot_stack.log" 2>/dev/null || true
if grep -q 'FAULT LATCHED' "$stack_log" 2>/dev/null; then
  echo "Direct executor latched a fault; it parks at zero velocity. Restart with: $0 stop" >&2
fi
echo "Adapter exited with $status. The direct stack stays at zero velocity; stop it with: $0 stop. Logs: $log_dir"
exit "$status"
