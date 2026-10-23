#!/usr/bin/env bash
# End-effector pose control option for 15x7 EEF policies (see
# franka_eef_pose_control_adapter.md). Independent of the pi0.5 direct joint
# velocity workflow: it uses the MoveIt Servo pose stack (run_robot.sh,
# /set_target_pose) and its own tmux sessions and log directory.
#
#   dry-run    policy + cameras + robot state; no target is sent
#   axis-test  one hand-built small motion along EEF_TEST_AXIS; no policy
#   home       move to the Task0 training start pose (one chunk, <=4 mm/step); no policy
#   execute    serial 15-step chunks on the real robot
#   probe      robot stack + cameras; record images and every gripper/arm state
#              source without commanding the arm (EEF_GRIPPER_TEST=I_UNDERSTAND_GRIPPER_MOTION
#              adds an open/close gripper cycle, EEF_GRIPPER_HOMING=true homes first)
#   status | stop
#   server | server-status | server-stop   EEF model server on the GPU host (ssh)
set -euo pipefail

mode="${1:-dry-run}"
case "$mode" in dry-run|axis-test|home|execute|probe|status|stop|server|server-status|server-stop) ;; *)
  echo "Usage: $0 {dry-run|axis-test|home|execute|probe|status|stop|server|server-status|server-stop}" >&2; exit 2;; esac
# home is an axis-test whose single chunk goes to the training start pose.
[[ "$mode" == home ]] && { mode=axis-test; export EEF_TEST_AXIS=home EEF_ROTATION_FRAME=tool; }
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"
robot_session=franka_vr_eef_robot
camera_session=franka_vr_eef_cameras
host="${EEF_SERVER_HOST:-10.24.116.20}"; port="${EEF_SERVER_PORT:-8095}"
# The Task0 EEF checkpoint lives on the 20 machine's /data2; the server runs there.
remote="${EEF_REMOTE_SSH:-mac_20}"
remote_project="${EEF_REMOTE_PROJECT:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA}"
checkpoint="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar/Qwen3VL2B+Franka-Task0+lr1e-05+droid_pretrained_bar_eef01/save_6000_chkpt}"
actioncodec="${EEF_ACTIONCODEC:-}"
state_stats="${EEF_STATE_STATS:-}"
server_gpu="${EEF_SERVER_GPU:-1}"
remote_session="franka_eef_server_$port"
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

remote_listening() { ssh "$remote" "ss -ltn | grep -q ':$port '"; }
start_server() {
  if remote_listening; then
    echo "EEF server already listening on $host:$port; the adapter verifies its checkpoint."; return 0
  fi
  local script; script="$(mktemp)"
  cat > "$script" <<EOF
#!/usr/bin/env bash
set -euo pipefail
P='$remote_project'
PY="\$P/.conda/qwenvla/bin/python"; [[ -x "\$PY" ]] || PY=/home/xmu/anaconda3/envs/qwenvla/bin/python
cd ~/franka_eef_server
export CUDA_VISIBLE_DEVICES='$server_gpu' PYTHONPATH="\$P/third_party/python:\$P" VLA_PROJECT_ROOT="\$P"
export HF_HOME="\$P/actioncodec_libero/.cache/huggingface" USE_TF=0 TRANSFORMERS_NO_TF=1 TF_CPP_MIN_LOG_LEVEL=3 TOKENIZERS_PARALLELISM=false
"\$PY" -u deploy_franka_task0_qwen_actioncodec_eef.py --checkpoint '$checkpoint' --actioncodec '$actioncodec' --state-stats '$state_stats' --port '$port' 2>&1 | tee -a server_$port.log
EOF
  ssh "$remote" "mkdir -p ~/franka_eef_server"
  scp -q "$root_dir/deploy_franka_task0_qwen_actioncodec_eef.py" "$remote:franka_eef_server/"
  scp -q "$script" "$remote:franka_eef_server/run_server_$port.sh"; rm -f "$script"
  ssh "$remote" "tmux kill-session -t '$remote_session' 2>/dev/null || true; tmux new-session -d -s '$remote_session' 'bash ~/franka_eef_server/run_server_$port.sh'"
  echo "Starting EEF server on $remote (GPU $server_gpu, port $port)..."
  for _ in $(seq 1 120); do
    if remote_listening; then echo "EEF server ready at $host:$port."; return 0; fi
    ssh "$remote" "tmux has-session -t '$remote_session' 2>/dev/null" || break
    sleep 2
  done
  echo "EEF server did not start; inspect: ssh $remote 'tail -50 ~/franka_eef_server/server_$port.log'" >&2
  return 1
}

case "$mode" in
  server) start_server; exit $?;;
  server-status)
    if remote_listening; then echo "EEF server $host:$port: listening"; else echo "EEF server $host:$port: stopped"; fi
    exit 0;;
  server-stop)
    ssh "$remote" "tmux kill-session -t '$remote_session' 2>/dev/null || true"; echo "EEF server session stopped."
    exit 0;;
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
  start_session "$robot_session" "set -o pipefail; ROBOT_IP='${ROBOT_IP:-172.16.0.2}' LOAD_GRIPPER=true USE_RVIZ=false ./run_robot.sh 2>&1 | tee -a '$log_dir/robot_live.log'; status=\${PIPESTATUS[0]}; echo ROBOT_RUNNER_EXIT=\$status | tee -a '$log_dir/robot_live.log'; exit \$status"
fi
if [[ "$mode" != axis-test ]]; then
  if [[ "$commands" == *vla_cameras.launch.py* ]]; then
    echo "Reusing running VLA cameras; make sure they publish $camera_mode images."
  else
    start_session "$camera_session" "set -o pipefail; VLA_CAMERA_MODE='$camera_mode' ./run_vla_cameras.sh 2>&1 | tee -a '$log_dir/cameras_live.log'; status=\${PIPESTATUS[0]}; echo CAMERA_RUNNER_EXIT=\$status | tee -a '$log_dir/cameras_live.log'; exit \$status"
  fi
fi

# Keep the robot/camera console output next to the run's own logs.
capture_sessions() {
  local name
  for name in "$robot_session" "$camera_session"; do
    tmux has-session -t "$name" 2>/dev/null && tmux capture-pane -p -J -S - -t "$name" > "$log_dir/${name#franka_vr_eef_}_tmux.log" || true
  done
}

if [[ "$mode" == probe ]]; then
  probe_flags=(--out-dir /eef_logs --exterior-image-topic "/camera_${exterior_camera}/$suffix"
    --duration "${EEF_PROBE_DURATION:-20}" --image-period "${EEF_PROBE_IMAGE_PERIOD:-0.5}")
  if [[ "${EEF_GRIPPER_TEST:-}" == I_UNDERSTAND_GRIPPER_MOTION ]]; then
    probe_flags+=(--gripper-cycle --confirm-gripper GRIPPER_MOTION)
    [[ "${EEF_GRIPPER_HOMING:-false}" == true ]] && probe_flags+=(--homing)
  fi
  printf -v quoted_flags '%q ' "${probe_flags[@]}"
  echo "EEF STATE PROBE: camera=$exterior_camera/$camera_mode logs=$log_dir (the arm is never commanded)"
  set +e
  docker compose run --rm -v "$log_dir:/eef_logs" franka_vr bash -lc '
    set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
    exec python3 src/franka_vr/franka_vr/scripts/eef_state_probe.py '"$quoted_flags"'
  ' 2>&1 | tee "$log_dir/probe.log"
  status=${PIPESTATUS[0]}
  set -e
  capture_sessions
  echo "Probe exited with $status. Pack the logs with: tar -czf $(basename "$log_dir").tar.gz -C $(dirname "$log_dir") $(basename "$log_dir")"
  exit "$status"
fi

adapter_flags=(--server-host "$host" --server-port "$port" --instruction "$instruction"
  --exterior-image-topic "/camera_${exterior_camera}/$suffix"
  --bounds-config /ws_franka_vr/src/franka_vr/franka_vr/config/vla_calibration.yaml
  --chunk-start "${EEF_CHUNK_START:-measured}"
  --gripper-state-source "${EEF_GRIPPER_STATE_SOURCE:-measured}"
  --max-translation "${EEF_MAX_TRANSLATION:-0.006}" --max-rotation "${EEF_MAX_ROTATION:-0.035}"
  --max-tf-age "${EEF_MAX_TF_AGE:-0.15}"
  --startup-timeout "${EEF_STARTUP_TIMEOUT:-180}" --jsonl-log /eef_logs/policy.jsonl)
[[ -n "${EEF_ROTATION_FRAME:-}" ]] && adapter_flags+=(--rotation-frame "$EEF_ROTATION_FRAME")
[[ "$mode" != axis-test && -n "$checkpoint" ]] && adapter_flags+=(--expected-checkpoint "$checkpoint")
# Save every image pair sent to the policy (EEF_SAVE_IMAGES=false to skip).
[[ "$mode" != axis-test && "${EEF_SAVE_IMAGES:-true}" == true ]] && adapter_flags+=(--save-observation-dir /eef_logs/observations)

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
capture_sessions
echo "Adapter exited with $status. The Servo stack keeps holding; stop it with: $0 stop. Logs: $log_dir"
exit "$status"
