#!/usr/bin/env bash
# Reuse the deployed models; never start, reload or stop a remote policy.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root_dir/scripts/runtime_log_root.sh" "${2:-run}"
cd "$root_dir"
task="${1:?task required}"; mode="${2:-run}"
case "$task" in
  1) port=8101; gpu=5; instruction='put the red test tube into the red basket' ;;
  2) port=8102; gpu=6; instruction='place the test tube rack in the black tray' ;;
  6) port=8106; gpu=7; instruction='open the lid' ;;
  *) echo "Unknown tk task: $task" >&2; exit 2 ;;
esac
case "$mode" in run|stop|STOP|status|probe) ;; *) echo "Usage: ./tk${task}.sh [run|stop|status|probe]" >&2; exit 2 ;; esac
live_dry_run="${TK_LIVE_DRY_RUN:-0}"
[[ "$live_dry_run" == 0 || "$live_dry_run" == 1 ]] || { echo 'TK_LIVE_DRY_RUN must be 0 or 1' >&2; exit 2; }
host=10.24.116.66
robot_session="franka_vr_tk${task}_robot"
camera_session="franka_vr_tk${task}_cameras"
ssh_args=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10
  -i /home/mac/.ssh/id_rsa_xmu_66 -p 1172 "xmu@$host")
monitor_pid=""

stop_local() {
  local id command source session attempt
  if [[ -n "$monitor_pid" ]]; then
    kill "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
    monitor_pid=""
  fi
  # Match only this tk task's direct robot containers by their log bind mount.
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}')"
    [[ "$command" == *franka_pi05_direct_velocity.launch.py* || "$command" == *tk-adapter* ]] || continue
    source="$(docker inspect "$id" --format '{{range .Mounts}}{{if eq .Destination "/pi05_logs"}}{{.Source}}{{end}}{{end}}')"
    [[ "$source" == "$FRANKA_LOG_ROOT/tk${task}_"* || "$source" == "$root_dir/logs/tk${task}_"* ]] || continue
    if [[ "$command" == *franka_pi05_direct_velocity.launch.py* ]]; then
      timeout --signal=TERM --kill-after=2s 5s docker exec "$id" bash -lc '
      source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
      ros2 topic pub -r 15 --times 4 /pi05/direct_joint_velocity_cmd control_msgs/msg/JointJog \
        "{joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], velocities: [0, 0, 0, 0, 0, 0, 0], duration: 0.0666667}"
      ' >/dev/null 2>&1 || true
    fi
    docker kill --signal=SIGINT "$id" >/dev/null || true
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
  for session in "$robot_session" "$camera_session"; do
    if tmux has-session -t "$session" 2>/dev/null; then
      tmux send-keys -t "$session" C-c
      for attempt in $(seq 1 150); do
        tmux has-session -t "$session" 2>/dev/null || break
        sleep .1
      done
      if tmux has-session -t "$session" 2>/dev/null; then
        echo "Local session is still shutting down: $session" >&2
        return 1
      fi
    fi
  done
  echo "Local tk${task} task stopped; resident model at $host:$port kept running."
}
if [[ "$mode" == stop || "$mode" == STOP ]]; then stop_local; exit; fi

mapfile -t selected < <(python3 scripts/select_vla_task.py --config "$root_dir/vla_instruction_weights.json" --instruction "$instruction")
mapfile -t runtime < <(python3 scripts/select_vla_task.py --config "$root_dir/vla_instruction_weights.json" --instruction 'place the test tube rack in the black tray')
[[ ${#selected[@]} == 7 && ${#runtime[@]} == 7 ]] || { echo 'Could not resolve task configuration' >&2; exit 2; }
checkpoint="${selected[0]}/${selected[1]}"
printf -v verify_command 'python3 - %q %q %q %q' "$port" "$checkpoint" "${selected[2]}" "$gpu"
ssh "${ssh_args[@]}" "$verify_command" < scripts/check_tk_policy.py
curl --fail --silent --show-error --max-time 5 "http://$host:$port/healthz" >/dev/null || {
  echo "Resident model is unavailable; no reload was attempted." >&2; exit 1; }
echo "tk${task}: $host:$port GPU=$gpu checkpoint=$checkpoint"
if [[ "$mode" == status ]]; then
  for session in "$robot_session" "$camera_session"; do
    if tmux has-session -t "$session" 2>/dev/null; then echo "$session: running"; else echo "$session: stopped"; fi
  done
  exit 0
fi
if [[ "$mode" == probe ]]; then
  printf -v probe_command 'cd %q && /data/vla/openpi-main/.venv/bin/python - --host 127.0.0.1 --port %q --instruction %q --timeout 30' \
    /mnt/82_store/lyh/VLA/openpi-main "$port" "$instruction"
  ssh "${ssh_args[@]}" "$probe_command" < scripts/pi05_remote_probe.py
  exit
fi

# The robot is shared by all tasks. A second tk invocation must not acquire it.
exec 9>"${TMPDIR:-/tmp}/franka_vr_tk_direct_${UID}.lock"
flock -n 9 || { echo 'Another tk task is running; stop it first.' >&2; exit 1; }
while read -r id; do
  [[ -n "$id" ]] || continue
  command="$(docker inspect "$id" --format '{{json .Config.Cmd}}')"
  case "$command" in
    *franka_pi05_direct_velocity.launch.py*|*franka_pi05_joint_velocity.launch.py*|*franka_twist.launch.py*)
      echo "An existing robot stack is running ($id); stop it before starting tk${task}." >&2; exit 1 ;;
  esac
done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
for session in "$robot_session" "$camera_session"; do
  if tmux has-session -t "$session" 2>/dev/null; then echo "Existing session: $session; stop it first." >&2; exit 1; fi
done
log_dir="$FRANKA_LOG_ROOT/tk${task}_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$log_dir"
python3 - "$log_dir/run.json" "$task" "$host" "$port" "$gpu" "$checkpoint" "$instruction" <<'PY'
import json, sys
path, task, host, port, gpu, checkpoint, instruction = sys.argv[1:]
with open(path, "w") as output:
    json.dump(dict(task=task, host=host, port=int(port), gpu=int(gpu), checkpoint=checkpoint,
                   instruction=instruction, resident_policy=True), output, indent=2)
PY
trap 'stop_local || true' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf -v camera_command 'cd %q && VLA_CAMERA_MODE=raw ./run_vla_cameras.sh >>%q 2>&1' "$root_dir" "$log_dir/cameras.log"
tmux new-session -d -s "$camera_session" "$camera_command"
echo 'Starting cameras and waiting for real frames before acquiring robot control...'
docker compose run --rm -T -v "$root_dir/scripts:/tk_scripts:ro" franka_vr bash -lc '
  set -eo pipefail; source /opt/ros/humble/setup.bash
  exec python3 /tk_scripts/wait_tk_cameras.py --timeout 60
' tk-camera-check 2>&1 | tee "$log_dir/camera_readiness.log"
tmux has-session -t "$camera_session" 2>/dev/null || { echo 'Camera stack exited.' >&2; exit 1; }
printf -v robot_command 'cd %q && PI05_DIRECT_LOG_DIR=%q PI05_DIRECT_LOAD_GRIPPER=true PI05_DIRECT_MAX_VELOCITY=%q PI05_DIRECT_MAX_ACCELERATION=%q ./run_robot_pi05_direct.sh armed' \
  "$root_dir" "$log_dir" "${runtime[4]}" "${runtime[5]}"
tmux new-session -d -s "$robot_session" "$robot_command"
echo "Waiting for robot stack (up to 180 seconds). Logs: $log_dir"
ready=false
for attempt in $(seq 1 180); do
  tmux has-session -t "$robot_session" 2>/dev/null || { echo "Robot stack exited; inspect $log_dir/robot.log" >&2; tail -n 40 "$log_dir/robot.log" >&2; exit 1; }
  if [[ -f "$log_dir/robot.log" ]]; then
    if grep -Eq 'FAULT LATCHED|process has died.*spawner|Could not successfully call service' "$log_dir/robot.log"; then
      echo 'Robot controller startup failed; first diagnostics:' >&2
      grep -n -E 'FAULT LATCHED|Failed getting a result|Could not successfully call service|process has died' "$log_dir/robot.log" >&2 || true
      exit 1
    fi
    if grep -q 'armed dependencies ready' "$log_dir/robot.log" &&
       grep -q 'Configured and activated.*fr3_direct_velocity_controller' "$log_dir/robot.log"; then ready=true; break; fi
  fi
  sleep 1
done
[[ "$ready" == true ]] || { echo 'Robot startup timed out.' >&2; exit 1; }
adapter_common=(--server-host "$host" --server-port "$port" --instruction "$instruction"
  --left-image-topic /camera_left/image_raw --execute-horizon 16 --max-policy-latency 10
  --max-observation-wait 10)
adapter_container="franka_tk${task}_adapter_$$"
run_adapter() {
  docker compose run --rm -T --name "$adapter_container" -v "$log_dir:/pi05_logs" franka_vr bash -lc '
    set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
    exec python3 src/franka_vr/franka_vr/scripts/pi05_direct_policy_adapter.py "$@"
  ' tk-adapter "${adapter_common[@]}" "$@"
}
watch_stack() {
  while true; do
    if ! tmux has-session -t "$robot_session" 2>/dev/null ||
       ! tmux has-session -t "$camera_session" 2>/dev/null ||
       grep -Eq 'FAULT LATCHED|communication_constraints_violation|process has died.*ros2_control_node' "$log_dir/robot.log"; then
      echo 'Robot/camera stack stopped or faulted; stopping adapter.' | tee "$log_dir/stack_fault.log" >&2
      tail -n 35 "$log_dir/robot.log" >&2
      # The adapter container may still be starting. Retry until cleanup stops
      # this monitor so a startup race cannot leave it waiting indefinitely.
      docker kill --signal=SIGINT "$adapter_container" >/dev/null 2>&1 || true
    fi
    sleep 1
  done
}
run_supervised_adapter() {
  local output_log="$1" adapter_status
  shift
  watch_stack & monitor_pid=$!
  set +e
  run_adapter "$@" 2>&1 | tee "$output_log"
  adapter_status=${PIPESTATUS[0]}
  kill "$monitor_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  monitor_pid=""
  set -e
  [[ ! -f "$log_dir/stack_fault.log" ]] || return 1
  return "$adapter_status"
}
if [[ "$live_dry_run" == 1 ]]; then
  echo 'Validating live observations against the resident model...'
  run_supervised_adapter "$log_dir/full_dry_run.log" --jsonl-log /pi05_logs/full_dry_run_policy.jsonl
  grep -q '"robot_command_sent": false' "$log_dir/full_dry_run_policy.jsonl" || { echo 'Dry-run validation missing.' >&2; exit 1; }
else
  echo 'tk mode: live dry-run skipped; starting execution adapter.'
fi
run_supervised_adapter "$log_dir/adapter.log" --execute --confirm-real-robot DIRECT_JOINT_VELOCITY_PI05 \
  --max-actions "${runtime[3]}" --enable-gripper --gripper-speed 0.02 --gripper-force "${runtime[6]}" \
  --gripper-open-width 0.07 --gripper-closed-width 0.01 \
  --gripper-epsilon-inner 0.01 --gripper-epsilon-outer 0.07 \
  --gripper-require-grasp-success 0 \
  --gripper-open-tolerance 0.005 --gripper-max-plausible-width 0.085 \
  --jsonl-log /pi05_logs/policy.jsonl
if grep -q 'FAULT LATCHED' "$log_dir/adapter.log" "$log_dir/robot.log"; then
  echo "Task faulted; inspect $log_dir" >&2; exit 1
fi
echo "tk${task} finished. Logs: $log_dir"
