#!/usr/bin/env bash
# Generic EEF task/model entry point. Fixed model identities come from JSON.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="${EEF_TASK_CONFIG:-$root_dir/config/eef_tasks.json}"
usage() {
  echo "Usage: $0 TASK_KEY MODEL_KEY [run|dry-run|axis-test AXIS|home|probe|stop|stop-only|status|show]"
  echo "       $0 list"
  echo "stop: close EEF resources, then run fci_init.sh (gripper homing + arm return)."
  echo "stop-only: close resources without homing or arm return."
}
case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  list) exec python3 "$root_dir/scripts/eef_task_config.py" list --config "$config" ;;
esac
if (( $# < 2 || $# > 4 )); then usage >&2; exit 2; fi
task_key="$1"; model_key="$2"; mode="${3:-run}"
case "$mode" in run|dry-run|home|probe|stop|STOP|stop-only|status|show|server|server-status) ;;
  axis-test)
    case "${4:-}" in +x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz) ;;
      *) echo "axis-test requires +x,-x,+y,-y,+z,-z,+rx,-rx,+ry,-ry,+rz,-rz" >&2; exit 2;;
    esac ;;
  *) usage >&2; exit 2 ;;
esac
if (( $# == 4 )) && [[ "$mode" != axis-test ]]; then usage >&2; exit 2; fi

temporary="$(mktemp -d "${TMPDIR:-/tmp}/eef_task.XXXXXXXX")"
trap 'rm -rf -- "$temporary"' EXIT
control_flags=()
[[ -n "${EEF_CONTROL:-}" ]] && control_flags=(--control "$EEF_CONTROL")
# Keep an explicit per-run horizon override.  The resolved profile supplies the
# default, but callers must be able to run 15-predict/N-execute experiments
# without editing the shared task registry.
requested_execute_horizon="${EEF_EXECUTE_HORIZON:-}"
python3 "$root_dir/scripts/eef_task_config.py" resolve --config "$config" \
  --task "$task_key" --model "$model_key" "${control_flags[@]}" \
  --output "$temporary/profile.json" --env-file "$temporary/env"
if [[ "$mode" == show ]]; then cat "$temporary/profile.json"; exit 0; fi
while IFS= read -r -d '' key && IFS= read -r -d '' value; do export "$key=$value"; done < "$temporary/env"
if [[ -n "$requested_execute_horizon" ]]; then
  [[ "$requested_execute_horizon" =~ ^[0-9]+$ ]] &&
    (( requested_execute_horizon >= 1 && requested_execute_horizon <= 15 )) || {
      echo "EEF_EXECUTE_HORIZON must be an integer in [1, 15]." >&2; exit 2; }
  export EEF_EXECUTE_HORIZON="$requested_execute_horizon"
fi
source "$root_dir/scripts/runtime_log_root.sh" "$mode"

case "$EEF_CONTROL" in
  direct) runner="$root_dir/run_eef_direct.sh"; enable_var=EEF_DIRECT_ENABLE_REAL_ROBOT; enable_token=I_UNDERSTAND_DIRECT_EEF ;;
  servo) runner="$root_dir/run_eef_pose.sh"; enable_var=EEF_ENABLE_REAL_ROBOT; enable_token=I_UNDERSTAND_EEF_POSE ;;
esac
commissioned="$FRANKA_LOG_ROOT/.eef_direct_commissioned"
log_prefix="eef_${task_key}_${model_key}"
stamp() { date +%Y%m%d_%H%M%S; }

eef_containers() {
  local id command
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true)"
    case "$command" in
      *eef_pose_direct_adapter.py*|*eef_pose_policy_adapter.py*|*franka_pi05_direct_velocity.launch.py*|*franka_twist.launch.py*) echo "$id" ;;
    esac
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
}
stop_eef() {
  local id command attempt remaining failed=0
  # Stop policy publishers before releasing their arm controller.
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true)"
    case "$command" in
      *eef_pose_direct_adapter.py*|*eef_pose_policy_adapter.py*) docker kill --signal=SIGINT "$id" >/dev/null || failed=1 ;;
    esac
  done < <(eef_containers)
  "$root_dir/run_eef_direct.sh" stop || failed=1
  "$root_dir/run_eef_pose.sh" stop || failed=1
  "$root_dir/run_eef_pose.sh" server-stop || failed=1
  for attempt in $(seq 1 100); do
    remaining="$(eef_containers)"
    if [[ -z "$remaining" ]]; then return "$failed"; fi
    sleep .2
  done
  echo "EEF containers are still running; FCI initialization will not start." >&2
  return 1
}

case "$mode" in
  stop|STOP) stop_eef; "$root_dir/fci_init.sh"; exit $? ;;
  stop-only) stop_eef; exit $? ;;
  status) "$runner" status; "$root_dir/run_eef_pose.sh" server-status; exit $? ;;
  server|server-status) "$runner" "$mode"; exit $? ;;
  axis-test)
    log="$FRANKA_LOG_ROOT/${log_prefix}_axis_${4}_$(stamp)_$$"
    env EEF_LOG_DIR="$log" "$enable_var=$enable_token" EEF_TEST_AXIS="$4" "$runner" axis-test
    python3 "$root_dir/scripts/eef_task_config.py" check-axis "$log/policy.jsonl"
    if [[ "$EEF_CONTROL" == direct ]]; then
      echo "$(date -Is) $4 $log" >> "$commissioned"
    fi
    echo "Axis test passed. Log: $log"; exit 0 ;;
  home)
    env EEF_LOG_DIR="$FRANKA_LOG_ROOT/${log_prefix}_home_$(stamp)_$$" "$enable_var=$enable_token" "$runner" home
    exit $? ;;
  probe) "$root_dir/run_eef_pose.sh" probe; exit $? ;;
esac

if [[ "$EEF_CONTROL" == direct && "$mode" == run && ! -s "$commissioned" && "${EEF_DIRECT_SKIP_COMMISSION:-}" != true ]]; then
  echo "REFUSED: the direct tracker has no successful axis-test record." >&2
  echo "Run: $0 $task_key $model_key axis-test +z" >&2
  exit 3
fi
"$runner" server
run_dir="${EEF_RUN_DIR:-$FRANKA_LOG_ROOT/${log_prefix}_$(stamp)_$$}"
mkdir -p "$run_dir"
cp "$temporary/profile.json" "$run_dir/selected_profile.json"
dry_env=()
if [[ "$EEF_CONTROL" == direct ]]; then
  stack_record="$FRANKA_LOG_ROOT/.eef_direct_stack_log_dir"
  if [[ "$mode" == run ]] && tmux has-session -t franka_vr_eef_direct_robot 2>/dev/null &&
      ! grep -q 'direct executor is ARMED' "$(cat "$stack_record" 2>/dev/null)/robot.log" 2>/dev/null; then
    "$runner" stop
  fi
  [[ "$mode" == run ]] && dry_env=(EEF_DIRECT_STACK=armed "$enable_var=$enable_token")
fi
env EEF_LOG_DIR="$run_dir/dry_run" EEF_MAX_CHUNKS=1 "${dry_env[@]}" "$runner" dry-run
python3 "$root_dir/scripts/eef_task_config.py" check-dry-run "$run_dir/dry_run/policy.jsonl"

set +e
python3 "$root_dir/scripts/eef_task_config.py" check-start "$run_dir/dry_run/policy.jsonl" \
  --profile "$temporary/profile.json"
start_status=$?
set -e
case "$start_status" in
  0) ;;
  3)
    if [[ "$mode" == dry-run ]]; then
      echo "Pose is away from the configured training start; run would home first."
    elif [[ "$EEF_AUTO_HOME" != true ]]; then
      echo "Pose is away from the configured training start and auto_home=false." >&2; exit 1
    else
      env EEF_LOG_DIR="$run_dir/home" "$enable_var=$enable_token" "$runner" home
    fi ;;
  *) echo "Training start-pose verification failed." >&2; exit "$start_status" ;;
esac
if [[ "$mode" == dry-run ]]; then echo "Dry-run finished. Logs: $run_dir"; exit 0; fi
env EEF_LOG_DIR="$run_dir/execute" "$enable_var=$enable_token" EEF_ENABLE_GRIPPER=true \
  "$runner" execute
