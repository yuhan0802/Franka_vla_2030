#!/usr/bin/env bash
# Explicit entry point for the parallel pi0.5 direct joint velocity workflow.
set -euo pipefail

mode="${1:-dry-run}"
case "$mode" in server|probe|dry-run|fake-execute|fake-commission|commission|execute|full-execute|status|stop) ;; *)
  echo "Usage: $0 {server|probe|dry-run|fake-execute|fake-commission|commission|execute|full-execute|status|stop}" >&2; exit 2;; esac
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
robot_session=franka_vr_pi05_direct_robot
camera_session=franka_vr_pi05_direct_cameras
server_session=pi05-droid-66-8094
instruction="${VLA_INSTRUCTION:-place the test tube rack in the black tray}"
host="${VLA_REMOTE_HOST:-10.24.116.66}"; port="${VLA_SERVER_PORT:-8094}"
task_config="${VLA_TASK_CONFIG:-$root_dir/vla_instruction_weights.json}"
mapfile -t selected_task < <(
  python3 "$root_dir/scripts/select_vla_task.py" --config "$task_config" --instruction "$instruction"
)
if (( ${#selected_task[@]} != 7 )); then
  echo "Failed to resolve VLA_INSTRUCTION from $task_config" >&2
  exit 2
fi
# Reuse the deployment identity and exact checkpoint selection proven by
# run_20k.sh. `execute` keeps its immutable commissioning envelope;
# `full-execute` imports the selected task's production runtime settings.
export VLA_REMOTE_HOST="$host" VLA_REMOTE_USER="${VLA_REMOTE_USER:-vla}" VLA_SSH_PORT="${VLA_SSH_PORT:-1172}"
export VLA_SSH_KEY="${VLA_SSH_KEY:-/home/mac/下载/id_rsa_66}" VLA_SERVER_PORT="$port"
export PI05_CHECKPOINT_ROOT="${PI05_CHECKPOINT_ROOT:-${selected_task[0]}}"
export PI05_CHECKPOINT_STEP="${PI05_CHECKPOINT_STEP:-${selected_task[1]}}"
export PI05_POLICY_CONFIG="${PI05_POLICY_CONFIG:-${selected_task[2]}}"
export PI05_SERVER_GPU="${PI05_SERVER_GPU:-0}" PI05_DROID_TMUX_SESSION="$server_session"
if [[ "$mode" == full-execute ]]; then default_camera_mode=raw; else default_camera_mode=calibrated; fi
camera_mode="${VLA_CAMERA_MODE:-$default_camera_mode}"
case "$camera_mode" in raw) suffix=image_raw;; calibrated) suffix=image_rect;;
  *) echo "VLA_CAMERA_MODE must be raw or calibrated" >&2; exit 2;; esac
run_stamp="$(date +%Y%m%d_%H%M%S)_$$"; log_dir="${PI05_DIRECT_LOG_DIR:-$root_dir/logs/pi05_direct_$run_stamp}"
if [[ "$mode" == full-execute ]]; then
  [[ "${PI05_DIRECT_ENABLE_FULL_TASK:-}" == I_UNDERSTAND_FULL_DIRECT_TASK ]] || {
    echo "REFUSED: full-execute requires PI05_DIRECT_ENABLE_FULL_TASK=I_UNDERSTAND_FULL_DIRECT_TASK." >&2; exit 3; }
  [[ "${PI05_DIRECT_VELOCITY_INTERFACE_COMMISSIONED:-}" == I_ACCEPT_VELOCITY_INTERFACE_RISK ]] || {
    echo "REFUSED: new velocity interface has not passed real-robot commissioning; full-execute is disabled." >&2; exit 3; }
  full_actions="${selected_task[3]}"
  # task2 使用 run_20k.sh 同一份 runtime 配置：速度 0.6 rad/s、加速度
  # 2.0 rad/s²。此前 direct 路径把它覆盖成 0.15/0.5，实测将模型 4 号关节
  # 0.253 rad/s 的请求压到了约 0.147 rad/s，改变了抓取接近阶段的动作节奏。
  # 这里直接读取所选任务配置，避免模型、普通 20k 路径与 direct 路径参数不一致。
  full_velocity="${PI05_DIRECT_FULL_MAX_VELOCITY:-${selected_task[4]}}"
  full_acceleration="${PI05_DIRECT_FULL_MAX_ACCELERATION:-${selected_task[5]}}"
  full_gripper_force="${selected_task[6]}"
  python3 - "$full_actions" "$full_velocity" "$full_acceleration" "$full_gripper_force" <<'PY'
import sys
actions, velocity, acceleration, force = map(float, sys.argv[1:])
if not (1 <= actions <= 2000): raise SystemExit("full task max_actions must be in [1, 2000]")
if not (0 < velocity <= .6): raise SystemExit("full direct velocity must be in (0, 0.6] rad/s")
if not (0 < acceleration <= 2.0): raise SystemExit("full direct acceleration must be in (0, 2.0] rad/s^2")
if not (1 <= force <= 30): raise SystemExit("full task gripper force must be in [1, 30] N")
PY
fi

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
request_restricted_stop() {
  local id command
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true)"
    [[ "$command" == *franka_pi05_direct_velocity.launch.py* ]] || continue
    echo "Requesting zero target in direct launch container: $id"
    timeout --signal=TERM --kill-after=2s 5s docker exec "$id" bash -lc '
      source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
      ros2 topic pub -r 15 --times 4 /pi05/direct_joint_velocity_cmd control_msgs/msg/JointJog \
        "{joint_names: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration: 0.0666667}"
    ' >/dev/null 2>&1 || true
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
}
stop_direct_containers() {
  local id command
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true)"
    [[ "$command" == *franka_pi05_direct_velocity.launch.py* ]] || continue
    echo "Forwarding SIGINT to direct launch container: $id"
    docker kill --signal=SIGINT "$id" >/dev/null || true
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
}
full_cleanup_done=false
cleanup_full_task() {
  [[ ( "$mode" == full-execute || "$mode" == commission || "$mode" == fake-commission ) && "$full_cleanup_done" == false ]] || return 0
  full_cleanup_done=true
  request_restricted_stop
  stop_session "$camera_session" || true
  stop_session "$robot_session" || true
  stop_direct_containers
  if [[ "$mode" == full-execute ]]; then "$root_dir/run_vla_pi05_droid_66.sh" stop || true; fi
}

case "$mode" in
  server|probe) exec "$root_dir/run_vla_pi05_droid_66.sh" "$mode";;
  status)
    status_session "$robot_session"; status_session "$camera_session"
    "$root_dir/run_vla_pi05_droid_66.sh" status || true
    echo "Expected server tmux: $server_session"; echo "Latest logs:"; ls -dt "$root_dir"/logs/pi05_direct_* 2>/dev/null | head -3 || true
    exit 0;;
  stop)
    failed=0; request_restricted_stop; stop_session "$camera_session" || failed=1; stop_session "$robot_session" || failed=1
    stop_direct_containers
    "$root_dir/run_vla_pi05_droid_66.sh" stop || failed=1; exit "$failed";;
esac

mkdir -p "$log_dir"; log_dir="$(cd "$log_dir" && pwd)"; export PI05_DIRECT_LOG_DIR="$log_dir"
echo "SELECTED TASK: $instruction"
echo "SELECTED CHECKPOINT: $PI05_CHECKPOINT_ROOT/$PI05_CHECKPOINT_STEP ($PI05_POLICY_CONFIG)"
if [[ "$mode" == full-execute || "$mode" == commission || "$mode" == fake-commission ]]; then trap cleanup_full_task EXIT INT TERM; fi
if [[ "$mode" == fake-execute ]]; then
  start_session "$robot_session" "PI05_DIRECT_LOG_DIR='$log_dir' PI05_DIRECT_MAX_VELOCITY=0.05 PI05_DIRECT_MAX_ACCELERATION=0.15 ./run_robot_pi05_direct.sh fake"
  echo "Fake hardware armed. No real FCI connection is used. Logs: $log_dir"
  exit 0
fi
if [[ "$mode" == commission || "$mode" == fake-commission ]]; then
  if [[ "$mode" == commission && "${PI05_DIRECT_ENABLE_REAL_ROBOT:-}" != I_UNDERSTAND_DIRECT_VELOCITY ]]; then
    echo "REFUSED: commission requires PI05_DIRECT_ENABLE_REAL_ROBOT=I_UNDERSTAND_DIRECT_VELOCITY and on-site safety confirmation." >&2
    exit 3
  fi
  if [[ "$mode" == commission ]]; then robot_mode=armed; else robot_mode=fake; fi
  start_session "$robot_session" "PI05_DIRECT_LOG_DIR='$log_dir' PI05_DIRECT_MAX_VELOCITY=0.02 PI05_DIRECT_MAX_ACCELERATION=0.1 ./run_robot_pi05_direct.sh '$robot_mode'"
  echo "Commissioning ($robot_mode): joint 1, 0.02 rad/s, 0.5 s, no policy server, cameras, or gripper."
fi

# The first policy call after loading performs model/JAX warm-up and may exceed
# the runtime 2 s latency gate. Run the server's robot-free synthetic probe
# before acquiring live observations so runtime latency is measured warm.
if [[ "$mode" != commission && "$mode" != fake-commission ]]; then
  "$root_dir/run_vla_pi05_droid_66.sh" probe
  start_session "$camera_session" "VLA_CAMERA_MODE='$camera_mode' ./run_vla_cameras.sh"
fi
if [[ "$mode" == dry-run ]]; then
  start_session "$robot_session" "PI05_DIRECT_LOG_DIR='$log_dir' ./run_robot_pi05_direct.sh disarmed"
  adapter_flags=()
elif [[ "$mode" == commission || "$mode" == fake-commission ]]; then
  adapter_flags=()
elif [[ "$mode" == execute ]]; then
  [[ "${PI05_DIRECT_ENABLE_REAL_ROBOT:-}" == I_UNDERSTAND_DIRECT_VELOCITY ]] || {
    echo "REFUSED: set PI05_DIRECT_ENABLE_REAL_ROBOT=I_UNDERSTAND_DIRECT_VELOCITY only after the现场 safety confirmation." >&2; exit 3; }
  # Short first-motion envelope: 0.05 rad/s, 0.15 rad/s², horizon 2, 10 actions, no gripper.
  start_session "$robot_session" "PI05_DIRECT_LOG_DIR='$log_dir' PI05_DIRECT_MAX_VELOCITY=0.05 PI05_DIRECT_MAX_ACCELERATION=0.15 ./run_robot_pi05_direct.sh armed"
  adapter_flags=(--execute --confirm-real-robot DIRECT_JOINT_VELOCITY_PI05 --execute-horizon 2 --max-actions 10)
else
  # Keep the selected task's action budget, but never broaden the deliberately
  # conservative direct-controller motion envelope below.
  echo "FULL DIRECT TASK: actions=$full_actions velocity=$full_velocity acceleration=$full_acceleration gripper_force=$full_gripper_force camera=$camera_mode"
  start_session "$robot_session" "PI05_DIRECT_LOG_DIR='$log_dir' PI05_DIRECT_LOAD_GRIPPER=true PI05_DIRECT_MAX_VELOCITY='$full_velocity' PI05_DIRECT_MAX_ACCELERATION='$full_acceleration' ./run_robot_pi05_direct.sh armed"
  # The 20k checkpoint was trained and previously validated with complete
  # 16-action chunks.  Replanning after only the first four actions repeatedly
  # discards the late-chunk grasp transition and leaves the policy in approach.
  adapter_flags=(--execute --confirm-real-robot DIRECT_JOINT_VELOCITY_PI05 --execute-horizon 16
    --max-actions "$full_actions" --enable-gripper --gripper-speed 0.02 --gripper-force "$full_gripper_force"
    --gripper-open-width "${PI05_DIRECT_GRIPPER_OPEN_WIDTH:-0.08}"
    --gripper-closed-width "${PI05_DIRECT_GRIPPER_CLOSED_WIDTH:-0.0}"
    --gripper-epsilon-inner "${PI05_DIRECT_GRIPPER_EPSILON_INNER:-0.01}"
    --gripper-epsilon-outer "${PI05_DIRECT_GRIPPER_EPSILON_OUTER:-0.01}")
fi

echo "Waiting for the direct robot stack to become ready (up to 120 seconds)..."
ready=false
for attempt in $(seq 1 120); do
  if ! tmux has-session -t "$robot_session" 2>/dev/null; then
    echo "Direct robot stack exited; inspect $log_dir/robot.log" >&2
    exit 1
  fi
  if [[ -f "$log_dir/robot.log" ]] && grep -q 'FAULT LATCHED' "$log_dir/robot.log"; then
    echo "Direct executor faulted during startup; refusing to start the policy adapter." >&2
    exit 1
  fi
  if [[ "$mode" == dry-run ]]; then
    grep -q 'direct executor is DISARMED' "$log_dir/robot.log" &&
      grep -q 'Configured and activated.*joint_state_broadcaster' "$log_dir/robot.log" && { ready=true; break; }
  else
    grep -q 'armed dependencies ready' "$log_dir/robot.log" &&
      grep -q 'Configured and activated.*fr3_direct_velocity_controller' "$log_dir/robot.log" && { ready=true; break; }
  fi
  sleep 1
done
[[ "$ready" == true ]] || { echo "Direct robot startup timed out; policy adapter was not started." >&2; exit 1; }

if [[ "$mode" == commission || "$mode" == fake-commission ]]; then
  if [[ "$mode" == fake-commission ]]; then commission_fake_flag=--fake; else commission_fake_flag=; fi
  set +e
  docker compose run --rm -v "$log_dir:/pi05_logs" franka_vr bash -lc '
    set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
    exec python3 src/franka_vr/franka_vr/scripts/pi05_direct_velocity_commission.py --json-log /pi05_logs/commission.json '"$commission_fake_flag"'
  ' 2>&1 | tee "$log_dir/commission.log"
  commission_status=${PIPESTATUS[0]}
  set -e
  cleanup_full_task
  (( commission_status == 0 )) || { echo "Commissioning failed; inspect $log_dir" >&2; exit "$commission_status"; }
  echo "Commissioning passed. Logs: $log_dir"
  exit 0
fi

if [[ "$mode" == full-execute ]]; then
  echo "Running mandatory live-observation dry-run before full direct execution..."
  docker compose run --rm \
    -v "$log_dir:/pi05_logs" franka_vr bash -lc '
      set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
      exec python3 src/franka_vr/franka_vr/scripts/pi05_direct_policy_adapter.py \
        --server-host "'"$host"'" --server-port "'"$port"'" --instruction "'"$instruction"'" \
        --left-image-topic /camera_left/'"$suffix"' --execute-horizon 16 \
        --jsonl-log /pi05_logs/full_dry_run_policy.jsonl
    ' 2>&1 | tee "$log_dir/full_dry_run.log"
  grep -q '"robot_command_sent": false' "$log_dir/full_dry_run_policy.jsonl" || {
    echo "Mandatory full-task dry-run did not prove robot_command_sent=false." >&2; exit 1; }
fi

set +e
docker compose run --rm \
  -v "$log_dir:/pi05_logs" franka_vr bash -lc '
    set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; source install/setup.bash
    exec python3 src/franka_vr/franka_vr/scripts/pi05_direct_policy_adapter.py \
      --server-host "'"$host"'" --server-port "'"$port"'" --instruction "'"$instruction"'" \
      --left-image-topic /camera_left/'"$suffix"' --jsonl-log /pi05_logs/policy.jsonl '"${adapter_flags[*]}"'
  ' 2>&1 | tee "$log_dir/adapter.log"
adapter_status=${PIPESTATUS[0]}
set -e
if [[ "$mode" == full-execute ]]; then
  echo "Full direct task adapter exited; stopping all dedicated sessions."
  cleanup_full_task
fi
if grep -q 'FAULT LATCHED' "$log_dir/adapter.log" "$log_dir/robot.log" 2>/dev/null; then
  echo "Direct workflow reported a latched fault; inspect $log_dir." >&2
  exit 1
fi
(( adapter_status == 0 )) || exit "$adapter_status"
echo "Direct workflow finished. Logs: $log_dir"
