#!/usr/bin/env bash
# Reproduce the successful 2026-10-15 20k trial. Default mode moves real hardware.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"
mode="${1:-execute}"
case "$mode" in
  execute|dry-run|stop|status) ;;
  *) echo "Usage: $0 [execute|dry-run|stop|status]" >&2; exit 2 ;;
esac

export ROBOT_IP=172.16.0.2
export VLA_SSH_KEY='/home/mac/下载/id_rsa_66'
export VLA_REMOTE_HOST=10.24.116.66 VLA_REMOTE_USER=vla VLA_SSH_PORT=1172
export VLA_INSTRUCTION="${VLA_INSTRUCTION:-put the red test tube into the red basket}"
task_config="${VLA_TASK_CONFIG:-$root_dir/vla_instruction_weights.json}"
mapfile -t selected_task < <(
  python3 "$root_dir/scripts/select_vla_task.py" \
    --config "$task_config" \
    --instruction "$VLA_INSTRUCTION"
)
if (( ${#selected_task[@]} != 7 )); then
  echo "Failed to resolve VLA_INSTRUCTION from $task_config" >&2
  exit 2
fi
export PI05_CHECKPOINT_ROOT="${selected_task[0]}"
export PI05_CHECKPOINT_STEP="${selected_task[1]}"
export PI05_POLICY_CONFIG="${selected_task[2]}"
export PI05_MAX_ACTIONS="${selected_task[3]}"
export PI05_MAX_JOINT_VELOCITY="${selected_task[4]}"
export PI05_MAX_JOINT_ACCELERATION="${selected_task[5]}"
export PI05_GRIPPER_FORCE="${selected_task[6]}"
export VLA_SERVER_PORT=8094 PI05_SERVER_GPU=0
export PI05_DROID_TMUX_SESSION=pi05-droid-66-8094
export PI05_ENABLE_REAL_ROBOT_EXECUTION=I_UNDERSTAND PI05_ENABLE_GRIPPER=true
# raw: legacy image_raw + autofocus; calibrated: image_rect + calibration.
export VLA_CAMERA_MODE="${VLA_CAMERA_MODE:-raw}"
case "$VLA_CAMERA_MODE" in
  raw|calibrated) ;;
  *) echo "VLA_CAMERA_MODE must be raw or calibrated" >&2; exit 2 ;;
esac

if [[ "$mode" == stop || "$mode" == status ]]; then
  exec ./run_pi05.sh "$mode"
fi

run_dir="$root_dir/logs/pi05_20k_relaxed_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run_dir"
export PI05_LOG_DIR="$run_dir/robot"
exec > >(tee -a "$run_dir/run.log") 2>&1
echo "Logs: $run_dir"
echo "Instruction: $VLA_INSTRUCTION"
echo "Checkpoint: $PI05_CHECKPOINT_ROOT/$PI05_CHECKPOINT_STEP ($PI05_POLICY_CONFIG)"
echo "20k; left + wrist; DROID x3; serial 15 Hz; $PI05_MAX_ACTIONS actions; gripper inner/outer 10/30 mm."
if [[ "$mode" == execute ]]; then
  echo "This run executes real robot and gripper motion after the dry-run check."
fi

robot_session=franka_vr_pi05_joint_robot
camera_session=franka_vr_pi05_joint_cameras
if tmux has-session -t "$camera_session" 2>/dev/null; then
  active_camera_mode="$(tmux show-environment -t "$camera_session" VLA_CAMERA_MODE 2>/dev/null || true)"
  if [[ "$active_camera_mode" != "VLA_CAMERA_MODE=$VLA_CAMERA_MODE" ]]; then
    echo "Camera session mode is unknown or differs from $VLA_CAMERA_MODE. Stop the camera session before retrying." >&2
    exit 1
  fi
fi
echo "Camera mode: $VLA_CAMERA_MODE"
./run_pi05.sh server
started_robot=false
cleanup() {
  if [[ "$started_robot" == true ]] && tmux has-session -t "$robot_session" 2>/dev/null; then
    echo "Releasing robot control connection..."
    tmux send-keys -t "$robot_session" C-c
    for attempt in $(seq 1 150); do
      tmux has-session -t "$robot_session" 2>/dev/null || return 0
      sleep 0.1
    done
    echo "Robot is still shutting down; inspect its container before resetting."
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if ! tmux has-session -t "$robot_session" 2>/dev/null; then
  printf -v robot_command 'cd %q && PI05_LOG_DIR=%q ROBOT_IP=172.16.0.2 exec ./run_robot_pi05_joint.sh' "$root_dir" "$PI05_LOG_DIR"
  tmux new-session -d -s "$robot_session" "$robot_command"
  started_robot=true
fi
if ! tmux has-session -t "$camera_session" 2>/dev/null; then
  printf -v camera_command 'cd %q && VLA_CAMERA_MODE=%q exec ./run_vla_cameras.sh' "$root_dir" "$VLA_CAMERA_MODE"
  tmux new-session -d -s "$camera_session" -e "VLA_CAMERA_MODE=$VLA_CAMERA_MODE" "$camera_command"
fi

echo "Waiting for the robot controllers and Servo (up to 120 seconds)..."
ready=false
for attempt in $(seq 1 120); do
  if [[ "$started_robot" == false ]]; then
    # An existing stack is checked by the adapter's service/observation checks.
    ready=true
    break
  fi
  if ! tmux has-session -t "$robot_session" 2>/dev/null; then
    echo "Robot startup exited; see $PI05_LOG_DIR/robot.log" >&2
    exit 1
  fi
  if [[ -f "$PI05_LOG_DIR/robot.log" ]] &&
     grep -q 'Servo initialized successfully' "$PI05_LOG_DIR/robot.log" &&
     grep -q 'Configured and activated.*fr3_arm_controller' "$PI05_LOG_DIR/robot.log"; then
    ready=true
    break
  fi
  sleep 1
done
[[ "$ready" == true ]] || { echo "Robot startup timed out." >&2; exit 1; }

./run_pi05.sh dry-run 2>&1 | tee "$run_dir/dry_run.log"
if [[ "$mode" == execute ]]; then
  ./run_pi05.sh execute 2>&1 | tee "$run_dir/execute.log"
  if grep -q 'FAULT LATCHED' "$run_dir/execute.log"; then
    echo "Robot execution reported a fault; inspect execute.log." >&2
    exit 1
  fi
  echo "Execution finished. Check that the tube actually reached the basket."
fi
