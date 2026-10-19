#!/usr/bin/env bash
# Run the pi0.5 simulation-trained pink-tube-to-blue-basket checkpoint on FR3.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"
mode="${1:-probe}"
case "$mode" in
  server|probe|execute|dry-run|stop|status) ;;
  *) echo "Usage: $0 [server|probe|execute|dry-run|stop|status]" >&2; exit 2 ;;
esac

export ROBOT_IP=172.16.0.2
export VLA_SSH_KEY='/home/mac/下载/id_rsa_66'
export VLA_REMOTE_HOST=10.24.116.66 VLA_REMOTE_USER=vla VLA_SSH_PORT=1172
export PI05_CHECKPOINT_ROOT='/mnt/82_store/lyh/VLA/openpi-main/checkpoints/pi05_sim_tube_pick_pink2bluebin_finetune/pink2bluebin_full_bs32_20k'
export PI05_CHECKPOINT_STEP=20000
export PI05_POLICY_CONFIG='pi05_sim_tube_pick_pink2bluebin_finetune'
export VLA_SERVER_PORT=8095 PI05_SERVER_GPU=0
export PI05_DROID_TMUX_SESSION=pi05-sim-pink2blue-8095
export VLA_INSTRUCTION='Place the pink tube into the blue basket.'
export PI05_ENABLE_GRIPPER=true
if [[ "$mode" == execute ]]; then
  export PI05_ENABLE_REAL_ROBOT_EXECUTION=I_UNDERSTAND
fi
export PI05_MAX_ACTIONS=2000 PI05_MAX_JOINT_VELOCITY=0.10
export PI05_MAX_JOINT_ACCELERATION=0.30 PI05_GRIPPER_FORCE=10

if [[ "$mode" == server || "$mode" == probe || "$mode" == stop || "$mode" == status ]]; then
  exec ./run_pi05.sh "$mode"
fi

run_dir="$root_dir/logs/pi05_sim2real_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$run_dir"
export PI05_LOG_DIR="$run_dir/robot"
exec > >(tee -a "$run_dir/run.log") 2>&1
echo "Logs: $run_dir"
echo "sim 20k; pink tube to blue basket; left + wrist; DROID x3; serial 15 Hz; 2000 actions."
if [[ "$mode" == execute ]]; then
  echo "This run executes real robot and gripper motion after the dry-run check."
fi

./run_pi05.sh server
robot_session=franka_vr_pi05_joint_robot
camera_session=franka_vr_pi05_joint_cameras
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
  printf -v camera_command 'cd %q && exec ./run_vla_cameras.sh' "$root_dir"
  tmux new-session -d -s "$camera_session" "$camera_command"
fi

echo "Waiting for the robot controllers and Servo (up to 120 seconds)..."
ready=false
for attempt in $(seq 1 120); do
  if [[ "$started_robot" == false ]]; then
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
  echo "Execution finished. Check that the pink tube actually reached the blue basket."
fi
