#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

export VLA_SSH_KEY=/home/mac/.ssh/id_rsa_66
export FRANKA_LOG_ROOT=/media/mac/Elements2/franka_vr_docker/logs
export PI05_SERVER_GPU=5
export VLA_SERVER_PORT=8094
export PI05_REMOTE_ENV_FILE=/dev/shm/pi05-xmu/env.sh
export PI05_REMOTE_LOG_ROOT=/dev/shm/pi05-xmu/logs
export PI05_REMOTE_TMUX_TMPDIR=/dev/shm/pi05-xmu/tmux
export PI05_DIRECT_PREDICTION_HORIZON=16
export PI05_DIRECT_EXECUTE_HORIZON=16

mountpoint /media/mac/Elements2
./run_task.sh task20 show

tunnel=$(mktemp -d)
cleanup() {
  if [[ -S "$tunnel/ssh" ]]; then
    ssh -S "$tunnel/ssh" -O exit xmu@10.24.116.66 2>/dev/null || true
  fi
  rm -f -- "$tunnel/gripper_profile"
  rmdir -- "$tunnel" || true
}
trap cleanup EXIT

ssh -fN -M -S "$tunnel/ssh" -o ExitOnForwardFailure=yes \
  -L "127.0.0.1:18094:127.0.0.1:$VLA_SERVER_PORT" \
  -p 1172 -i "$VLA_SSH_KEY" xmu@10.24.116.66

FCI_GRIPPER_PROFILE_OUT="$tunnel/gripper_profile" ./fci_init.sh
read -r PI05_DIRECT_GRIPPER_OPEN_WIDTH PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH \
  < "$tunnel/gripper_profile"
export PI05_DIRECT_GRIPPER_OPEN_WIDTH PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH

PI05_POLICY_CONNECT_HOST=127.0.0.1 PI05_POLICY_CONNECT_PORT=18094 \
  ./run_task.sh task20 run
