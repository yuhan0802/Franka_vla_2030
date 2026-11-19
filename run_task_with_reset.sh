#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if (( $# != 1 )); then
  echo "Usage: $0 TASK_ID" >&2
  exit 2
fi
task_id="$1"
./run_task.sh "$task_id" show >/dev/null

profile=$(mktemp)
trap 'rm -f -- "$profile"' EXIT
FCI_GRIPPER_PROFILE_OUT="$profile" ./fci_init.sh
read -r PI05_DIRECT_GRIPPER_OPEN_WIDTH PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH < "$profile"
if [[ ! "$PI05_DIRECT_GRIPPER_OPEN_WIDTH" =~ ^0\.[0-9]+$ ||
      ! "$PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH" =~ ^0\.[0-9]+$ ]]; then
  echo "Invalid gripper profile from this reset." >&2
  exit 1
fi
export PI05_DIRECT_GRIPPER_OPEN_WIDTH PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH

./run_task.sh "$task_id" run
