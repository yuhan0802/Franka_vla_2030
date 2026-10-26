#!/usr/bin/env bash
# Shared task entry point. Task-specific settings live in vla_instruction_weights.json.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if (( $# < 1 || $# > 2 )); then
  echo "Usage: $0 TASK_ID [run|stop|stop-only|status|show]" >&2; exit 2
fi
task_id="$1"
action="${2:-run}"
case "$action" in run|stop|STOP|stop-only|status|show) ;;
  *) echo "Usage: $0 TASK_ID [run|stop|stop-only|status|show]" >&2; exit 2;;
esac

export VLA_REMOTE_HOST="${VLA_REMOTE_HOST:-10.24.116.66}"
export VLA_REMOTE_USER="${VLA_REMOTE_USER:-xmu}"
export VLA_SSH_PORT="${VLA_SSH_PORT:-1172}"
export VLA_SSH_KEY="${VLA_SSH_KEY:-/home/mac/.ssh/id_rsa_xmu_66}"
export PI05_REMOTE_UV_PROJECT_ENVIRONMENT="${PI05_REMOTE_UV_PROJECT_ENVIRONMENT:-/data/vla/openpi-main/.venv}"
export PI05_REMOTE_ENV_FILE="${PI05_REMOTE_ENV_FILE:-/dev/null}"
export PI05_REMOTE_LOG_ROOT="${PI05_REMOTE_LOG_ROOT:-/home/xmu/.local/state/pi05-droid/logs}"
export PI05_REMOTE_TMUX_TMPDIR="${PI05_REMOTE_TMUX_TMPDIR:-/home/xmu/.cache/pi05-droid/tmux}"
export PI05_SERVER_GPU="${PI05_SERVER_GPU:-1}"

task_config="$(mktemp --suffix=.json "${TMPDIR:-/tmp}/pi05_task.XXXXXXXX")"
trap 'rm -f -- "$task_config"' EXIT
mapfile -t selected < <(python3 "$root_dir/scripts/resolve_direct_task.py" \
  --config "${VLA_TASK_CONFIG:-$root_dir/vla_instruction_weights.json}" \
  --task-id "$task_id" --output "$task_config")
if (( ${#selected[@]} != 5 )); then echo "Failed to resolve task $task_id" >&2; exit 2; fi

export VLA_INSTRUCTION="${selected[0]}" VLA_TASK_CONFIG="$task_config"
# The selected task owns its checkpoint. Do not inherit another task's weights.
unset PI05_CHECKPOINT_ROOT PI05_CHECKPOINT_STEP PI05_POLICY_CONFIG
case "$action" in
  show) cat "$task_config"; exit 0 ;;
  status) "$root_dir/run_pi05_direct.sh" status; exit $? ;;
  stop|STOP)
    "$root_dir/run_pi05_direct.sh" stop
    if [[ "${selected[1]}" == 1 ]]; then "$root_dir/fci_init.sh"; fi
    exit 0 ;;
  stop-only) "$root_dir/run_pi05_direct.sh" stop; exit $? ;;
esac

export PI05_DIRECT_ENABLE_FULL_TASK=I_UNDERSTAND_FULL_DIRECT_TASK
export PI05_DIRECT_VELOCITY_INTERFACE_COMMISSIONED=I_ACCEPT_VELOCITY_INTERFACE_RISK
export PI05_DIRECT_GRIPPER_OPEN_WIDTH="${PI05_DIRECT_GRIPPER_OPEN_WIDTH:-0.07}"
export PI05_DIRECT_GRIPPER_CLOSED_WIDTH="${PI05_DIRECT_GRIPPER_CLOSED_WIDTH:-0.01}"
export PI05_DIRECT_GRIPPER_EPSILON_INNER="${PI05_DIRECT_GRIPPER_EPSILON_INNER:-0.01}"
export PI05_DIRECT_GRIPPER_EPSILON_OUTER="${PI05_DIRECT_GRIPPER_EPSILON_OUTER:-0.07}"
if [[ -n "${selected[2]}" ]]; then
  export PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH="${PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH:-${selected[2]}}"
fi
if [[ -n "${selected[3]}" ]]; then
  export PI05_DIRECT_GRIPPER_FEEDBACK_CLOSED_WIDTH="${PI05_DIRECT_GRIPPER_FEEDBACK_CLOSED_WIDTH:-${selected[3]}}"
fi
if [[ -n "${selected[4]}" ]]; then
  export PI05_DIRECT_GRIPPER_VERIFY_OPEN_WIDTH="${PI05_DIRECT_GRIPPER_VERIFY_OPEN_WIDTH:-${selected[4]}}"
fi
"$root_dir/run_pi05_direct.sh" full-execute
