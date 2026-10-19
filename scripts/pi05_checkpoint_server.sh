#!/usr/bin/env bash
# Runs on server 66 through SSH stdin; never touches robot interfaces.
set -euo pipefail
step="$1"; port="$2"; session="$3"; deploy="$4"; root="$5"
gpu="${6:-0}"
policy_config="${7:-pi05_franka_tube_pick_v5_finetune}"
load_timeout="${8:-300}"
root="${root:-/mnt/82_store/lyh/VLA/openpi-main/checkpoints/pi05_franka_tube_pick_v5_finetune/tube_pick_v5_full_bs32_20k}"
checkpoint="$root/$step"
[[ "$load_timeout" =~ ^[1-9][0-9]*$ ]] || {
  echo "Invalid server load timeout: $load_timeout" >&2
  exit 2
}
[[ -d "$checkpoint/params" ]] || {
  echo "Missing pi0.5 checkpoint params: $checkpoint/params" >&2
  echo "Provide the actual checkpoint root using PI05_CHECKPOINT_ROOT. Existing weights were not changed." >&2
  exit 4
}
export TMUX_TMPDIR="${TMUX_TMPDIR:-/data/vla/tmp/tmux}"
listening() { ss -ltn | awk -v port=":$port" '$4 ~ port"$" {found=1} END {exit !found}'; }
verify() {
  python3 - "$port" "$checkpoint" "$policy_config" <<'PY'
import pathlib
import sys
port, checkpoint, policy_config = sys.argv[1:]
for path in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
    try:
        args = path.read_bytes().decode().split('\0')
    except (OSError, UnicodeError):
        continue
    if not any(arg.endswith('serve_policy.py') for arg in args):
        continue
    if '--port' not in args or args[args.index('--port') + 1] != port:
        continue
    if '--policy.dir=' + checkpoint in args and '--policy.config=' + policy_config in args:
        print('Verified serving checkpoint: ' + checkpoint + ' (' + policy_config + ')')
        sys.exit(0)
sys.exit('Running service does not match checkpoint/config: ' + checkpoint + ' (' + policy_config + ')')
PY
}
if listening; then
  if verify; then
    exit 0
  fi
  if ! tmux has-session -t "$session" 2>/dev/null; then
    echo "Refusing to replace an unmanaged service on port $port." >&2
    exit 8
  fi
  echo "Switching managed policy service to: $checkpoint ($policy_config)"
  tmux kill-session -t "$session"
  for attempt in $(seq 1 100); do
    listening || break
    sleep 0.1
  done
  if listening; then
    echo "Managed service did not release port $port; refusing to start another process." >&2
    exit 9
  fi
fi
if tmux has-session -t "$session" 2>/dev/null; then
  echo "Session already exists but port is unavailable: $session; inspect its log before retrying." >&2
  exit 5
fi
log_root=/data/vla/openpi-main/logs/pi05-droid-66
mkdir -p "$TMUX_TMPDIR" "$log_root"
# Put all settings in the pane command: tmux does not reliably inherit the
# caller's environment when its server is already running.
printf -v command 'exec env CUDA_VISIBLE_DEVICES=%q PORT=%q PI05_DROID_TMUX_SESSION=%q CHECKPOINT_STEP=%q CHECKPOINT_ROOT=%q CONFIG_NAME=%q bash %q foreground >>%q 2>&1' \
  "$gpu" "$port" "$session" "$step" "$root" "$policy_config" "$deploy" "$log_root/$session.log"
tmux new-session -d -s "$session" "$command"
echo "Waiting up to $load_timeout seconds for policy service on port $port..."
for attempt in $(seq 1 "$load_timeout"); do
  if listening; then verify; exit 0; fi
  tmux has-session -t "$session" 2>/dev/null || {
    tail -40 "$log_root/$session.log" >&2
    exit 6
  }
  if (( attempt % 30 == 0 )); then
    echo "Still loading policy checkpoint ($attempt/$load_timeout seconds)..."
  fi
  sleep 1
done
echo "Server loading timed out after $load_timeout seconds; inspect $log_root/$session.log" >&2
exit 7
