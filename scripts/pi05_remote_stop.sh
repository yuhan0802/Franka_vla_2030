#!/usr/bin/env bash
# 在远端执行：只停止参数与本任务完全匹配的 π0.5 推理服务。
set -euo pipefail

port="$1"
session="$2"
checkpoint="$3"
policy_config="$4"
tmux_tmpdir="$5"
export TMUX_TMPDIR="$tmux_tmpdir"

listening() {
  ss -ltn | awk -v port=":$port" '$4 ~ port"$" {found=1} END {exit !found}'
}

matching_pids() {
  python3 - "$port" "$checkpoint" "$policy_config" <<'PY'
import pathlib
import sys

port, checkpoint, policy_config = sys.argv[1:]
for path in pathlib.Path("/proc").glob("[0-9]*/cmdline"):
    try:
        args = path.read_bytes().decode().split("\0")
    except (OSError, UnicodeError):
        continue
    if not any(arg.endswith("serve_policy.py") for arg in args):
        continue
    if "--port" not in args or args[args.index("--port") + 1] != port:
        continue
    if "--policy.dir=" + checkpoint not in args:
        continue
    if "--policy.config=" + policy_config not in args:
        continue
    print(path.parent.name)
PY
}

if tmux has-session -t "$session" 2>/dev/null; then
  echo "Sending SIGINT through remote tmux: $session"
  tmux send-keys -t "$session" C-c
fi

# JAX 内核执行期间信号处理可能延迟，先给正常退出留出时间。
for _ in $(seq 1 300); do
  listening || break
  sleep .1
done

if listening; then
  mapfile -t pids < <(matching_pids)
  if (( ${#pids[@]} != 1 )); then
    echo "Port $port is still listening, but no unique matching policy process was found; refusing to kill it." >&2
    exit 1
  fi
  echo "Sending SIGINT to verified policy process: ${pids[0]}"
  kill -INT "${pids[0]}"
  for _ in $(seq 1 300); do
    listening || break
    sleep .1
  done
fi

if listening; then
  echo "Verified policy process did not release port $port; SIGKILL was not used." >&2
  exit 1
fi
tmux kill-session -t "$session" 2>/dev/null || true
echo "Stopped verified policy service on port $port."
