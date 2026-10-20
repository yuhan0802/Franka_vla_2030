#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-run}" in
  run) ;;
  stop|STOP) exec "$script_dir/run_pi05_direct.sh" stop ;;
  *) echo "Usage: $0 [stop]" >&2; exit 2 ;;
esac

source_config="${VLA_TASK_CONFIG:-$script_dir/vla_instruction_weights.json}"
task_config="$(mktemp --suffix=.json "${TMPDIR:-/tmp}/pi05_task1.XXXXXXXX")"
trap 'rm -f -- "$task_config"' EXIT

# 只切换到 task1 的指令和权重；其余运行参数沿用 task2。
python3 - "$source_config" "$task_config" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as source:
    config = json.load(source)
tasks = {task["instruction"]: task for task in config["tasks"]}
task1 = dict(tasks["put the red test tube into the red basket"])
task2 = tasks["place the test tube rack in the black tray"]
task1["runtime"] = task2["runtime"]
with open(sys.argv[2], "w", encoding="utf-8") as output:
    json.dump({"version": config["version"], "tasks": [task1]}, output)
PY

env -u PI05_CHECKPOINT_ROOT -u PI05_CHECKPOINT_STEP -u PI05_POLICY_CONFIG \
  VLA_TASK_CONFIG="$task_config" \
  VLA_INSTRUCTION="put the red test tube into the red basket" \
  PI05_DIRECT_ENABLE_FULL_TASK=I_UNDERSTAND_FULL_DIRECT_TASK \
  PI05_DIRECT_VELOCITY_INTERFACE_COMMISSIONED=I_ACCEPT_VELOCITY_INTERFACE_RISK \
  PI05_DIRECT_GRIPPER_OPEN_WIDTH=0.14 \
  PI05_DIRECT_GRIPPER_CLOSED_WIDTH=0.07 \
  PI05_DIRECT_GRIPPER_EPSILON_INNER=0.01 \
  PI05_DIRECT_GRIPPER_EPSILON_OUTER=0.07 \
  "$script_dir/run_pi05_direct.sh" full-execute
