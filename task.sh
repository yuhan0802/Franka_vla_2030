#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 联合任务暂时使用 20 号 GPU 服务器；task1.sh/task2.sh 仍保持原来的 66。
remote_host="10.24.116.20"
remote_user="xmu"
remote_ssh_port="1172"
remote_ssh_key="/home/mac/.ssh/id_rsa_20"
server_port="8094"
server_gpu="0"
remote_uv_bin="/home/xmu/.local/bin/uv"
remote_uv_project_environment="/data/Franka_vr/openpi-pi05-venv"
remote_env_file="/home/xmu/.config/openpi/env-20.sh"
remote_log_root="/home/xmu/.local/state/pi05-droid/logs"
remote_tmux_tmpdir="/home/xmu/.cache/pi05-droid/tmux"

case "${1:-}" in
  exc1)
    instruction="put the red test tube into the red basket"
    ;;
  exc2)
    instruction="place the test tube rack in the black tray"
    ;;
  stop|STOP)
    exec env \
      VLA_REMOTE_HOST="$remote_host" \
      VLA_REMOTE_USER="$remote_user" \
      VLA_SSH_PORT="$remote_ssh_port" \
      VLA_SSH_KEY="$remote_ssh_key" \
      VLA_SERVER_PORT="$server_port" \
      PI05_SERVER_GPU="$server_gpu" \
      PI05_REMOTE_UV_BIN="$remote_uv_bin" \
      PI05_REMOTE_UV_PROJECT_ENVIRONMENT="$remote_uv_project_environment" \
      PI05_REMOTE_ENV_FILE="$remote_env_file" \
      PI05_REMOTE_LOG_ROOT="$remote_log_root" \
      PI05_REMOTE_TMUX_TMPDIR="$remote_tmux_tmpdir" \
      "$script_dir/run_pi05_direct.sh" stop
    ;;
  *)
    echo "Usage: $0 {exc1|exc2|stop}" >&2
    echo "  exc1: 红色试管放入红色篮子" >&2
    echo "  exc2: 试管架放入黑色托盘" >&2
    exit 2
    ;;
esac

source_config="${VLA_TASK_CONFIG:-$script_dir/vla_instruction_weights.json}"
task_config="$(mktemp --suffix=.json "${TMPDIR:-/tmp}/pi05_joint_task.XXXXXXXX")"
trap 'rm -f -- "$task_config"' EXIT

# 两条指令固定共用 Task1+Task2 联合训练的 15k 完整权重。运行参数沿用
# 已完成真机验证的 task2 配置，因此 exc1 与 exc2 之间只有指令不同。
python3 - "$source_config" "$task_config" "$instruction" <<'PY'
import json
import sys

source_path, output_path, instruction = sys.argv[1:]
with open(source_path, encoding="utf-8") as source:
    config = json.load(source)

tasks = {task["instruction"]: task for task in config["tasks"]}
task = dict(tasks[instruction])
task2 = tasks["place the test tube rack in the black tray"]
task["checkpoint_root"] = (
    "/mnt/82_store/lyh/VLA/openpi-main/checkpoints/"
    "pi05_franka_tube_pick_v5_finetune/task1_task2_full_bs32_20k"
)
task["checkpoint_step"] = 15000
task["policy_config"] = "pi05_franka_tube_pick_v5_finetune"
task["runtime"] = task2["runtime"]

with open(output_path, "w", encoding="utf-8") as output:
    json.dump({"version": config["version"], "tasks": [task]}, output)
PY

env -u PI05_CHECKPOINT_ROOT -u PI05_CHECKPOINT_STEP -u PI05_POLICY_CONFIG \
  VLA_REMOTE_HOST="$remote_host" \
  VLA_REMOTE_USER="$remote_user" \
  VLA_SSH_PORT="$remote_ssh_port" \
  VLA_SSH_KEY="$remote_ssh_key" \
  VLA_SERVER_PORT="$server_port" \
  PI05_SERVER_GPU="$server_gpu" \
  PI05_REMOTE_UV_BIN="$remote_uv_bin" \
  PI05_REMOTE_UV_PROJECT_ENVIRONMENT="$remote_uv_project_environment" \
  PI05_REMOTE_ENV_FILE="$remote_env_file" \
  PI05_REMOTE_LOG_ROOT="$remote_log_root" \
  PI05_REMOTE_TMUX_TMPDIR="$remote_tmux_tmpdir" \
  VLA_TASK_CONFIG="$task_config" \
  VLA_INSTRUCTION="$instruction" \
  PI05_DIRECT_ENABLE_FULL_TASK=I_UNDERSTAND_FULL_DIRECT_TASK \
  PI05_DIRECT_VELOCITY_INTERFACE_COMMISSIONED=I_ACCEPT_VELOCITY_INTERFACE_RISK \
  PI05_DIRECT_GRIPPER_OPEN_WIDTH=0.08 \
  PI05_DIRECT_GRIPPER_CLOSED_WIDTH=0.01 \
  PI05_DIRECT_GRIPPER_EPSILON_INNER=0.01 \
  PI05_DIRECT_GRIPPER_EPSILON_OUTER=0.07 \
  "$script_dir/run_pi05_direct.sh" full-execute
