#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认使用已验证可用的 66 服务器账号与运行环境。
# 所有参数仍可在命令行临时覆盖，例如 PI05_SERVER_GPU=0 ./task2.sh。
export VLA_REMOTE_HOST="${VLA_REMOTE_HOST:-10.24.116.66}"
export VLA_REMOTE_USER="${VLA_REMOTE_USER:-xmu}"
export VLA_SSH_PORT="${VLA_SSH_PORT:-1172}"
export VLA_SSH_KEY="${VLA_SSH_KEY:-/home/mac/.ssh/id_rsa_xmu_66}"
export PI05_REMOTE_UV_PROJECT_ENVIRONMENT="${PI05_REMOTE_UV_PROJECT_ENVIRONMENT:-/data/vla/openpi-main/.venv}"
export PI05_REMOTE_ENV_FILE="${PI05_REMOTE_ENV_FILE:-/dev/null}"
export PI05_REMOTE_LOG_ROOT="${PI05_REMOTE_LOG_ROOT:-/home/xmu/.local/state/pi05-droid/logs}"
export PI05_REMOTE_TMUX_TMPDIR="${PI05_REMOTE_TMUX_TMPDIR:-/home/xmu/.cache/pi05-droid/tmux}"
export PI05_SERVER_GPU="${PI05_SERVER_GPU:-1}"
case "${1:-run}" in
  run) ;;
  stop|STOP) exec "$script_dir/run_pi05_direct.sh" stop ;;
  *) echo "Usage: $0 [stop]" >&2; exit 2 ;;
esac

exec env \
  PI05_DIRECT_ENABLE_FULL_TASK=I_UNDERSTAND_FULL_DIRECT_TASK \
  PI05_DIRECT_VELOCITY_INTERFACE_COMMISSIONED=I_ACCEPT_VELOCITY_INTERFACE_RISK \
  PI05_DIRECT_GRIPPER_OPEN_WIDTH=0.14 \
  PI05_DIRECT_GRIPPER_CLOSED_WIDTH=0.07 \
  PI05_DIRECT_GRIPPER_EPSILON_INNER=0.01 \
  PI05_DIRECT_GRIPPER_EPSILON_OUTER=0.07 \
  "$script_dir/run_pi05_direct.sh" full-execute
