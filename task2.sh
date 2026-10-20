#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
