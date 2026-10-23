#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Task0 EEF 增强版权重：从 save_8000 继续训练 5k 步（左右侧相机各 50%、主相机随机裁剪、
# 两路颜色抖动+噪声）。流程、指令、图像尺寸、控制方式（默认绕过 Servo，EEF_CONTROL=servo
# 退回）与 task1-act.sh 完全相同，只换权重和端口；端口 8096 与 task1-act.sh 的 8095 服务
# 互不复用，可同时存在。
# 用法同 task1-act.sh：./task1-act2.sh [dry-run|axis-test <axis>|home|probe|stop]
export EEF_CHECKPOINT="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar/Qwen3VL2B+Franka-Task0+lr1e-05+from8k_side50_primaryrrc_bothcolornoise_5k_8gpu_b256/save_4000_chkpt}"
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8096}"
export EEF_LOG_PREFIX="${EEF_LOG_PREFIX:-eef_task1_act2}"

exec "$script_dir/task1-act.sh" "$@"
