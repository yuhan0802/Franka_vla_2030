#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# DROID 预训练 EEF 权重（Qwen3-VL-2B state-first BAR，droid_1.0.1 训练 20k 步，未在 Task0 上微调）。
# 模型服务换成 deploy_droid_qwen_actioncodec_eef.py（DROID 训练的 prompt/图像/状态格式），
# 端口 8097；机器人侧流程、控制方式（默认绕过 Servo，EEF_CONTROL=servo 退回）与 task1-act.sh 相同。
# 与 Task0 权重的差别（均来自 DROID 训练数据，服务端 metadata 声明，adapter 自动采用）：
#   - 夹爪状态为连续开度（measured：宽度 0.01..0.08 m 线性映射到 0..1），不是二值阈值
#   - 旋转动作是基座坐标系命令（左乘，rotation_frame=world）
#   - 指令可为任意自然语言，用 EEF_INSTRUCTION 覆盖
# 用法同 task1-act.sh：./task1-droid.sh [dry-run|axis-test <axis>|home|probe|stop]
export EEF_CHECKPOINT="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/droid_state_first/Qwen3VL2B+DROID+lr1e-05+droid_eef_bar16_so3_continue_b256-20261019_161048_edb4cec7/save_20000_chkpt}"
export EEF_ACTIONCODEC="${EEF_ACTIONCODEC:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA/actioncodec_libero/weights/ActionCodec-Base-RVQft}"
export EEF_STATE_STATS="${EEF_STATE_STATS:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA/droid_qwen3vl_actioncodec/droid_eef_axis_angle_state_stats.json}"
export EEF_SERVER_SCRIPT="${EEF_SERVER_SCRIPT:-deploy_droid_qwen_actioncodec_eef.py}"
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8097}"
export EEF_GRIPPER_STATE_SOURCE="${EEF_GRIPPER_STATE_SOURCE:-measured}"
export EEF_INSTRUCTION="${EEF_INSTRUCTION:-place the red test tube in the red basket}"
export EEF_LOG_PREFIX="${EEF_LOG_PREFIX:-eef_task1_droid}"

exec "$script_dir/task1-act.sh" "$@"
