#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Task0 EEF 模型（Qwen3-VL-2B + DROID BAR，末端位姿控制），走 run_eef_pose.sh。
# 模型服务在 20 服务器上运行（权重位于其 /data2），端口 8095。
# 所有参数仍可在命令行临时覆盖，例如 EEF_SERVER_GPU=2 ./task1-act.sh。
export EEF_REMOTE_SSH="${EEF_REMOTE_SSH:-mac_20}"
export EEF_SERVER_HOST="${EEF_SERVER_HOST:-10.24.116.20}"
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8095}"
export EEF_SERVER_GPU="${EEF_SERVER_GPU:-1}"
export EEF_CHECKPOINT="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar/Qwen3VL2B+Franka-Task0+lr1e-05+taskprompt_resize256_localdata_resume1k_8gpu_b256/save_8000_chkpt}"
export EEF_ACTIONCODEC="${EEF_ACTIONCODEC:-/data2/lyh/weights/ActionCodec-Base-RVQft}"
# 必须与训练数据中的任务指令完全一致。
export VLA_INSTRUCTION="put the red test tube into the basket"
# 训练状态：夹爪宽度 >= 0.045 m 为张开(1)，否则闭合(0)。
export EEF_GRIPPER_STATE_SOURCE=binary
export EEF_EXTERIOR_CAMERA="${EEF_EXTERIOR_CAMERA:-left}"

mode="${1:-run}"
case "$mode" in
  run|dry-run) ;;
  # 只观测：保存图像与各来源的夹爪/机械臂状态，不发机械臂命令（夹爪测试见 run_eef_pose.sh 说明）。
  probe) exec "$script_dir/run_eef_pose.sh" probe ;;
  stop|STOP)
    "$script_dir/run_eef_pose.sh" stop || true
    exec "$script_dir/run_eef_pose.sh" server-stop ;;
  *) echo "Usage: $0 [dry-run|probe|stop]" >&2; exit 2 ;;
esac

"$script_dir/run_eef_pose.sh" server
run_dir="${EEF_RUN_DIR:-$script_dir/logs/eef_task1_act_$(date +%Y%m%d_%H%M%S)_$$}"

# 真机执行前先做一次真实观测 dry-run：只推理，不发布任何目标。
EEF_LOG_DIR="$run_dir/dry_run" EEF_MAX_CHUNKS=1 "$script_dir/run_eef_pose.sh" dry-run
grep -q '"robot_command_sent": false' "$run_dir/dry_run/policy.jsonl" || {
  echo "Mandatory dry-run did not prove robot_command_sent=false; refusing to execute." >&2; exit 1; }
[[ "$mode" == dry-run ]] && { echo "Dry-run finished. Logs: $run_dir/dry_run"; exit 0; }

# 完整任务：串行 15 步动作块，默认最多 60 块（约 60 s），启用夹爪。
exec env \
  EEF_LOG_DIR="$run_dir/execute" \
  EEF_ENABLE_REAL_ROBOT=I_UNDERSTAND_EEF_POSE \
  EEF_ENABLE_GRIPPER=true \
  EEF_MAX_CHUNKS="${EEF_MAX_CHUNKS:-60}" \
  EEF_MAX_ACTIONS="${EEF_MAX_ACTIONS:-900}" \
  "$script_dir/run_eef_pose.sh" execute
