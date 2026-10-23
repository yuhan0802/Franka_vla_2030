#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Task0 EEF 模型（Qwen3-VL-2B + DROID BAR，末端位姿控制），走 run_eef_pose.sh。
# 模型服务默认在 63 服务器上运行（权重位于其 /data2），端口 8095。
# 所有参数仍可在命令行临时覆盖，例如 EEF_SERVER_GPU=2 ./task1-act.sh。
export EEF_REMOTE_SSH="${EEF_REMOTE_SSH:-mac_63}"
export EEF_SERVER_HOST="${EEF_SERVER_HOST:-10.24.116.63}"
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8095}"
export EEF_SERVER_GPU="${EEF_SERVER_GPU:-1}"
export EEF_CHECKPOINT="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar/Qwen3VL2B+Franka-Task0+lr1e-05+taskprompt_resize256_localdata_resume1k_8gpu_b256/save_8000_chkpt}"
export EEF_ACTIONCODEC="${EEF_ACTIONCODEC:-/data2/lyh/vla/assets/ActionCodec-Base-RVQft}"
export EEF_STATE_STATS="${EEF_STATE_STATS:-/data2/lyh/vla/config/droid_eef_axis_angle_state_stats.json}"
# 必须与训练数据中的任务指令完全一致（/data2/lyh/vla/Franka_vr/tasks.json 的 "0"）。
export VLA_INSTRUCTION="place the red test tube in the red basket"
# 训练状态：夹爪宽度 >= 0.045 m 为张开(1)，否则闭合(0)。
export EEF_GRIPPER_STATE_SOURCE=binary
export EEF_EXTERIOR_CAMERA="${EEF_EXTERIOR_CAMERA:-left}"
# 该权重训练时两路图像都缩放到 256x256；adapter 拒绝不按此缩放的（旧）模型服务。
export EEF_MODEL_IMAGE_SIZE="${EEF_MODEL_IMAGE_SIZE:-256}"

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

# 训练的 29 条示范起点几乎相同（<0.3 mm / 0.13°），起点偏差会让抓取位置明显偏移。
# 与 eef_pose_safety.TASK0_START_POSITION/QUATERNION 一致。
start_offset() {  # 输出 "位置偏差(m) 姿态偏差(rad)"，取自 armed 事件
  local line position quaternion
  line="$(grep -m1 '"event": "armed"' "$1" 2>/dev/null)" || return 1
  position="$(sed -n 's/.*"position": \[\([^]]*\)\].*/\1/p' <<<"$line" | tr -d ,)"
  quaternion="$(sed -n 's/.*"quaternion_xyzw": \[\([^]]*\)\].*/\1/p' <<<"$line" | tr -d ,)"
  [[ -n "$position" && -n "$quaternion" ]] || return 1
  awk -v p="$position" -v q="$quaternion" 'BEGIN {
    split(p, a, " "); split(q, b, " ")
    d = sqrt((a[1] - 0.5577)^2 + (a[2] + 0.0015)^2 + (a[3] - 0.3205)^2)
    c = b[1] * 0.92537 + b[2] * -0.37885 + b[3] * -0.01236 + b[4] * 0.00364; if (c < 0) c = -c; if (c > 1) c = 1
    printf "%.4f %.4f\n", d, 2 * atan2(sqrt(1 - c * c), c) }'
}
read -r offset turn < <(start_offset "$run_dir/dry_run/policy.jsonl") || {
  echo "Dry-run log has no armed pose; refusing to execute." >&2; exit 1; }
echo "Start pose: ${offset} m / ${turn} rad from the Task0 training start."
if awk -v d="$offset" -v r="$turn" 'BEGIN { exit !(d > 0.005 || r > 0.02) }'; then
  home_cmd="EEF_ENABLE_REAL_ROBOT=I_UNDERSTAND_EEF_POSE $script_dir/run_eef_pose.sh home"
  if [[ "$mode" == dry-run ]]; then
    echo "Start pose is off the training start; run executes home first (or: $home_cmd)."
  elif [[ "${EEF_AUTO_HOME:-true}" != true ]]; then
    echo "Start pose is off the training start and EEF_AUTO_HOME=false; run: $home_cmd" >&2; exit 1
  else
    echo "Moving to the Task0 training start first (home)."
    EEF_LOG_DIR="$run_dir/home" EEF_ENABLE_REAL_ROBOT=I_UNDERSTAND_EEF_POSE "$script_dir/run_eef_pose.sh" home || {
      echo "home failed; refusing to execute." >&2; exit 1; }
  fi
fi
[[ "$mode" == dry-run ]] && { echo "Dry-run finished. Logs: $run_dir/dry_run"; exit 0; }

# 完整任务：串行 15 步动作块，默认最多 60 块，启用夹爪。每块约 3 s（执行 1 s + 到位等待 + 推理）。
exec env \
  EEF_LOG_DIR="$run_dir/execute" \
  EEF_ENABLE_REAL_ROBOT=I_UNDERSTAND_EEF_POSE \
  EEF_ENABLE_GRIPPER=true \
  EEF_MAX_CHUNKS="${EEF_MAX_CHUNKS:-60}" \
  EEF_MAX_ACTIONS="${EEF_MAX_ACTIONS:-900}" \
  "$script_dir/run_eef_pose.sh" execute
