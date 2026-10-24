#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Task0 EEF 模型（Qwen3-VL-2B + DROID BAR，末端位姿控制）。
# 默认绕过 MoveIt Servo：run_eef_direct.sh（pi0.5 direct 关节速度栈 + 进程内与 VR 采集时
# Servo 相同的 P 跟踪器）。EEF_CONTROL=servo 退回 run_eef_pose.sh（MoveIt Servo）。
# 模型服务默认在 63 服务器上运行（权重位于其 /data2），端口 8095。
# 所有参数仍可在命令行临时覆盖，例如 EEF_SERVER_GPU=2 ./task1-act.sh。
#
#   ./task1-act.sh axis-test +z   direct 首次使用先逐轴调试（每轴 5 步 x 3 mm），通过后才允许 run
#   ./task1-act.sh dry-run        只推理，不发任何运动命令
#   ./task1-act.sh                完整任务
#   ./task1-act.sh home | probe | stop
export EEF_REMOTE_SSH="${EEF_REMOTE_SSH:-mac_63}"
export EEF_SERVER_HOST="${EEF_SERVER_HOST:-10.24.116.63}"
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8095}"
export EEF_SERVER_GPU="${EEF_SERVER_GPU:-1}"
export EEF_CHECKPOINT="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar_v4/Qwen3VL2B+Franka-Task0+lr0.0001+v4_measured_world_left_aug_10k_8gpu_b256/save_6000_chkpt}"
export EEF_ACTIONCODEC="${EEF_ACTIONCODEC:-/data2/lyh/vla/assets/ActionCodec-Base-RVQft}"
export EEF_STATE_STATS="${EEF_STATE_STATS:-/data2/lyh/vla/config/droid_eef_axis_angle_state_stats.json}"
# 必须与训练数据中的任务指令完全一致（/data2/lyh/vla/Franka_vr/tasks.json 的 "0"）。
# 其他权重（如 task1-droid.sh）用 EEF_INSTRUCTION 覆盖；不读 VLA_INSTRUCTION，避免沿用 pi0.5 流程的旧值。
export VLA_INSTRUCTION="${EEF_INSTRUCTION:-place the red test tube in the red basket}"
# 训练状态：夹爪宽度 >= 0.045 m 为张开(1)，否则闭合(0)。
export EEF_GRIPPER_STATE_SOURCE="${EEF_GRIPPER_STATE_SOURCE:-binary}"
export EEF_EXTERIOR_CAMERA="${EEF_EXTERIOR_CAMERA:-left}"
# 该权重训练时两路图像都缩放到 256x256；adapter 拒绝不按此缩放的（旧）模型服务。
export EEF_MODEL_IMAGE_SIZE="${EEF_MODEL_IMAGE_SIZE:-256}"
# 每块从上一块的最终目标接着积分（到位等待后二者相差 <=3 mm）。训练时机械臂一直在追领先
# 约 2 cm 的 VR 目标，进度从不清零；从实测位姿重起会让接近试管时每块 1-2 mm 的细调
# 被 3 mm 到位容差吃掉（2026-10-23 实测 z 指令 -1.8 mm/块只走 -1.5 mm，悬停 15 块）。
export EEF_CHUNK_START="${EEF_CHUNK_START:-last_target}"

control="${EEF_CONTROL:-direct}"
case "$control" in
  direct) runner="$script_dir/run_eef_direct.sh"; enable_var=EEF_DIRECT_ENABLE_REAL_ROBOT; enable_token=I_UNDERSTAND_DIRECT_EEF;;
  servo)  runner="$script_dir/run_eef_pose.sh";   enable_var=EEF_ENABLE_REAL_ROBOT;        enable_token=I_UNDERSTAND_EEF_POSE;;
  *) echo "EEF_CONTROL must be direct or servo" >&2; exit 2;;
esac
commissioned="$script_dir/logs/.eef_direct_commissioned"
log_prefix="${EEF_LOG_PREFIX:-eef_task1_act}"
stamp() { date +%Y%m%d_%H%M%S; }

mode="${1:-run}"
case "$mode" in
  run|dry-run) ;;
  axis-test)
    axis="${2:-}"
    case "$axis" in +x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz) ;; *)
      echo "Usage: $0 axis-test {+x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz}" >&2; exit 2;; esac
    log="$script_dir/logs/${log_prefix}_axis_${axis}_$(stamp)_$$"
    env EEF_LOG_DIR="$log" "$enable_var=$enable_token" EEF_TEST_AXIS="$axis" \
      EEF_ROTATION_FRAME="${EEF_ROTATION_FRAME:-tool}" "$runner" axis-test
    grep -q '"event": "complete"' "$log/policy.jsonl" && ! grep -q '"event": "fault"' "$log/policy.jsonl" || {
      echo "axis-test $axis did not complete cleanly; inspect $log" >&2; exit 1; }
    if [[ "$control" == direct ]]; then
      mkdir -p "$(dirname "$commissioned")"; echo "$(date -Is) $axis $log" >> "$commissioned"
    fi
    echo "axis-test $axis ($control) passed. Logs: $log"; exit 0 ;;
  home)
    exec env EEF_LOG_DIR="$script_dir/logs/${log_prefix}_home_$(stamp)_$$" "$enable_var=$enable_token" "$runner" home ;;
  # 只观测：保存图像与各来源的夹爪/机械臂状态，不发机械臂命令（走 Servo 栈，见 run_eef_pose.sh 说明）。
  probe) exec "$script_dir/run_eef_pose.sh" probe ;;
  stop|STOP)
    "$script_dir/run_eef_direct.sh" stop || true
    "$script_dir/run_eef_pose.sh" stop || true
    exec "$script_dir/run_eef_pose.sh" server-stop ;;
  *) echo "Usage: $0 [dry-run|axis-test <axis>|home|probe|stop]" >&2; exit 2 ;;
esac

if [[ "$control" == direct && "$mode" == run && ! -s "$commissioned" && "${EEF_DIRECT_SKIP_COMMISSION:-}" != true ]]; then
  echo "REFUSED: the Servo-free tracker has not passed a real-robot axis-test yet." >&2
  echo "  Run e.g.: $0 axis-test +z && $0 axis-test -z && $0 axis-test +rz   (hand on the E-stop)" >&2
  echo "  Or use MoveIt Servo: EEF_CONTROL=servo $0" >&2
  exit 3
fi

"$runner" server
run_dir="${EEF_RUN_DIR:-$script_dir/logs/${log_prefix}_$(stamp)_$$}"

dry_env=()
if [[ "$control" == direct ]]; then
  # A dry-run-only invocation leaves a DISARMED direct stack; a full run needs it armed.
  stack_record="$script_dir/logs/.eef_direct_stack_log_dir"
  if [[ "$mode" == run ]] && tmux has-session -t franka_vr_eef_direct_robot 2>/dev/null &&
      ! grep -q 'direct executor is ARMED' "$(cat "$stack_record" 2>/dev/null)/robot.log" 2>/dev/null; then
    echo "Restarting the disarmed direct stack armed."; "$runner" stop
  fi
  [[ "$mode" == run ]] && dry_env=(EEF_DIRECT_STACK=armed "$enable_var=$enable_token")
fi

# 真机执行前先做一次真实观测 dry-run：只推理，不发布任何目标/速度。
env EEF_LOG_DIR="$run_dir/dry_run" EEF_MAX_CHUNKS=1 ${dry_env[@]+"${dry_env[@]}"} "$runner" dry-run
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
  if [[ "$mode" == dry-run ]]; then
    echo "Start pose is off the training start; run executes home first (or: $0 home)."
  elif [[ "${EEF_AUTO_HOME:-true}" != true ]]; then
    echo "Start pose is off the training start and EEF_AUTO_HOME=false; run: $0 home" >&2; exit 1
  else
    echo "Moving to the Task0 training start first (home)."
    env EEF_LOG_DIR="$run_dir/home" "$enable_var=$enable_token" "$runner" home || {
      echo "home failed; refusing to execute." >&2; exit 1; }
  fi
fi
[[ "$mode" == dry-run ]] && { echo "Dry-run finished ($control). Logs: $run_dir/dry_run"; exit 0; }

# 完整任务：串行 15 步动作块，默认最多 60 块，启用夹爪。每块约 3 s（执行 1 s + 到位等待 + 推理）。
exec env \
  EEF_LOG_DIR="$run_dir/execute" \
  "$enable_var=$enable_token" \
  EEF_ENABLE_GRIPPER=true \
  EEF_MAX_CHUNKS="${EEF_MAX_CHUNKS:-60}" \
  EEF_MAX_ACTIONS="${EEF_MAX_ACTIONS:-900}" \
  "$runner" execute
