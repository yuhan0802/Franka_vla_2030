#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Task0 EEF 模型，绕过 MoveIt Servo：走 run_eef_direct.sh（pi0.5 direct 关节速度栈 +
# 进程内与 VR 采集时 Servo 相同的 P 跟踪器，增益 2/s、0.30 m/s、0.975 rad/s）。
# 策略侧（指令、256 图像、起点检查、夹爪、到位等待）与 task1-act.sh 完全相同。
# 默认使用增强版权重（同 task1-act2.sh，端口 8096 的服务可直接复用）。
#
#   ./task1-act3.sh axis-test +z   首次使用先逐轴调试（每轴 5 步 x 3 mm），通过后才允许 run
#   ./task1-act3.sh dry-run        只推理（direct 栈 disarmed，不创建速度发布器）
#   ./task1-act3.sh                完整任务
#   ./task1-act3.sh home | stop
export EEF_REMOTE_SSH="${EEF_REMOTE_SSH:-mac_63}"
export EEF_SERVER_HOST="${EEF_SERVER_HOST:-10.24.116.63}"
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8096}"
export EEF_SERVER_GPU="${EEF_SERVER_GPU:-1}"
export EEF_CHECKPOINT="${EEF_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar/Qwen3VL2B+Franka-Task0+lr1e-05+from8k_side50_primaryrrc_bothcolornoise_5k_8gpu_b256/save_4000_chkpt}"
export EEF_ACTIONCODEC="${EEF_ACTIONCODEC:-/data2/lyh/vla/assets/ActionCodec-Base-RVQft}"
export EEF_STATE_STATS="${EEF_STATE_STATS:-/data2/lyh/vla/config/droid_eef_axis_angle_state_stats.json}"
# 必须与训练数据中的任务指令完全一致（/data2/lyh/vla/Franka_vr/tasks.json 的 "0"）。
export VLA_INSTRUCTION="place the red test tube in the red basket"
export EEF_GRIPPER_STATE_SOURCE=binary
export EEF_EXTERIOR_CAMERA="${EEF_EXTERIOR_CAMERA:-left}"
export EEF_MODEL_IMAGE_SIZE="${EEF_MODEL_IMAGE_SIZE:-256}"
run_direct="$script_dir/run_eef_direct.sh"
commissioned="$script_dir/logs/.eef_direct_commissioned"
direct_enable=I_UNDERSTAND_DIRECT_EEF

mode="${1:-run}"
case "$mode" in
  run|dry-run) ;;
  axis-test)
    axis="${2:-}"
    case "$axis" in +x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz) ;; *)
      echo "Usage: $0 axis-test {+x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz}" >&2; exit 2;; esac
    log="$script_dir/logs/eef_task1_act3_axis_${axis}_$(date +%Y%m%d_%H%M%S)_$$"
    EEF_LOG_DIR="$log" EEF_DIRECT_ENABLE_REAL_ROBOT="$direct_enable" EEF_TEST_AXIS="$axis" \
      EEF_ROTATION_FRAME="${EEF_ROTATION_FRAME:-tool}" "$run_direct" axis-test
    grep -q '"event": "complete"' "$log/policy.jsonl" && ! grep -q '"event": "fault"' "$log/policy.jsonl" || {
      echo "axis-test $axis did not complete cleanly; inspect $log" >&2; exit 1; }
    mkdir -p "$(dirname "$commissioned")"; echo "$(date -Is) $axis $log" >> "$commissioned"
    echo "axis-test $axis passed (recorded in $commissioned). Logs: $log"; exit 0 ;;
  home)
    exec env EEF_LOG_DIR="$script_dir/logs/eef_task1_act3_home_$(date +%Y%m%d_%H%M%S)_$$" \
      EEF_DIRECT_ENABLE_REAL_ROBOT="$direct_enable" "$run_direct" home ;;
  stop|STOP)
    "$run_direct" stop || true
    exec "$run_direct" server-stop ;;
  *) echo "Usage: $0 [dry-run|axis-test <axis>|home|stop]" >&2; exit 2 ;;
esac

if [[ "$mode" == run && ! -s "$commissioned" && "${EEF_DIRECT_SKIP_COMMISSION:-}" != true ]]; then
  echo "REFUSED: the Servo-free tracker has not passed a real-robot axis-test yet." >&2
  echo "  Run e.g.: $0 axis-test +z && $0 axis-test -z && $0 axis-test +rz   (hand on the E-stop)" >&2
  exit 3
fi

"$run_direct" server
run_dir="${EEF_RUN_DIR:-$script_dir/logs/eef_task1_act3_$(date +%Y%m%d_%H%M%S)_$$}"

# A dry-run-only invocation leaves a DISARMED stack; a full run needs it armed.
stack_record="$script_dir/logs/.eef_direct_stack_log_dir"
if [[ "$mode" == run ]] && tmux has-session -t franka_vr_eef_direct_robot 2>/dev/null &&
    ! grep -q 'direct executor is ARMED' "$(cat "$stack_record" 2>/dev/null)/robot.log" 2>/dev/null; then
  echo "Restarting the disarmed direct stack armed."; "$run_direct" stop
fi

# 真机执行前先做一次真实观测 dry-run：只推理，不发布任何速度。
if [[ "$mode" == run ]]; then stack=armed; else stack=disarmed; fi
EEF_LOG_DIR="$run_dir/dry_run" EEF_MAX_CHUNKS=1 EEF_DIRECT_STACK="$stack" \
  EEF_DIRECT_ENABLE_REAL_ROBOT="$([[ "$stack" == armed ]] && echo "$direct_enable")" "$run_direct" dry-run
grep -q '"robot_command_sent": false' "$run_dir/dry_run/policy.jsonl" || {
  echo "Mandatory dry-run did not prove robot_command_sent=false; refusing to execute." >&2; exit 1; }

# 训练的 29 条示范起点几乎相同（<0.3 mm / 0.13°）；与 eef_pose_safety.TASK0_START_* 一致。
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
    EEF_LOG_DIR="$run_dir/home" EEF_DIRECT_ENABLE_REAL_ROBOT="$direct_enable" "$run_direct" home || {
      echo "home failed; refusing to execute." >&2; exit 1; }
  fi
fi
[[ "$mode" == dry-run ]] && { echo "Dry-run finished. Logs: $run_dir/dry_run"; exit 0; }

# 完整任务：串行 15 步动作块，默认最多 60 块，启用夹爪。每块约 3 s（执行 1 s + 到位等待 + 推理）。
exec env \
  EEF_LOG_DIR="$run_dir/execute" \
  EEF_DIRECT_ENABLE_REAL_ROBOT="$direct_enable" \
  EEF_ENABLE_GRIPPER=true \
  EEF_MAX_CHUNKS="${EEF_MAX_CHUNKS:-60}" \
  EEF_MAX_ACTIONS="${EEF_MAX_ACTIONS:-900}" \
  "$run_direct" execute
