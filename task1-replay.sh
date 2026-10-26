#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/scripts/runtime_log_root.sh"

# 回放：不跑模型，把真机数据预处理后的轨迹（franka_measured_eef_base_15hz_v5.npz）原样发给真机。
# replay_eef_trajectory_server.py 在 63 上按模型服务的协议逐块返回缓存里的动作块（world 帧左乘增量，
# 即 preprocess_franka_task_measured_eef_v4/v5.py 的逆运算），其余链路（dry-run、回起点、direct 跟踪器、
# 夹爪、安全检查）与 task1-act.sh 完全相同。轨迹相对起始位姿回放，试管/篮子需按录制时摆放。
# 所有参数仍可在命令行临时覆盖，例如 EEF_REPLAY_NPZ=/data2/.../xxx_v5.npz ./task1-replay.sh。
#
#   ./task1-replay.sh verify      只在 63 上校验：把动作积分回去，与缓存里实测位姿比对，不动机械臂
#   ./task1-replay.sh axis-test +z   direct 首次使用先逐轴调试（每轴 5 步 x 3 mm），通过后才允许 run
#   ./task1-replay.sh dry-run     只取动作不发运动命令
#   ./task1-replay.sh             完整回放（手放急停）
#   ./task1-replay.sh home | probe | stop
export EEF_REMOTE_SSH="${EEF_REMOTE_SSH:-mac_63}"
export EEF_SERVER_HOST="${EEF_SERVER_HOST:-10.24.116.63}"
# 回放服务单独占用 8098，不影响 8095 上的模型服务。
export EEF_SERVER_PORT="${EEF_SERVER_PORT:-8098}"
export EEF_SERVER_GPU="${EEF_SERVER_GPU:-1}"
replay_npz="${EEF_REPLAY_NPZ:-/data2/lyh/vla/Franka_vr/Task0/TS_PB_v5_001_cxc/data/franka_measured_eef_base_15hz_v5.npz}"
export EEF_CHECKPOINT="$replay_npz"   # 回放服务把它当作缓存路径，并以此声明 checkpoint
export EEF_ACTIONCODEC="" EEF_STATE_STATS=""
export EEF_SERVER_SCRIPT="replay_eef_trajectory_server.py"
# 服务端每块前进的帧数必须等于 adapter 每块实际执行的步数。
export EEF_EXECUTE_HORIZON="${EEF_EXECUTE_HORIZON:-15}"
export EEF_SERVER_ARGS="--execute-horizon $EEF_EXECUTE_HORIZON"
# 必须与训练数据中的任务指令完全一致（/data2/lyh/vla/Franka_vr/tasks.json 的 "0"）。
# 其他权重（如 task1-droid.sh）用 EEF_INSTRUCTION 覆盖；不读 VLA_INSTRUCTION，避免沿用 pi0.5 流程的旧值。
export VLA_INSTRUCTION="${EEF_INSTRUCTION:-place the red test tube in the red basket}"
# v4 训练状态：夹爪开度连续，clip((width - 0.01) / 0.07, 0, 1)；动作里的夹爪仍是 0/1 目标。
export EEF_GRIPPER_STATE_SOURCE="${EEF_GRIPPER_STATE_SOURCE:-measured}"
export EEF_EXTERIOR_CAMERA="${EEF_EXTERIOR_CAMERA:-left}"
# 该权重训练时两路图像都缩放到 256x256；adapter 拒绝不按此缩放的（旧）模型服务。
export EEF_MODEL_IMAGE_SIZE="${EEF_MODEL_IMAGE_SIZE:-256}"
# 按 world frame 左乘执行模型输出的旋转增量；服务端和客户端会交叉校验这一声明。
export EEF_ROTATION_FRAME="${EEF_ROTATION_FRAME:-world}"
export EEF_ROTATION_EXECUTION="${EEF_ROTATION_EXECUTION:-relative_rotvec_left_multiply}"
# Task1 默认完整执行模型的平移增量；0 表示不做单步 6 mm 平移裁剪。
export EEF_MAX_TRANSLATION="${EEF_MAX_TRANSLATION:-0}"
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
commissioned="$FRANKA_LOG_ROOT/.eef_direct_commissioned"
log_prefix="${EEF_LOG_PREFIX:-eef_task1_replay}"
stamp() { date +%Y%m%d_%H%M%S; }
# 回放位置是服务端进程内的状态（dry-run 也会推进它），每次使用前都重启，保证从第 0 帧开始。
fresh_server() { "$script_dir/run_eef_pose.sh" server-stop; "$runner" server; }

mode="${1:-run}"
case "$mode" in
  run|dry-run) ;;
  verify)
    ssh "$EEF_REMOTE_SSH" "mkdir -p ~/franka_eef_server && cd ~/franka_eef_server
      export VLA_PROJECT_ROOT=/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA
      /data2/lyh/envs/qwenvla/bin/python - --verify --checkpoint '$replay_npz'" \
      < "$script_dir/replay_eef_trajectory_server.py"; exit $? ;;
  axis-test)
    axis="${2:-}"
    case "$axis" in +x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz) ;; *)
      echo "Usage: $0 axis-test {+x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz}" >&2; exit 2;; esac
    log="$FRANKA_LOG_ROOT/${log_prefix}_axis_${axis}_$(stamp)_$$"
    env EEF_LOG_DIR="$log" "$enable_var=$enable_token" EEF_TEST_AXIS="$axis" \
      EEF_ROTATION_FRAME="${EEF_ROTATION_FRAME:-tool}" "$runner" axis-test
    grep -q '"event": "complete"' "$log/policy.jsonl" && ! grep -q '"event": "fault"' "$log/policy.jsonl" || {
      echo "axis-test $axis did not complete cleanly; inspect $log" >&2; exit 1; }
    if [[ "$control" == direct ]]; then
      mkdir -p "$(dirname "$commissioned")"; echo "$(date -Is) $axis $log" >> "$commissioned"
    fi
    echo "axis-test $axis ($control) passed. Logs: $log"; exit 0 ;;
  home)
    exec env EEF_LOG_DIR="$FRANKA_LOG_ROOT/${log_prefix}_home_$(stamp)_$$" "$enable_var=$enable_token" "$runner" home ;;
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

fresh_server
run_dir="${EEF_RUN_DIR:-$FRANKA_LOG_ROOT/${log_prefix}_$(stamp)_$$}"

dry_env=()
if [[ "$control" == direct ]]; then
  # A dry-run-only invocation leaves a DISARMED direct stack; a full run needs it armed.
  stack_record="$FRANKA_LOG_ROOT/.eef_direct_stack_log_dir"
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
fresh_server   # dry-run 已让回放前进了一块，真机回放前重新从第 0 帧开始

# 完整回放：每块 15 步，启用夹爪。轨迹播完后服务端返回静止动作，夹爪释放且连续 2 块静止即结束。
exec env \
  EEF_LOG_DIR="$run_dir/execute" \
  "$enable_var=$enable_token" \
  EEF_ENABLE_GRIPPER=true \
  EEF_MAX_CHUNKS="${EEF_MAX_CHUNKS:-60}" \
  EEF_MAX_ACTIONS="${EEF_MAX_ACTIONS:-900}" \
  "$runner" execute
