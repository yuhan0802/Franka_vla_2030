#!/usr/bin/env -S LC_ALL=C.UTF-8 LANG=C.UTF-8 bash
# Generic EEF task/model entry point. Fixed model identities come from JSON.
set -euo pipefail
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="${EEF_TASK_CONFIG:-$root_dir/config/eef_tasks.json}"
usage() {
  echo "Usage: $0 TASK_KEY MODEL_KEY [run|dry-run|axis-test AXIS|home|probe|stop|stop-only|status|show]"
  echo "       $0 list"
  echo "stop: close EEF resources, then run fci_init.sh (gripper homing + arm return)."
  echo "stop-only: close resources without homing or arm return."
}
case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  list) exec python3 "$root_dir/scripts/eef_task_config.py" list --config "$config" ;;
esac
if (( $# < 2 || $# > 4 )); then usage >&2; exit 2; fi
task_key="$1"; model_key="$2"; mode="${3:-run}"
case "$mode" in run|dry-run|home|probe|stop|STOP|stop-only|status|show|server|server-status) ;;
  axis-test)
    case "${4:-}" in +x|-x|+y|-y|+z|-z|+rx|-rx|+ry|-ry|+rz|-rz) ;;
      *) echo "axis-test requires +x,-x,+y,-y,+z,-z,+rx,-rx,+ry,-ry,+rz,-rz" >&2; exit 2;;
    esac ;;
  *) usage >&2; exit 2 ;;
esac
if (( $# == 4 )) && [[ "$mode" != axis-test ]]; then usage >&2; exit 2; fi

temporary="$(mktemp -d "${TMPDIR:-/tmp}/eef_task.XXXXXXXX")"
trap 'rm -rf -- "$temporary"' EXIT
control_flags=()
[[ -n "${EEF_CONTROL:-}" ]] && control_flags=(--control "$EEF_CONTROL")
# Keep an explicit per-run horizon override.  The resolved profile supplies the
# default, but callers must be able to run 15-predict/N-execute experiments
# without editing the shared task registry.
requested_execute_horizon="${EEF_EXECUTE_HORIZON:-}"
python3 "$root_dir/scripts/eef_task_config.py" resolve --config "$config" \
  --task "$task_key" --model "$model_key" "${control_flags[@]}" \
  --output "$temporary/profile.json" --env-file "$temporary/env"
if [[ "$mode" == show ]]; then cat "$temporary/profile.json"; exit 0; fi
while IFS= read -r -d '' key && IFS= read -r -d '' value; do export "$key=$value"; done < "$temporary/env"
if [[ -n "$requested_execute_horizon" ]]; then
  [[ "$requested_execute_horizon" =~ ^[0-9]+$ ]] &&
    (( requested_execute_horizon >= 1 && requested_execute_horizon <= 15 )) || {
      echo "EEF_EXECUTE_HORIZON must be an integer in [1, 15]." >&2; exit 2; }
  export EEF_EXECUTE_HORIZON="$requested_execute_horizon"
fi
source "$root_dir/scripts/runtime_log_root.sh" "$mode"

case "$EEF_CONTROL" in
  direct) runner="$root_dir/run_eef_direct.sh"; enable_var=EEF_DIRECT_ENABLE_REAL_ROBOT; enable_token=I_UNDERSTAND_DIRECT_EEF ;;
  servo) runner="$root_dir/run_eef_pose.sh"; enable_var=EEF_ENABLE_REAL_ROBOT; enable_token=I_UNDERSTAND_EEF_POSE ;;
esac
commissioned="$FRANKA_LOG_ROOT/.eef_direct_commissioned"
log_prefix="eef_${task_key}_${model_key}"
stamp() { date +%Y%m%d_%H%M%S; }

stage() {
  printf '\n>>> [%s] %s\n' "$1" "$2"
}

passed() {
  printf '>>> [通过] %s\n' "$1"
}

print_launch_summary() {
  python3 - "$temporary/profile.json" "$mode" "$FRANKA_LOG_ROOT" "$0" <<'PY'
import json
import pathlib
import sys

profile_path, mode, log_root, script = sys.argv[1:]
with open(profile_path, encoding="utf-8") as handle:
    profile = json.load(handle)

runtime = profile["runtime"]
server = profile["server"]
model_info = profile.get("model_info", {})
action_shape = model_info.get("action_shape", ["?", "?"])
gain = ", ".join(str(value) for value in runtime.get("action_gain", []))
control = profile["control"]
control_label = "direct (MoveIt Servo bypassed)" if control == "direct" else control
mode_labels = {
    "run": "真机执行",
    "dry-run": "安全试运行",
    "server": "模型服务",
    "server-status": "模型服务状态",
    "status": "控制器状态",
    "stop": "停止并复位",
    "STOP": "停止并复位",
    "stop-only": "仅停止",
    "home": "返回初始姿态",
    "probe": "合成数据探测",
    "axis-test": "坐标轴测试",
}

print("=" * 72)
print("Franka EEF 真机任务启动器")
print("=" * 72)
print(f"运行模式   : {mode_labels.get(mode, mode)}")
print(f"任务       : {profile['task_key']}")
print(f"模型配置   : {profile['model_key']}")
print(f"任务指令   : {profile['instruction']}")
print(f"控制方式   : {control_label}")
print(f"推理服务   : {server['host']}:{server['port']} via {server['ssh']} / GPU {server['gpu']}")
print(f"模型权重   : {server['checkpoint']}")
print(f"动作格式   : shape={action_shape}, normalization={profile['action_normalization']['method']}")
print(
    "分块参数   : "
    f"{runtime.get('chunk_mode', 'serial')}, predict={action_shape[0]}, "
    f"execute={runtime.get('execute_horizon', action_shape[0])}, "
    f"blend={runtime.get('rtc_blend_steps', 'n/a')}"
)
print(f"动作增益   : [{gain}]")
print(f"日志目录   : {pathlib.Path(log_root)}")
if mode == "run":
    print("安全检查   : 真机运动前执行 dry-run 和初始姿态校验")
    print(f"停止命令   : {script} {profile['task_key']} {profile['model_key']} stop-only")
print("=" * 72)
PY
}

pretty_execute_stream() {
  awk '
    function marker(label, text) {
      printf("\n>>> [%s] %s\n", label, text)
      fflush()
    }
    function emit_action(raw, values, count, xyz_norm, rot_norm, gripper, gripper_state) {
      gsub(/[\[\]]/, "", raw)
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", raw)
      count = split(raw, values, /[[:space:]]+/)
      if (count < 7) {
        marker("动作", "模型首个动作=[" raw "]")
        return
      }
      xyz_norm = sqrt((values[1] + 0) ^ 2 + (values[2] + 0) ^ 2 + (values[3] + 0) ^ 2)
      rot_norm = sqrt((values[4] + 0) ^ 2 + (values[5] + 0) ^ 2 + (values[6] + 0) ^ 2)
      gripper = values[7] + 0
      gripper_state = gripper >= 0.5 ? "张开" : "闭合"
      printf(">>> [动作] 平移 xyz=[% .5f, % .5f, % .5f] |xyz|=%.5f\n",
             values[1], values[2], values[3], xyz_norm)
      printf(">>> [动作] 旋转 rot=[% .5f, % .5f, % .5f] |rot|=%.5f | 夹爪=%.5f (%s)\n",
             values[4], values[5], values[6], rot_norm, gripper, gripper_state)
      fflush()
    }
    {
      if (capturing_action) {
        action_values = action_values " " $0
        if ($0 ~ /\]/) {
          sub(/\].*$/, "", action_values)
          emit_action(action_values)
          capturing_action = 0
          action_values = ""
        }
        next
      }
      if ($0 ~ /\[eef_pose_policy_adapter\]: startup checks passed:/) {
        marker("正常", "机器人反馈、TF 和控制服务均已就绪。")
      } else if ($0 ~ /\[eef_pose_policy_adapter\]: chunk [0-9]+:/) {
        summary = $0
        sub(/^.*\]: chunk /, "", summary)
        sub(/, first=.*/, "", summary)
        gsub(/: latency=/, "：推理耗时=", summary)
        gsub(/, delay=/, "，RTC延迟=", summary)
        gsub(/ steps/, " 步", summary)
        marker("运行中", "动作块 " summary)
        if ($0 ~ /first=\[/) {
          action_values = $0
          sub(/^.*first=\[/, "", action_values)
          if (action_values ~ /\]/) {
            sub(/\].*$/, "", action_values)
            emit_action(action_values)
            action_values = ""
          } else {
            capturing_action = 1
          }
        }
      } else if ($0 ~ /\[eef_pose_policy_adapter\]: gripper (OPEN|CLOSE) requested:/ ||
                 $0 ~ /\[eef_pose_policy_adapter\]: gripper (OPENED|CLOSED);/) {
        summary = $0
        sub(/^.*\]: gripper /, "", summary)
        gsub(/^OPEN requested:/, "请求张开：", summary)
        gsub(/^CLOSE requested:/, "请求闭合：", summary)
        gsub(/^OPENED;/, "已张开；", summary)
        gsub(/^CLOSED;/, "已闭合；", summary)
        marker("夹爪", summary)
      } else if ($0 ~ /\[ERROR\]|Traceback|collision|controller fault|aborted/) {
        marker("异常", "上方日志报告了控制器或运行时错误，请立即检查。")
      }
    }
  '
}

print_launch_summary

eef_containers() {
  local id command
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true)"
    case "$command" in
      *eef_pose_direct_adapter.py*|*eef_pose_policy_adapter.py*|*franka_pi05_direct_velocity.launch.py*|*franka_twist.launch.py*) echo "$id" ;;
    esac
  done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$root_dir")
}
stop_eef() {
  local id command attempt remaining failed=0
  # Stop policy publishers before releasing their arm controller.
  while read -r id; do
    [[ -n "$id" ]] || continue
    command="$(docker inspect "$id" --format '{{json .Config.Cmd}}' 2>/dev/null || true)"
    case "$command" in
      *eef_pose_direct_adapter.py*|*eef_pose_policy_adapter.py*) docker kill --signal=SIGINT "$id" >/dev/null || failed=1 ;;
    esac
  done < <(eef_containers)
  "$root_dir/run_eef_direct.sh" stop || failed=1
  "$root_dir/run_eef_pose.sh" stop || failed=1
  "$root_dir/run_eef_pose.sh" server-stop || failed=1
  for attempt in $(seq 1 100); do
    remaining="$(eef_containers)"
    if [[ -z "$remaining" ]]; then return "$failed"; fi
    sleep .2
  done
  echo "EEF containers are still running; FCI initialization will not start." >&2
  return 1
}

case "$mode" in
  stop|STOP)
    stage "停止" "正在发送零速度并关闭 EEF 控制资源..."
    stop_eef
    passed "EEF 控制器和模型服务已停止。"
    stage "复位" "正在校准夹爪并将机械臂返回保存姿态..."
    "$root_dir/fci_init.sh"
    passed "机械臂和夹爪复位完成。"
    exit 0 ;;
  stop-only)
    stage "停止" "正在发送零速度并关闭 EEF 控制资源..."
    stop_eef
    passed "EEF 控制器和模型服务已停止。"
    exit 0 ;;
  status)
    stage "STATUS" "Checking the controller stack and model server..."
    "$runner" status
    "$root_dir/run_eef_pose.sh" server-status
    exit $? ;;
  server)
    stage "SERVER" "Starting or verifying the model server..."
    "$runner" server
    passed "Model server is ready at $EEF_SERVER_HOST:$EEF_SERVER_PORT."
    exit 0 ;;
  server-status)
    stage "SERVER" "Checking the model server..."
    "$runner" server-status
    exit $? ;;
  axis-test)
    stage "AXIS TEST" "Running the commissioned ${4} motion check..."
    log="$FRANKA_LOG_ROOT/${log_prefix}_axis_${4}_$(stamp)_$$"
    env EEF_LOG_DIR="$log" "$enable_var=$enable_token" EEF_TEST_AXIS="$4" "$runner" axis-test
    python3 "$root_dir/scripts/eef_task_config.py" check-axis "$log/policy.jsonl"
    if [[ "$EEF_CONTROL" == direct ]]; then
      echo "$(date -Is) $4 $log" >> "$commissioned"
    fi
    echo "Axis test passed. Log: $log"; exit 0 ;;
  home)
    stage "HOME" "Returning to the configured training start pose..."
    env EEF_LOG_DIR="$FRANKA_LOG_ROOT/${log_prefix}_home_$(stamp)_$$" "$enable_var=$enable_token" "$runner" home
    exit $? ;;
  probe)
    stage "PROBE" "Running synthetic inference without robot motion..."
    "$root_dir/run_eef_pose.sh" probe
    exit $? ;;
esac

if [[ "$EEF_CONTROL" == direct && "$mode" == run && ! -s "$commissioned" && "${EEF_DIRECT_SKIP_COMMISSION:-}" != true ]]; then
  echo "REFUSED: the direct tracker has no successful axis-test record." >&2
  echo "Run: $0 $task_key $model_key axis-test +z" >&2
  exit 3
fi
stage "1/4 启动模型" "正在启动或检查模型推理服务..."
"$runner" server
passed "模型服务已就绪：$EEF_SERVER_HOST:$EEF_SERVER_PORT。"
run_dir="${EEF_RUN_DIR:-$FRANKA_LOG_ROOT/${log_prefix}_$(stamp)_$$}"
mkdir -p "$run_dir"
cp "$temporary/profile.json" "$run_dir/selected_profile.json"
printf '>>> [日志] 本次运行目录：%s\n' "$run_dir"
dry_env=()
if [[ "$EEF_CONTROL" == direct ]]; then
  stack_record="$FRANKA_LOG_ROOT/.eef_direct_stack_log_dir"
  if [[ "$mode" == run ]] && tmux has-session -t franka_vr_eef_direct_robot 2>/dev/null &&
      ! grep -q 'direct executor is ARMED' "$(cat "$stack_record" 2>/dev/null)/robot.log" 2>/dev/null; then
    "$runner" stop
  fi
  [[ "$mode" == run ]] && dry_env=(EEF_DIRECT_STACK=armed "$enable_var=$enable_token")
fi
stage "2/4 安全试运行" "正在执行单个动作块的 dry-run，并校验模型元数据..."
env EEF_LOG_DIR="$run_dir/dry_run" EEF_MAX_CHUNKS=1 "${dry_env[@]}" "$runner" dry-run
python3 "$root_dir/scripts/eef_task_config.py" check-dry-run "$run_dir/dry_run/policy.jsonl"
passed "dry-run 和模型身份校验完成。"

stage "3/4 姿态检查" "正在检查机械臂实测姿态是否与训练初始姿态一致..."
set +e
python3 "$root_dir/scripts/eef_task_config.py" check-start "$run_dir/dry_run/policy.jsonl" \
  --profile "$temporary/profile.json"
start_status=$?
set -e
case "$start_status" in
  0) passed "初始姿态在允许误差范围内。" ;;
  3)
    if [[ "$mode" == dry-run ]]; then
      echo "Pose is away from the configured training start; run would home first."
    elif [[ "$EEF_AUTO_HOME" != true ]]; then
      echo "Pose is away from the configured training start and auto_home=false." >&2; exit 1
    else
      stage "自动回位" "初始姿态超出误差范围，执行前正在自动回位..."
      env EEF_LOG_DIR="$run_dir/home" "$enable_var=$enable_token" "$runner" home
      passed "自动回位完成。"
    fi ;;
  *) echo "Training start-pose verification failed." >&2; exit "$start_status" ;;
esac
if [[ "$mode" == dry-run ]]; then
  passed "dry-run 完成。日志：$run_dir"
  exit 0
fi

stage "4/4 真机执行" "真机控制即将开始，后续将持续输出动作块和首个 Action。"
printf '>>> [运行参数] RTC 预测=%s，执行=%s，融合=%s，动作增益=%s\n' \
  "15" "$EEF_EXECUTE_HORIZON" "$EEF_RTC_BLEND_STEPS" "$EEF_ACTION_GAIN"
printf '>>> [任务指令] %s\n' "$VLA_INSTRUCTION"
printf '>>> [提示] 运行中将显示：动作块、推理耗时、RTC 延迟、平移、旋转和夹爪状态。\n'

execute_dir="$run_dir/execute"
execute_log="$execute_dir/adapter.log"
monitor_fifo="$temporary/action-monitor.fifo"
mkdir -p "$execute_dir"
: > "$execute_log"
rm -f "$monitor_fifo"
mkfifo "$monitor_fifo"
pretty_execute_stream < "$monitor_fifo" &
pretty_monitor_pid=$!
tail -n 0 -F -s 0.1 "$execute_log" > "$monitor_fifo" 2>/dev/null &
pretty_tail_pid=$!
set +e
env EEF_LOG_DIR="$run_dir/execute" "$enable_var=$enable_token" EEF_ENABLE_GRIPPER=true \
  "$runner" execute
execute_status=$?
set -e
sleep 0.2
kill "$pretty_tail_pid" "$pretty_monitor_pid" >/dev/null 2>&1 || true
wait "$pretty_tail_pid" 2>/dev/null || true
wait "$pretty_monitor_pid" 2>/dev/null || true
rm -f "$monitor_fifo"
if (( execute_status == 0 )); then
  passed "执行正常结束。日志：$run_dir/execute"
else
  printf '>>> [异常] 执行退出码=%s。日志：%s\n' "$execute_status" "$run_dir/execute" >&2
fi
exit "$execute_status"
