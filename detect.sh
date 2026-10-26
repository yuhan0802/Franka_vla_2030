#!/usr/bin/env bash
# Read-only preflight for the real-robot FrankaVR three-view VLA deployment.
# It never calls /set_target_pose, publishes a target, or starts a control stack.

set -Eeuo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$ROOT_DIR/scripts/runtime_log_root.sh"
LOG_DIR=${DETECT_LOG_DIR:-$FRANKA_LOG_ROOT}
mkdir -p "$LOG_DIR"
LOG_FILE=${DETECT_LOG_FILE:-$LOG_DIR/detect_$(date +%Y%m%d_%H%M%S).log}
exec > >(tee "$LOG_FILE") 2>&1
FRONT_IMAGE_TOPIC=${FRONT_IMAGE_TOPIC:-${VLA_FRONT_IMAGE_TOPIC:-/camera/camera_front/color/image_raw}}
LEFT_IMAGE_TOPIC=${LEFT_IMAGE_TOPIC:-${VLA_LEFT_IMAGE_TOPIC:-/camera_left/image_raw}}
RIGHT_IMAGE_TOPIC=${RIGHT_IMAGE_TOPIC:-${VLA_RIGHT_IMAGE_TOPIC:-/camera_right/image_raw}}
JOINT_STATE_TOPIC=${JOINT_STATE_TOPIC:-/joint_states}
TF_PARENT=${TF_PARENT:-world}
TF_CHILD=${TF_CHILD:-fr3_link8}
TARGET_SERVICE=${TARGET_SERVICE:-/set_target_pose}
ROBOT_IP=${ROBOT_IP:-172.16.0.2}
FCI_INTERFACE=${FCI_INTERFACE:-enp110s0}
FCI_IRQ_CPU=${FCI_IRQ_CPU:-2}
CONTROL_CPUS=${CONTROL_CPUS:-4,5}
NON_FCI_IRQ_PATTERN=${NON_FCI_IRQ_PATTERN:-nvme|iwlwifi|thunderbolt}
MODEL_HOST=${MODEL_HOST:-${VLA_MODEL_HOST:-10.24.116.29}}
MODEL_PORT=${MODEL_PORT:-${VLA_MODEL_PORT:-8093}}
MODEL_SSH_ALIAS=${MODEL_SSH_ALIAS:-${VLA_MODEL_SSH_ALIAS:-mac_29}}
MODEL_STATUS_CMD=${MODEL_STATUS_CMD:-}
MODEL_IO_CHECK_CMD=${MODEL_IO_CHECK_CMD:-}
MODEL_INSTRUCTION=${MODEL_INSTRUCTION:-pick up the red block}
CALIBRATION_FILE=${CALIBRATION_FILE:-$ROOT_DIR/workspace/src/franka_vr/franka_vr/config/vla_calibration.yaml}
BRIDGE_FILE=${BRIDGE_FILE:-$ROOT_DIR/workspace/src/franka_vr/franka_vr/scripts/franka_vr_vla_bridge.py}
SERVO_CONFIG=${SERVO_CONFIG:-$ROOT_DIR/workspace/src/franka_vr/franka_vr/config/fr3_real_config.yaml}
MODEL_PROBE=${MODEL_PROBE:-$ROOT_DIR/scripts/detect_vla_model_io.py}
MAX_SAFE_X=${MAX_SAFE_X:-0.75}
CHECK_TIMEOUT=${CHECK_TIMEOUT:-10}
SENSOR_TIMEOUT=${SENSOR_TIMEOUT:-15}
MODEL_RESPONSE_TIMEOUT=${MODEL_RESPONSE_TIMEOUT:-90}
# On-site verification recorded by the operator on 2026-10-08.
CONFIRM_CAMERA_MAPPING=${CONFIRM_CAMERA_MAPPING:-true}
CONFIRM_DIRECTION=${CONFIRM_DIRECTION:-true}
CONFIRM_WORKSPACE=${CONFIRM_WORKSPACE:-true}
CONFIRM_SAFETY=${CONFIRM_SAFETY:-true}

PASS=0
FAIL=0
WARN=0
BLOCKED=0
SENSORS_OK=false
ROS_BACKEND=""

pass() { PASS=$((PASS + 1)); printf '[PASS] %s\n' "$*"; }
fail() { FAIL=$((FAIL + 1)); printf '[FAIL] %s\n' "$*"; }
warn() { WARN=$((WARN + 1)); printf '[WARN] %s\n' "$*"; }
blocked() { BLOCKED=$((BLOCKED + 1)); printf '[BLOCKED] %s\n' "$*"; }
section() { printf '\n== %s ==\n' "$*"; }

step_summary() {
    local label=$1 fail_before=$2 warn_before=$3 blocked_before=$4
    if (( FAIL > fail_before )); then
        printf '\n>>>>>>>> %s检测状态：异常 <<<<<<<<\n' "$label"
    elif (( BLOCKED > blocked_before )); then
        printf '\n>>>>>>>> %s检测状态：自动检查正常，现场确认待完成 <<<<<<<<\n' "$label"
    elif (( WARN > warn_before )); then
        printf '\n>>>>>>>> %s检测状态：正常（存在警告） <<<<<<<<\n' "$label"
    else
        printf '\n>>>>>>>> %s检测状态：正常 <<<<<<<<\n' "$label"
    fi
}

run_check_with_summary() {
    local label=$1 function_name=$2
    local fail_before=$FAIL warn_before=$WARN blocked_before=$BLOCKED
    "$function_name"
    step_summary "$label" "$fail_before" "$warn_before" "$blocked_before"
}

usage() {
    cat <<'EOF'
Usage: bash ./detect.sh

Run nine read-only preflight checks for the real FrankaVR VLA deployment.
The script never calls /set_target_pose and never sends robot/gripper commands.

Important overrides:
  FRONT_IMAGE_TOPIC=/camera/camera_front/color/image_raw
  LEFT_IMAGE_TOPIC=/camera_left/image_raw
  RIGHT_IMAGE_TOPIC=/camera_right/image_raw
  JOINT_STATE_TOPIC=/joint_states TF_PARENT=world TF_CHILD=fr3_link8
  TARGET_SERVICE=/set_target_pose
  FCI_INTERFACE=enp110s0 ROBOT_IP=172.16.0.2
  MODEL_HOST=10.24.116.29 MODEL_PORT=8093 MODEL_SSH_ALIAS=mac_29
  MODEL_STATUS_CMD='...'       # optional exact remote process/listener check
  MODEL_IO_CHECK_CMD='...'     # optional replacement for the bundled probe
  MODEL_INSTRUCTION='pick up the red block'
  CHECK_TIMEOUT=10 SENSOR_TIMEOUT=15 MODEL_RESPONSE_TIMEOUT=90

Physical/operator gates (never set these without real on-site verification):
  CONFIRM_CAMERA_MAPPING=true  # serial 43DDCA6F=left, CD7DCA6F=right
  CONFIRM_DIRECTION=true
  CONFIRM_WORKSPACE=true
  CONFIRM_SAFETY=true

Without all physical confirmations, the result is BLOCKED and the exit status
is non-zero even if every automatable check passes.
EOF
}

is_true() {
    case "${1,,}" in
        1|true|yes|y) return 0 ;;
        *) return 1 ;;
    esac
}

operator_gate() {
    local value=$1 label=$2
    if is_true "$value"; then
        pass "$label（现场操作者已显式确认）"
    else
        blocked "$label 未获得现场操作者确认"
    fi
}

run_timeout() { timeout "$CHECK_TIMEOUT" "$@"; }

ros_cmd() {
    if [[ "$ROS_BACKEND" == host ]]; then
        timeout "$CHECK_TIMEOUT" ros2 "$@"
        return
    fi
    local existing_ros_container
    existing_ros_container=$(docker ps -q \
        --filter label=com.docker.compose.service=franka_vr | head -n 1)
    if [[ -n "$existing_ros_container" ]]; then
        timeout "$((CHECK_TIMEOUT + 5))" docker exec \
            -e "DETECT_INNER_TIMEOUT=$CHECK_TIMEOUT" "$existing_ros_container" bash -lc '
                set -e
                source /opt/ros/humble/setup.bash
                if [[ -r /ws_franka_vr/install/setup.bash ]]; then source /ws_franka_vr/install/setup.bash; fi
                exec timeout "$DETECT_INNER_TIMEOUT" ros2 "$@"
            ' detect-ros "$@"
        return
    fi
    timeout "$((CHECK_TIMEOUT + 10))" docker compose \
        --project-directory "$ROOT_DIR" run --rm --no-deps -T --interactive=false \
        -e "DETECT_INNER_TIMEOUT=$CHECK_TIMEOUT" franka_vr bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            if [[ -r /ws_franka_vr/install/setup.bash ]]; then source /ws_franka_vr/install/setup.bash; fi
            exec timeout "$DETECT_INNER_TIMEOUT" ros2 "$@"
        ' detect-ros "$@" </dev/null
}

run_model_probe() {
    local sensors_only=${1:-false}
    local -a probe_args=(
        --host "$MODEL_HOST" --port "$MODEL_PORT"
        --front-topic "$FRONT_IMAGE_TOPIC" --left-topic "$LEFT_IMAGE_TOPIC" --right-topic "$RIGHT_IMAGE_TOPIC"
        --joint-topic "$JOINT_STATE_TOPIC" --tf-parent "$TF_PARENT" --tf-child "$TF_CHILD"
        --instruction "$MODEL_INSTRUCTION" --sensor-timeout "$SENSOR_TIMEOUT"
        --response-timeout "$MODEL_RESPONSE_TIMEOUT"
    )
    [[ "$sensors_only" == true ]] && probe_args+=(--sensors-only)
    if [[ "$ROS_BACKEND" == host ]]; then
        timeout "$((SENSOR_TIMEOUT + MODEL_RESPONSE_TIMEOUT + 15))" python3 "$MODEL_PROBE" "${probe_args[@]}"
        return
    fi
    timeout "$((SENSOR_TIMEOUT + MODEL_RESPONSE_TIMEOUT + 25))" docker compose \
        --project-directory "$ROOT_DIR" run --rm --no-deps -T --interactive=false \
        -v "$ROOT_DIR:/project:ro" franka_vr bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            if [[ -r /ws_franka_vr/install/setup.bash ]]; then source /ws_franka_vr/install/setup.bash; fi
            exec python3 /project/scripts/detect_vla_model_io.py "$@"
        ' detect-model "${probe_args[@]}" </dev/null
}

check_workspace_and_safety() {
    local output
    if [[ ! -r "$CALIBRATION_FILE" || ! -r "$BRIDGE_FILE" || ! -r "$SERVO_CONFIG" ]]; then
        fail "安全配置或桥接器不可读: $CALIBRATION_FILE, $BRIDGE_FILE, $SERVO_CONFIG"
        return
    fi
    if output=$(python3 - "$CALIBRATION_FILE" "$BRIDGE_FILE" "$SERVO_CONFIG" "$MAX_SAFE_X" <<'PY' 2>&1
import math
import re
import sys

calibration_path, bridge_path, servo_path, approved_x_text = sys.argv[1:]
calibration = open(calibration_path, encoding="utf-8").read()
bridge = open(bridge_path, encoding="utf-8").read()
servo = open(servo_path, encoding="utf-8").read()
section = re.search(r"^workspace_bounds_world:\s*$([\s\S]*?)(?=^[^ \t#\n][^:\n]*:\s*$|\Z)", calibration, re.M)
if not section:
    raise SystemExit("missing workspace_bounds_world")

def vector(name):
    match = re.search(rf"^[ \t]+{name}:\s*\[([^\]]+)\]", section.group(1), re.M)
    if not match:
        raise SystemExit(f"missing workspace {name}")
    values = [float(item.strip()) for item in match.group(1).split(",")]
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise SystemExit(f"invalid workspace {name}")
    return values

lower, upper = vector("min"), vector("max")
if any(lo >= hi for lo, hi in zip(lower, upper)):
    raise SystemExit("workspace min must be lower than max on every axis")
approved_x = float(approved_x_text)
if not math.isfinite(approved_x) or upper[0] > approved_x + 1e-9:
    raise SystemExit(f"workspace X max {upper[0]} exceeds approved {approved_x}")
checks = {
    "loads workspace bounds": "workspace_bounds_world" in bridge,
    "rejects out-of-bounds target": "target_position < self.minimum" in bridge and "target_position > self.maximum" in bridge,
    "translation step clipping": "np.clip(raw[:3]*.075, -self.args.max_translation, self.args.max_translation)" in bridge,
    "rotation step clipping": "np.clip(raw[3:6]*.15, -self.args.max_rotation, self.args.max_rotation)" in bridge,
    "dry-run execute gate": "if self.args.execute" in bridge and "--execute" in bridge,
}
missing = [name for name, ok in checks.items() if not ok]
if missing:
    raise SystemExit("bridge safety checks missing: " + ", ".join(missing))

def scalar(name):
    match = re.search(rf"^[ \t]*{re.escape(name)}:\s*([^#\n]+)", servo, re.M)
    return match.group(1).strip() if match else None

if scalar("check_collisions") != "true":
    raise SystemExit("Servo check_collisions is not true")
for name in ("lower_singularity_threshold", "hard_stop_singularity_threshold"):
    value = scalar(name)
    if value is None or not math.isfinite(float(value)) or float(value) <= 0:
        raise SystemExit(f"invalid Servo {name}")
print(f"workspace_min={lower} workspace_max={upper} approved_x_max={approved_x}; bridge_limits=workspace+translation+rotation+dry-run; servo_collision=true singularity={scalar('lower_singularity_threshold')}/{scalar('hard_stop_singularity_threshold')}")
PY
    ); then
        pass "工作区、单步限幅及 Servo 安全配置有效: $output"
    else
        fail "工作区或安全限制检查失败: $output"
    fi
}

if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
    usage
    exit 0
elif [[ $# -gt 0 ]]; then
    usage >&2
    exit 2
fi
if ! [[ "$MODEL_PORT" =~ ^[0-9]+$ && "$CHECK_TIMEOUT" =~ ^[0-9]+$ && "$SENSOR_TIMEOUT" =~ ^[0-9]+$ && "$MODEL_RESPONSE_TIMEOUT" =~ ^[0-9]+$ ]]; then
    printf '[FAIL] 端口和超时必须是正整数\n' >&2
    exit 2
fi

cd "$ROOT_DIR"
printf '\n============================================================\n'
printf '          FrankaVR VLA 真机系统安全检测\n'
printf '============================================================\n'
printf '日志文件: %s\n' "$LOG_FILE"
printf '安全保证：本脚本不会调用 %s，也不会启动 execute。\n' "$TARGET_SERVICE"

check_environment() {
section "检测准备"
for command_name in timeout docker ssh ip python3; do
    if command -v "$command_name" >/dev/null 2>&1; then pass "命令可用: $command_name"; else fail "缺少命令: $command_name"; fi
done
if command -v ros2 >/dev/null 2>&1; then
    ROS_BACKEND=host
    pass "ROS 2 查询后端: 宿主机"
elif docker image inspect franka-vr:humble >/dev/null 2>&1; then
    ROS_BACKEND=container
    pass "ROS 2 查询后端: franka-vr:humble 只读临时容器"
else
    fail "宿主机无 ros2，且 franka-vr:humble 镜像不可用"
fi
}

check_single_real_stack() {
section "6/9 仅运行一个真实控制栈"
running_containers=$(docker ps --format '{{.ID}} {{.Names}} {{.Image}} {{.Command}}' 2>/dev/null || true)
conflict_containers=$(printf '%s\n' "$running_containers" | grep -Ei 'gazebo|gzserver|ignition|fake[_ .-]?hardware|mock' || true)
if [[ -n "$conflict_containers" ]]; then fail "检测到 Gazebo/fake/mock 容器: $conflict_containers"; else pass "未检测到 Gazebo/fake/mock 容器"; fi
real_stack_containers=$(docker ps \
    --filter label=com.docker.compose.service=franka_robot \
    --format '{{.ID}} {{.Names}} {{.Image}}' 2>/dev/null || true)
real_stack_count=$(printf '%s\n' "$real_stack_containers" | sed '/^[[:space:]]*$/d' | wc -l)
if [[ "$real_stack_count" -gt 1 ]]; then
    fail "检测到多个真机控制栈容器: $real_stack_containers"
elif [[ "$real_stack_count" -eq 1 ]]; then
    pass "仅检测到一个真机控制栈容器: $real_stack_containers"
else
    warn "未检测到正在运行的真机控制栈容器"
fi

if [[ -n "$ROS_BACKEND" ]]; then
    ros_nodes=$(ros_cmd node list 2>/dev/null || true)
    ros_conflicts=$(printf '%s\n' "$ros_nodes" | grep -Ei 'gazebo|gzserver|ignition|fake[_ .-]?hardware|mock' || true)
    if [[ -n "$ros_conflicts" ]]; then fail "检测到 Gazebo/fake/mock ROS 节点: $ros_conflicts"; else pass "未检测到 Gazebo/fake/mock ROS 节点"; fi
    manager_count=$(printf '%s\n' "$ros_nodes" | grep -Ec '(^|/)controller_manager$' || true)
    if [[ "$manager_count" -eq 1 ]]; then
        pass "仅存在一个 controller_manager"
    elif [[ "$manager_count" -eq 0 ]]; then
        fail "未发现 controller_manager（真机状态栈未运行）"
    else
        fail "发现 $manager_count 个 controller_manager，疑似多个控制栈"
    fi
fi
}

check_fci() {
section "7/9 使用专用 FCI 网口"
if ! ip link show dev "$FCI_INTERFACE" >/dev/null 2>&1; then
    fail "FCI 网口不存在: $FCI_INTERFACE"
else
    pass "FCI 网口存在: $FCI_INTERFACE"
    route=$(ip route get "$ROBOT_IP" 2>/dev/null || true)
    if printf '%s\n' "$route" | grep -Eq "(^|[[:space:]])dev[[:space:]]+$FCI_INTERFACE([[:space:]]|$)"; then pass "机器人路由经专用 FCI 网口: $route"; else fail "机器人路由未经过 $FCI_INTERFACE: ${route:-无路由}"; fi
    link_info=$(ethtool "$FCI_INTERFACE" 2>/dev/null || true)
    link_detected=$(awk -F': *' '/^[[:space:]]*Link detected:/{print $2; exit}' <<<"$link_info")
    link_speed=$(awk -F': *' '/^[[:space:]]*Speed:/{print $2; exit}' <<<"$link_info")
    link_duplex=$(awk -F': *' '/^[[:space:]]*Duplex:/{print $2; exit}' <<<"$link_info")
    if [[ "$link_detected" == yes && "$link_duplex" == Full ]]; then pass "FCI 链路已连接: speed=${link_speed:-unknown}, duplex=$link_duplex"; else fail "FCI 链路异常: link=${link_detected:-unknown}, speed=${link_speed:-unknown}, duplex=${link_duplex:-unknown}"; fi
    if timeout 3 ping -c 1 -W 2 -I "$FCI_INTERFACE" "$ROBOT_IP" >/dev/null 2>&1; then pass "机器人 IP 从 FCI 网口可达: $ROBOT_IP"; else fail "机器人 IP 无法从 $FCI_INTERFACE 到达: $ROBOT_IP"; fi
fi
if [[ "$(cat /sys/kernel/realtime 2>/dev/null || true)" == 1 ]]; then pass "PREEMPT_RT 已启用"; else warn "未检测到 /sys/kernel/realtime=1；需结合已验证的低延迟内核判断"; fi
irq_conflicts=$(python3 - "$CONTROL_CPUS" "$NON_FCI_IRQ_PATTERN" <<'PY'
import re
import sys
import time
from pathlib import Path

control_text, pattern = sys.argv[1:]

def cpus(text):
    result = set()
    for item in text.strip().split(","):
        if not item:
            continue
        if "-" in item:
            start, end = (int(value) for value in item.split("-", 1))
            result.update(range(start, end + 1))
        else:
            result.add(int(item))
    return result

control = cpus(control_text)

def snapshot():
    result = {}
    for line in Path("/proc/interrupts").read_text().splitlines():
        if not re.search(pattern, line, re.I) or ":" not in line:
            continue
        irq, remainder = line.split(":", 1)
        irq = irq.strip()
        path = Path(f"/proc/irq/{irq}/effective_affinity_list")
        if not irq.isdigit() or not path.exists():
            continue
        effective_text = path.read_text().strip()
        if not cpus(effective_text) & control:
            continue
        fields = remainder.split()
        counts = []
        for field in fields:
            if not field.isdigit():
                break
            counts.append(int(field))
        result[irq] = (effective_text, sum(counts), fields[-1])
    return result

before = snapshot()
time.sleep(2.0)
after = snapshot()
for irq, (effective, count, device) in after.items():
    delta = count - before.get(irq, (effective, count, device))[1]
    print(f"irq={irq} effective={effective} delta={delta} source={device}")
PY
)
if [[ -n "$irq_conflicts" ]]; then
    if printf '%s\n' "$irq_conflicts" | grep -Eq 'delta=[1-9][0-9]*'; then
        fail "非 FCI 中断正在实时控制核 $CONTROL_CPUS 上触发: $irq_conflicts"
    elif grep -Eq 'isolcpus=[^ ]*managed_irq' /proc/cmdline; then
        pass "managed_irq 隔离生效；托管 IRQ mask 虽含控制核，但采样期无实际中断: $irq_conflicts"
    else
        fail "非 FCI 中断占用实时控制核 $CONTROL_CPUS，且未启用 managed_irq: $irq_conflicts"
    fi
else
    pass "NVMe/Wi-Fi/Thunderbolt 中断未占用实时控制核 $CONTROL_CPUS"
fi
fci_irq=$(grep -m1 "$FCI_INTERFACE" /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' ' || true)
fci_irq_effective=""
if [[ -n "$fci_irq" ]]; then
    fci_irq_effective=$(cat "/proc/irq/$fci_irq/effective_affinity_list" 2>/dev/null || true)
fi
if [[ -n "$fci_irq" && "$fci_irq_effective" == "$FCI_IRQ_CPU" ]]; then
    pass "FCI IRQ $fci_irq 固定到专用 CPU $FCI_IRQ_CPU"
else
    fail "FCI IRQ affinity 异常: irq=${fci_irq:-unknown}, effective=${fci_irq_effective:-unknown}, expected=$FCI_IRQ_CPU"
fi
}

check_sensors() {
if [[ -z "$ROS_BACKEND" || ! -r "$MODEL_PROBE" ]]; then
    sensor_log=$(mktemp)
    printf 'DETECT_ERROR 无法运行只读传感器探针: %s\n' "$MODEL_PROBE" >"$sensor_log"
else
    sensor_log=$(mktemp)
    if run_model_probe true >"$sensor_log" 2>&1; then
        SENSORS_OK=true
    fi
fi

section "1/9 Intel RealSense D435i 正视角图像"
local fail_before=$FAIL warn_before=$WARN blocked_before=$BLOCKED
if grep -F 'SENSOR_IMAGE key=front ' "$sensor_log"; then
    pass "D435i 正视角图像新鲜、可解码为 RGB8"
else
    sed -n '1,40p' "$sensor_log"
    fail "D435i 正视角图像异常: $FRONT_IMAGE_TOPIC"
fi
step_summary "第1项" "$fail_before" "$warn_before" "$blocked_before"

section "2/9 两台 Logitech C922 左、右视角图像"
fail_before=$FAIL; warn_before=$WARN; blocked_before=$BLOCKED
left_image_ok=false
right_image_ok=false
grep -F 'SENSOR_IMAGE key=left ' "$sensor_log" && left_image_ok=true
grep -F 'SENSOR_IMAGE key=right ' "$sensor_log" && right_image_ok=true
if [[ "$left_image_ok" == true && "$right_image_ok" == true ]]; then
    pass "C922 左、右视角图像均新鲜且可解码为 RGB8"
else
    fail "C922 左或右视角图像异常: left=$LEFT_IMAGE_TOPIC, right=$RIGHT_IMAGE_TOPIC"
fi
printf '配置映射: left=43DDCA6F (%s), right=CD7DCA6F (%s)\n' "$LEFT_IMAGE_TOPIC" "$RIGHT_IMAGE_TOPIC"
operator_gate "$CONFIRM_CAMERA_MAPPING" "C922 左右物理视角及训练输入顺序"
step_summary "第2项" "$fail_before" "$warn_before" "$blocked_before"

section "3/9 实体机器人当前关节状态与 world -> fr3_link8 TF"
fail_before=$FAIL; warn_before=$WARN; blocked_before=$BLOCKED
joint_ok=false
tf_ok=false
grep -F 'SENSOR_JOINTS ' "$sensor_log" && joint_ok=true
grep -F 'SENSOR_TF parent=world child=fr3_link8 ' "$sensor_log" && tf_ok=true
if [[ "$joint_ok" == true && "$tf_ok" == true ]]; then
    pass "fr3_joint1..7、8 维状态及持续更新的 world -> fr3_link8 TF 均正常"
else
    fail "关节状态或 world -> fr3_link8 TF 异常；禁止模型推理"
fi
step_summary "第3项" "$fail_before" "$warn_before" "$blocked_before"
rm -f "$sensor_log"
}

check_ai_communication() {
section "4/9 AI 服务器与 Ubuntu 控制计算机的模型通信状态"
if nc -z -w 3 "$MODEL_HOST" "$MODEL_PORT" >/dev/null 2>&1; then pass "模型 TCP 端口可达: $MODEL_HOST:$MODEL_PORT（仅网络证据）"; else fail "模型 TCP 端口不可达: $MODEL_HOST:$MODEL_PORT"; fi
if [[ -n "$MODEL_STATUS_CMD" ]]; then
    if run_timeout bash -lc "$MODEL_STATUS_CMD"; then pass "自定义模型进程/监听检查通过"; else fail "自定义模型进程/监听检查失败"; fi
elif run_timeout ssh -o BatchMode=yes -o ConnectTimeout=5 "$MODEL_SSH_ALIAS" \
    "pgrep -af deploy_franka_vr_qwen_actioncodec.py | grep -F -- '--port $MODEL_PORT' >/dev/null && ss -ltn | grep -Eq '[:.]$MODEL_PORT[[:space:]]'"; then
    pass "远端正确 FrankaVR 服务进程监听 $MODEL_PORT: $MODEL_SSH_ALIAS"
else
    fail "远端未确认正确 FrankaVR 进程监听 $MODEL_PORT（TCP 可达不能替代此检查）"
fi
}

check_robot_communication() {
section "5/9 Ubuntu 控制机与实体机器人控制通信"
if [[ -n "$ROS_BACKEND" ]]; then
    services=$(ros_cmd service list -t 2>/dev/null || true)
    if printf '%s\n' "$services" | grep -Fq '/controller_manager/list_controllers'; then pass "controller_manager/list_controllers 服务可用"; else fail "controller_manager/list_controllers 服务不可用"; fi
    if printf '%s\n' "$services" | grep -F "$TARGET_SERVICE" | grep -Fq '[franka_vr/srv/SetTargetPose]'; then pass "$TARGET_SERVICE 是有效 SetTargetPose service（只检查，未调用）"; else fail "$TARGET_SERVICE SetTargetPose service 不存在或类型错误（未调用）"; fi
    controllers=""
    for attempt in 1 2 3; do
        controllers=$(ros_cmd control list_controllers 2>/dev/null \
            | sed $'s/\033\\[[0-9;]*m//g' || true)
        if printf '%s\n' "$controllers" | grep -Eq '^joint_state_broadcaster[[:space:]].*[[:space:]]active$' \
            && printf '%s\n' "$controllers" | grep -Eq '^fr3_arm_controller[[:space:]].*[[:space:]]active$'; then
            break
        fi
        [[ "$attempt" -lt 3 ]] && sleep 1
    done
    for controller in joint_state_broadcaster fr3_arm_controller; do
        if printf '%s\n' "$controllers" | grep -Eq "^$controller[[:space:]].*[[:space:]]active$"; then pass "控制器 active: $controller"; else fail "控制器未处于 active: $controller"; fi
    done
else
    fail "ROS 2 查询后端不可用，无法验证控制通信"
fi
}

check_model_io_and_workspace() {
section "8/9 模型输入输出及机器人工作区安全限制"
if [[ "$SENSORS_OK" != true ]]; then
    blocked "传感器、关节或 TF 未通过，未向模型发送请求"
elif [[ -n "$MODEL_IO_CHECK_CMD" ]]; then
    model_log=$(mktemp)
    if timeout "$((SENSOR_TIMEOUT + MODEL_RESPONSE_TIMEOUT + 15))" bash -lc "$MODEL_IO_CHECK_CMD" >"$model_log" 2>&1; then cat "$model_log"; pass "自定义模型 I/O 探针通过"; else sed -n '1,80p' "$model_log"; fail "自定义模型 I/O 探针失败"; fi
    rm -f "$model_log"
else
    model_log=$(mktemp)
    if run_model_probe false >"$model_log" 2>&1; then cat "$model_log"; pass "模型身份正确，响应为有限 float32[15,7]；action 未执行"; else sed -n '1,100p' "$model_log"; fail "真实模型 I/O 探针失败；action 未执行"; fi
    rm -f "$model_log"
fi

check_workspace_and_safety
}

check_motion_workspace_safety() {
section "9/9 机器人运动方向、实测工作空间及安全保护设置"
operator_gate "$CONFIRM_DIRECTION" "xyz/rpy 运动方向"
operator_gate "$CONFIRM_WORKSPACE" "实测工作空间边界（未为追求范围而放宽未知区域）"
operator_gate "$CONFIRM_SAFETY" "急停、清场、低速空载及碰撞/奇异性保护"
}

check_environment
check_sensors
run_check_with_summary "第4项" check_ai_communication
run_check_with_summary "第5项" check_robot_communication
run_check_with_summary "第6项" check_single_real_stack
run_check_with_summary "第7项" check_fci
run_check_with_summary "第8项" check_model_io_and_workspace
run_check_with_summary "第9项" check_motion_workspace_safety

section "结果"
printf '通过: %s  失败: %s  警告: %s  阻塞: %s\n' "$PASS" "$FAIL" "$WARN" "$BLOCKED"
printf '只读声明: 未调用 %s，未发布运动目标，未执行任何返回的 action。\n' "$TARGET_SERVICE"
if [[ "$FAIL" -eq 0 && "$BLOCKED" -eq 0 ]]; then
    printf '\n>>>>>>>> 最终自动检测状态：正常 <<<<<<<<\n'
    printf '检测通过；只允许由现场操作者按既定流程决定是否进入下一步 dry-run。\n'
    exit 0
elif [[ "$FAIL" -eq 0 ]]; then
    printf '\n>>>>>>>> 最终自动检测状态：正常 <<<<<<<<\n'
    printf '检测结果 BLOCKED：自动检查无失败，但现场确认未完成；禁止 execute。\n'
    exit 2
else
    printf '检测失败：禁止进入 execute；请按以上证据修复后重跑。\n'
    exit 1
fi
