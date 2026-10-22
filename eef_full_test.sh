#!/usr/bin/env bash
# EEF 完整任务测试（Task0 模型）：dry-run 预检 → 确认 → 真机执行 → 打包日志。
# 预检：夹爪必须张开（训练都从张开开始），起点偏离训练起点 >2 cm 时提醒。
# 执行中可随时 Ctrl-C（Servo 保持当前位置），日志照样打包。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
dir="logs/eef_full_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$dir"
{ git rev-parse --short HEAD; git status --short; git diff; } > "$dir/git_info.txt" 2>&1
export VLA_INSTRUCTION="${EEF_INSTRUCTION:-task 0}" EEF_GRIPPER_STATE_SOURCE=binary
pack() { tar -czf "$dir.tar.gz" -C logs "$(basename "$dir")" && echo "打包完成：$(pwd)/$dir.tar.gz"; }

./run_eef_pose.sh server || { echo "模型服务启动失败"; pack; exit 1; }
EEF_LOG_DIR="$dir/dry_run" EEF_MAX_CHUNKS=1 ./run_eef_pose.sh dry-run
log="$dir/dry_run/policy.jsonl"
width=$(grep -m1 '"event": "observation"' "$log" 2>/dev/null | sed -n 's/.*"gripper_width": \([0-9.e-]*\).*/\1/p')
start=$(grep -m1 '"event": "armed"' "$log" 2>/dev/null | sed -n 's/.*"position": \[\([^]]*\)\].*/\1/p' | tr -d ,)
[[ -n "$width" && -n "$start" ]] || { echo "dry-run 未产生观测，停止"; pack; exit 1; }
awk -v w="$width" 'BEGIN { exit !(w >= 0.06) }' || { echo "夹爪宽度 ${width} m，未张开，停止（先张开夹爪）"; pack; exit 1; }
offset=$(awk -v p="$start" 'BEGIN { split(p, a, " "); printf "%.3f", sqrt((a[1]-0.558)^2 + (a[2]+0.002)^2 + (a[3]-0.320)^2) }')
echo "预检：夹爪 ${width} m，起点 [${start}]，距训练起点 [0.558 -0.002 0.320] ${offset} m"
awk -v d="$offset" 'BEGIN { exit !(d > 0.02) }' && echo "提醒：起点偏离训练起点超过 2 cm"

read -r -p "开始真机完整执行？手放在急停上 [y/N] " answer
[[ "$answer" == y || "$answer" == Y ]] || { echo "已取消"; pack; exit 0; }
trap 'echo "已中断，打包日志"' INT
EEF_LOG_DIR="$dir/execute" EEF_ENABLE_REAL_ROBOT=I_UNDERSTAND_EEF_POSE EEF_ENABLE_GRIPPER=true \
  EEF_MAX_CHUNKS="${EEF_MAX_CHUNKS:-60}" EEF_MAX_ACTIONS="${EEF_MAX_ACTIONS:-900}" ./run_eef_pose.sh execute
pack
