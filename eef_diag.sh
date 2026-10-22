#!/usr/bin/env bash
# EEF 一键诊断：状态探测 → 夹爪开合 → dry-run，最后打包。机械臂全程不动。
# 夹爪会张开/闭合/再张开，开始前确认夹爪里没有东西；EEF_DIAG_GRIPPER=false 跳过。
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
dir="logs/eef_diag_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$dir"
{ git rev-parse --short HEAD; git status --short; git diff --stat; grep -n -i "instruction" run_eef_pose.sh task1-act.sh; } > "$dir/git_info.txt" 2>&1
step() { local name="$1"; shift; echo "=== $name ==="; "$@"; echo "$name exit=$?" | tee -a "$dir/steps.txt"; }

step probe env EEF_LOG_DIR="$dir/probe" ./run_eef_pose.sh probe
[[ "${EEF_DIAG_GRIPPER:-true}" == true ]] && step gripper env EEF_LOG_DIR="$dir/gripper" \
  EEF_GRIPPER_TEST=I_UNDERSTAND_GRIPPER_MOTION EEF_GRIPPER_HOMING="${EEF_GRIPPER_HOMING:-false}" ./run_eef_pose.sh probe
step dry_run env EEF_RUN_DIR="$dir" ./task1-act.sh dry-run

tar -czf "$dir.tar.gz" -C logs "$(basename "$dir")" && echo "打包完成：$(pwd)/$dir.tar.gz" && cat "$dir/steps.txt"
