#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/scripts/runtime_log_root.sh"

# 批量测试：模型服务只部署一次，反复测试不再重新部署，stop 也不会关掉模型。
#
# 每一次试验都复用 task1-act.sh / task1-act-v2.sh 的配置和执行流程（dry-run 校验、回起点、direct 跟踪器、
# 夹爪、安全检查全部相同），本脚本只负责：
#   1. 先检查模型服务是否已经部署（端口在听、权重一致），没有才部署；
#   2. 连续跑 N 次，每次之间提示复位试管，记录每次的结果到 results.csv；
#   3. stop 只停机器人侧（零速度、关栈），模型服务保留；要关模型用 server-stop。
#
# 选模型：EEF_TEST_MODEL=maxabs（默认，v5_maxabs save_8000，63 服务器，task1-act-v2.sh 的配置）
#         EEF_TEST_MODEL=x3     （v5_x3 save_8000，20 服务器，task1-act.sh 的配置）
# 权重、服务器、端口等固定从 config/eef_tasks.json 读取，与通用入口保持一致。
#
#   ./task1-test.sh server           只检查/部署模型服务，不动机械臂（测试前先跑一次，省掉每次的等待）
#   ./task1-test.sh status           模型服务是否在跑、跑的是哪个权重
#   ./task1-test.sh axis-test +z     首次使用 direct 先逐轴调试（同 task1-act.sh）
#   ./task1-test.sh dry-run          只推理，不发运动命令
#   ./task1-test.sh                  跑 1 次完整试验
#   ./task1-test.sh run 10           连续跑 10 次（每次之间提示复位，EEF_TEST_PAUSE=0 关闭提示）
#   ./task1-test.sh summary [目录]   汇总最近一批（或指定批次目录）的 results.csv
#   ./task1-test.sh home | probe     同 task1-act.sh
#   ./task1-test.sh stop             只停机器人侧，不关模型
#   ./task1-test.sh server-stop      关掉模型服务（需要换权重或不再测试时才用）
#
# 结果记录在 logs/eef_task1_test_<模型>_<时间>/：results.csv 以及每次试验的 trial_NN/ 日志。
# 模型服务已在跑但权重和当前配置不一致时默认拒绝（可能有人在用）；确认要换：EEF_TEST_REDEPLOY=1。
model="${EEF_TEST_MODEL:-maxabs}"
case "$model" in
  maxabs) model_key=act-v2; base="$script_dir/task1-act-v2.sh";;
  x3)     model_key=act;    base="$script_dir/task1-act.sh";;
  *) echo "EEF_TEST_MODEL must be maxabs or x3" >&2; exit 2;;
esac
base="${EEF_TEST_BASE_SCRIPT:-$base}"   # 仅供离线测试替换
[[ -f "$base" ]] || { echo "missing $base" >&2; exit 2; }

# 与 run_eef_task.sh 使用同一个固定模型配置，避免包装脚本改动后参数漂移。
profile_tmp="$(mktemp -d "${TMPDIR:-/tmp}/eef_test_profile.XXXXXXXX")"
python3 "$script_dir/scripts/eef_task_config.py" resolve \
  --config "${EEF_TASK_CONFIG:-$script_dir/config/eef_tasks.json}" \
  --task task1 --model "$model_key" \
  --output "$profile_tmp/profile.json" --env-file "$profile_tmp/env"
while IFS= read -r -d '' key && IFS= read -r -d '' value; do export "$key=$value"; done < "$profile_tmp/env"
rm -rf -- "$profile_tmp"
: "${EEF_REMOTE_SSH:?}" "${EEF_SERVER_PORT:?}" "${EEF_CHECKPOINT:?}"
runner="$script_dir/run_eef_direct.sh"
stamp() { date +%Y%m%d_%H%M%S; }
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=10)

server_listening() { ssh "${ssh_opts[@]}" "$EEF_REMOTE_SSH" "ss -ltn | grep -q ':$EEF_SERVER_PORT '" 2>/dev/null; }

# 正在跑的服务的 --checkpoint（取自进程命令行）；查不到时输出空。
deployed_checkpoint() {
  ssh "${ssh_opts[@]}" "$EEF_REMOTE_SSH" bash -s 2>/dev/null <<EOF || true
ps -eo args | grep -E -- "--port $EEF_SERVER_PORT( |\$)" | grep -v grep | head -1 | tr ' ' '\n' | grep -x -A1 -- '--checkpoint' | tail -1
EOF
}

ensure_server() {
  if server_listening; then
    local running; running="$(deployed_checkpoint)"
    if [[ -z "$running" ]]; then
      echo "[model] $EEF_REMOTE_SSH:$EEF_SERVER_PORT 已有服务在听（查不到权重参数，由 adapter 启动时校验）。"
    elif [[ "$running" == "$EEF_CHECKPOINT" ]]; then
      echo "[model] 已部署，直接复用：$EEF_REMOTE_SSH:$EEF_SERVER_PORT  $(basename "$running")"
    elif [[ "${EEF_TEST_REDEPLOY:-0}" == 1 ]]; then
      echo "[model] 正在跑的是另一个权重：$running"; echo "[model] EEF_TEST_REDEPLOY=1，重新部署为 $EEF_CHECKPOINT"
      "$script_dir/run_eef_pose.sh" server-stop; "$runner" server
    else
      echo "[model] $EEF_REMOTE_SSH:$EEF_SERVER_PORT 上正在跑的不是当前配置的权重，拒绝覆盖（可能有人在用）：" >&2
      echo "          正在跑：$running" >&2; echo "          当前配置：$EEF_CHECKPOINT" >&2
      echo "        确认要换：EEF_TEST_REDEPLOY=1 $0 ...    或先 $0 server-stop" >&2; return 1
    fi
  else
    echo "[model] $EEF_REMOTE_SSH:$EEF_SERVER_PORT 没有部署，现在部署：$(basename "$EEF_CHECKPOINT")"
    "$runner" server
  fi
}

# 记录一次试验：解析 execute/policy.jsonl，追加到 results.csv，并打印一行摘要。
record_trial() {  # trial run_dir rc label csv
  python3 - "$@" <<'PY'
import csv, json, math, os, sys
trial, run_dir, rc, label, path = sys.argv[1:6]
log = os.path.join(run_dir, "execute", "policy.jsonl")
events = []
if os.path.exists(log):
    with open(log) as handle:
        for line in handle:
            try: events.append(json.loads(line))
            except ValueError: pass
count = lambda name: sum(1 for e in events if e.get("event") == name)
end = next((e for e in reversed(events) if e.get("event") in ("complete", "fault")), None)
if not events: result, reason = "no_execute_log", "dry-run 或启动阶段就失败了（见 trial 目录下的 dry_run/ 日志）"
elif end is None: result, reason = "interrupted", "没有 complete/fault 事件（被中断或超时）"
else: result, reason = end["event"], str(end.get("reason", ""))
obs = [e for e in events if e.get("event") == "observation"]
closes = [e for e in events if e.get("event") == "gripper_request" and e.get("open") is False]
opens = [e for e in events if e.get("event") == "gripper_request" and e.get("open") is True]
close_xyz = ""
if closes and obs:
    before = [o for o in obs if o["monotonic_s"] <= closes[0]["monotonic_s"]] or obs[:1]
    close_xyz = " ".join(f"{v:.3f}" for v in before[-1]["eef_state"][:3])
zs = [o["eef_state"][2] for o in obs]
duration = events[-1]["monotonic_s"] - events[0]["monotonic_s"] if len(events) > 1 else 0.0
row = dict(trial=trial, result=result, reason=reason[:120], chunks=count("chunk"), actions=count("action"),
           close_requests=len(closes), open_requests=len(opens), first_close_xyz=close_xyz,
           min_z=f"{min(zs):.3f}" if zs else "", duration_s=f"{duration:.0f}", script_rc=rc, grasp_ok=label, run_dir=run_dir)
new = not os.path.exists(path)
with open(path, "a", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(row));
    if new: writer.writeheader()
    writer.writerow(row)
print(f"[trial {trial}] {result}  chunks={row['chunks']} close={row['close_requests']} open={row['open_requests']} "
      f"close_at=({close_xyz or '-'}) min_z={row['min_z'] or '-'} {row['duration_s']}s  夹到={label or '-'}  {reason[:80]}")
PY
}

summarize() {
  local csv_path="${1:-}"
  if [[ -z "$csv_path" ]]; then
    csv_path="$(ls -td "$FRANKA_LOG_ROOT"/eef_task1_test_*/ 2>/dev/null | head -1 || true)"results.csv
  elif [[ -d "$csv_path" ]]; then csv_path="${csv_path%/}/results.csv"; fi
  [[ -f "$csv_path" ]] || { echo "没有找到 results.csv：$csv_path" >&2; return 1; }
  python3 - "$csv_path" <<'PY'
import csv, sys, collections
rows = list(csv.DictReader(open(sys.argv[1])))
print(f"{sys.argv[1]}  共 {len(rows)} 次")
for r in rows:
    print(f"  #{r['trial']:>2} {r['result']:<14} close={r['close_requests']} open={r['open_requests']} close_at=({r['first_close_xyz'] or '-'}) "
          f"min_z={r['min_z'] or '-'} {r['duration_s']}s 夹到={r['grasp_ok'] or '-'}  {r['reason'][:60]}")
labeled = [r for r in rows if r["grasp_ok"] in ("y", "n")]
if labeled:
    ok = sum(r["grasp_ok"] == "y" for r in labeled)
    print(f"人工标注：夹到 {ok}/{len(labeled)}（{100 * ok / len(labeled):.0f}%）")
print("脚本判定：" + ", ".join(f"{k} {v}" for k, v in collections.Counter(r["result"] for r in rows).items()))
PY
}

mode="${1:-run}"
case "$mode" in
  server)
    ensure_server; echo "[model] 就绪。之后 $0 run N 会直接复用，stop 不会关它。" ;;
  status)
    if server_listening; then
      running="$(deployed_checkpoint)"
      echo "[model] 在跑：$EEF_REMOTE_SSH:$EEF_SERVER_PORT  ${running:-(查不到权重参数)}"
      [[ -z "$running" || "$running" == "$EEF_CHECKPOINT" ]] && echo "[model] 与当前配置（${model}）一致" || echo "[model] 与当前配置（${model}）不一致：$EEF_CHECKPOINT"
    else echo "[model] 没有部署：$EEF_REMOTE_SSH:$EEF_SERVER_PORT"; fi ;;
  summary) summarize "${2:-}" ;;
  stop|STOP)
    # 只停机器人侧；故意不调用 server-stop，模型服务保留。
    stop_rc=0
    "$runner" stop || stop_rc=1
    "$script_dir/run_eef_pose.sh" stop || stop_rc=1
    if (( stop_rc == 0 )); then echo "[stop] 机器人侧已停止；模型服务未动（要关：$0 server-stop）。"
    else echo "[stop] 警告：机器人侧停止命令返回了错误（见上方输出），请现场确认机械臂已停；模型服务未动。" >&2; fi ;;
  server-stop)
    "$script_dir/run_eef_pose.sh" server-stop ;;
  axis-test|home|probe|dry-run)
    ensure_server; exec "$base" "$@" ;;
  run)
    trials="${2:-${EEF_TEST_TRIALS:-1}}"
    [[ "$trials" =~ ^[1-9][0-9]*$ ]] || { echo "Usage: $0 run [次数]" >&2; exit 2; }
    batch_dir="$FRANKA_LOG_ROOT/eef_task1_test_${model}_$(stamp)"; mkdir -p "$batch_dir"
    results="$batch_dir/results.csv"
    interactive=false; [[ -t 0 && "${EEF_TEST_PAUSE:-1}" != 0 ]] && interactive=true
    echo "[batch] 模型=$model  次数=$trials  记录目录=$batch_dir"
    # Ctrl-C：停机器人侧（零速度），保留模型，打印已完成试验的汇总。
    trap 'echo; echo "[batch] 中断：停止机器人侧（模型服务保留）"; "$runner" stop || true; summarize "$results" || true; exit 130' INT TERM
    ensure_server
    for ((i = 1; i <= trials; i++)); do
      trial="$(printf %02d "$i")"; run_dir="$batch_dir/trial_$trial"
      echo; echo "=============== 试验 $i / $trials ==============="
      ensure_server || { echo "[batch] 模型服务不可用，批量终止。" >&2; break; }   # 每次都复查，服务被人关掉会自动重部署
      set +e; env EEF_RUN_DIR="$run_dir" EEF_LOG_PREFIX=eef_task1_test "$base" run; rc=$?; set -e
      if [[ $rc -eq 3 ]]; then echo "[batch] 基础脚本拒绝执行（rc=3，多半是 direct 还没通过 axis-test）；批量终止。" >&2; break; fi
      label=""
      if $interactive; then
        read -r -p "[trial $trial] 夹到试管了吗？ y=夹到 n=没夹到 回车=跳过  > " label || label=""
        [[ "$label" == y || "$label" == n ]] || label=""
      fi
      record_trial "$trial" "$run_dir" "$rc" "$label" "$results"
      if (( i < trials )); then
        if $interactive; then
          read -r -p "[batch] 复位试管/试管架，准备好后回车开始第 $((i + 1)) 次（q 结束）> " answer || answer=q
          [[ "$answer" == q ]] && break
        else
          echo "[batch] EEF_TEST_PAUSE=0 或非交互终端：不等待复位，直接开始下一次。"
        fi
      fi
    done
    trap - INT TERM
    echo; summarize "$results" ;;
  *) echo "Usage: $0 [server|status|dry-run|axis-test <axis>|run [N]|summary [目录]|home|probe|stop|server-stop]" >&2; exit 2 ;;
esac
