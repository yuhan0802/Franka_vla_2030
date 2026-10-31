#!/usr/bin/env bash
# Offline gate for an EEF checkpoint (run on the GPU host that holds the checkpoint).
# Usage: gate.sh CHECKPOINT GPU PORT OUTPUT_DIR
# Starts deploy_franka_task0_qwen_actioncodec_eef.py (copy next to this script), runs the image-swap
# test and the closed-loop rollout, writes summary.json, then stops the server.
# Pass criteria (engineering thresholds, see docs/task1_eef_failure_analysis_20261030.md):
#   image_reliance >= 0.5 and closed-loop over_30mm == 0 and within_20mm >= 8 of 10.
set -euo pipefail
checkpoint="$1"; gpu="$2"; port="$3"; out="$4"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# In the repo the deploy script sits two levels up; on a GPU host copy it next to this script.
deploy="$here/deploy_franka_task0_qwen_actioncodec_eef.py"
[[ -f "$deploy" ]] || deploy="$here/../../deploy_franka_task0_qwen_actioncodec_eef.py"
deploy_dir="$(cd "$(dirname "$deploy")" && pwd)"
P=/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA
# Use the env the checkpoint was trained with: act-v2 era checkpoints need transformers 5.3
# (GATE_PYTHON=$P/.conda/qwenvla/bin/python); the anaconda env has 4.57.
PY="${GATE_PYTHON:-/home/xmu/anaconda3/envs/qwenvla/bin/python}"
# Target-conditioned runs keep their tube/basket targets next to the run (the launcher copies them).
run_targets="$(dirname "$(dirname "$checkpoint")")/targets.json"
[[ -f "$run_targets" ]] && export EVAL_TARGETS_JSON="$run_targets" && echo "targets: $run_targets"
mkdir -p "$out"
session="offline_gate_$port"
tmux kill-session -t "$session" 2>/dev/null || true
# A previous server on this port may still be exiting; its socket would pass the readiness check below.
for _ in $(seq 1 60); do ss -ltn | grep -q ":$port " || break; sleep 2; done
ss -ltn | grep -q ":$port " && { echo "port $port still in use"; exit 1; }
tmux new-session -d -s "$session" "cd '$deploy_dir' && CUDA_VISIBLE_DEVICES='$gpu' PYTHONPATH='$P/third_party/python:$P' \
  VLA_PROJECT_ROOT='$P' HF_HOME='$P/actioncodec_libero/.cache/huggingface' USE_TF=0 TRANSFORMERS_NO_TF=1 \
  TOKENIZERS_PARALLELISM=false '$PY' -u deploy_franka_task0_qwen_actioncodec_eef.py --checkpoint '$checkpoint' \
  --port '$port' --rotation-execution relative_rotvec_left_multiply > '$out/server.log' 2>&1"
trap 'tmux kill-session -t "$session" 2>/dev/null || true' EXIT
for _ in $(seq 1 120); do
  ss -ltn | grep -q ":$port " && break
  tmux has-session -t "$session" 2>/dev/null || { echo "server died:"; tail -20 "$out/server.log"; exit 1; }
  sleep 5
done
ss -ltn | grep -q ":$port " || { echo "server did not start"; exit 1; }
grep -o "'state_input': '[a-z_]*'" "$out/server.log" || true
cd "$here"
# SIM_ONLY=1 skips the swap test (reuses swap.json if present); SIM_ARGS go to closed_loop_sim.py.
[[ "${SIM_ONLY:-0}" == 1 ]] || "$PY" swap_test.py --port "$port" --output "$out/swap.json" | tail -1
"$PY" closed_loop_sim.py --port "$port" --output "$out/closed_loop.json" ${SIM_ARGS:-} | tail -11
"$PY" - "$out" <<'PY'
import json, os, sys
out = sys.argv[1]
swap = json.load(open(f"{out}/swap.json"))["summary"] if os.path.exists(f"{out}/swap.json") else {"image_reliance": None}; loop = json.load(open(f"{out}/closed_loop.json"))["summary"]
passed = (swap["image_reliance"] or 0) >= 0.5 and loop["over_30mm"] == 0 and loop["within_20mm"] >= 8
json.dump(dict(swap=swap, closed_loop=loop, passed=passed), open(f"{out}/summary.json", "w"), indent=1)
print("GATE", "PASS" if passed else "FAIL", json.dumps(dict(swap=swap, closed_loop=loop)))
PY
