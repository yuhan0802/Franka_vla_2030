#!/usr/bin/env bash
# Training watchdog on mac_20 (tmux claude_overnight): every 2 min
#  1. relaunch a crashed act-v2 continuation run from its newest checkpoint (max 3 restarts per run);
#  2. gate every new checkpoint of every act-v2 continuation run (transformers 5.3 env, 8 threads).
# Runs are listed in ~/offline_eval/runs.conf as: TAG|launcher env|AFTER (TAG must match the env's TAG=;
# AFTER = space-separated tags that must reach their final save first, for runs queued on busy GPUs).
# Log: ~/offline_eval/results/overnight.log
set -u
P=/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA
R=/data2/lyh/vla/runs
LOG=~/offline_eval/results/overnight.log
export GATE_PYTHON=$P/.conda/qwenvla/bin/python OMP_NUM_THREADS=8 MKL_NUM_THREADS=8
declare -A RESTARTS
log() { echo "$(date '+%m-%d %T') $*" >> "$LOG"; }
final_step() { local m; m=$(grep "^$1|" ~/offline_eval/runs.conf | sed -n 's/.*MAX_STEPS=\([0-9]*\).*/\1/p'); echo "${m:-4000}"; }
newest_save() { ls -d $R/franka_task0_actv2_continue_$1_20261030/*/save_*_chkpt 2>/dev/null \
  | while read -r d; do [[ -f $d/checkpoint_complete.json ]] && echo "$(basename "$d" | tr -dc 0-9) $d"; done | sort -n | tail -1 | cut -d' ' -f2; }
log "watchdog start: $(cut -d'|' -f1 ~/offline_eval/runs.conf | tr '\n' ' ')"
while true; do
  while IFS='|' read -r tag envs after; do
    [[ -n "$tag" && "$tag" != \#* ]] || continue
    final=$(final_step $tag)
    waiting=0
    for dep in $after; do
      [[ "$(newest_save $dep)" == *save_$(final_step $dep)_chkpt ]] && ! tmux has-session -t claude_actv2_$dep 2>/dev/null || waiting=1
    done
    (( waiting )) && continue
    newest=$(newest_save $tag)
    if [[ "$newest" != *save_${final}_chkpt ]] && ! tmux has-session -t claude_actv2_$tag 2>/dev/null; then
      n=${RESTARTS[$tag]:-0}
      if (( n >= 3 )); then [[ $n == 3 ]] && log "$tag: down, restart budget exhausted"; RESTARTS[$tag]=4
      else RESTARTS[$tag]=$((n + 1)); log "$tag: training session gone; restart from ${newest:-init}"
        tmux new-session -d -s claude_actv2_$tag "cd $P && $envs RESUME_PATH=$newest bash train-scripts/run_franka_task0_actv2_continue_20.sh"; fi
    fi
  done < ~/offline_eval/runs.conf
  for ckpt in $(ls -d $R/franka_task0_actv2_continue_*_20261030/*/save_*_chkpt 2>/dev/null); do
    [[ -f $ckpt/checkpoint_complete.json ]] || continue
    tag=$(sed 's#.*/franka_task0_actv2_continue_\(.*\)_20261030/.*#\1#' <<< "$ckpt")
    step=$(basename "$ckpt" | tr -dc 0-9); out=~/offline_eval/results/actv2_${tag}_${step}
    [[ -f $out/summary.json ]] && continue
    tries=$(cat $out.tries 2>/dev/null || echo 0); (( tries >= 2 )) && continue; echo $((tries + 1)) > $out.tries
    # Gate on a GPU of the run's own set when it is training, else GPU 0; servers share fine.
    log "$tag $step: gate start (try $((tries + 1)))"
    bash ~/offline_eval/gate.sh "$ckpt" 0 8170 "$out" > "$out.log" 2>&1
    log "RESULT $tag $step $(grep -h '^GATE' "$out.log")"
  done
  sleep 120
done
