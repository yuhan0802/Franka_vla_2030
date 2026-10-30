#!/usr/bin/env bash
# Overnight watchdog on mac_20 (tmux claude_overnight): every 2 min
#  1. relaunch a crashed act-v2 continuation run from its newest checkpoint (max 3 restarts per run);
#  2. gate every new checkpoint with the transformers 5.3 env (run's own GPU set, one at a time).
# Log: ~/offline_eval/results/overnight.log
set -u
P=/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA
R=/data2/lyh/vla/runs
LOG=~/offline_eval/results/overnight.log
export GATE_PYTHON=$P/.conda/qwenvla/bin/python
declare -A DATA=([all50]=/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0 [holdout5]=/data2/lyh/vla/claude_data/holdout5/Task0)
declare -A GPUS=([all50]=0,1,2,3 [holdout5]=4,5,6,7)
declare -A PORT=([all50]=29711 [holdout5]=29712)
declare -A GATE_GPU=([all50]=1 [holdout5]=5)
declare -A GATE_PORT=([all50]=8141 [holdout5]=8142)
declare -A RESTARTS=([all50]=0 [holdout5]=0)
log() { echo "$(date '+%m-%d %T') $*" >> "$LOG"; }
newest_save() { ls -d $R/franka_task0_actv2_continue_$1_20261030/*/save_*_chkpt 2>/dev/null \
  | while read -r d; do [[ -f $d/checkpoint_complete.json ]] && echo "$(basename "$d" | tr -dc 0-9) $d"; done | sort -n | tail -1 | cut -d' ' -f2; }
log "watchdog start"
while true; do
  for tag in all50 holdout5; do
    newest=$(newest_save $tag)
    if [[ "$newest" != *save_4000_chkpt ]] && ! tmux has-session -t claude_actv2_$tag 2>/dev/null; then
      if (( RESTARTS[$tag] >= 3 )); then log "$tag: down, restart budget exhausted"; continue; fi
      RESTARTS[$tag]=$((RESTARTS[$tag] + 1))
      log "$tag: training session gone; restart ${RESTARTS[$tag]} from ${newest:-init}"
      tmux new-session -d -s claude_actv2_$tag "cd $P && TAG=$tag DATA_ROOT=${DATA[$tag]} GPUS=${GPUS[$tag]} MASTER_PORT=${PORT[$tag]} RESUME_PATH=$newest bash train-scripts/run_franka_task0_actv2_continue_20.sh"
    fi
    for ckpt in $(ls -d $R/franka_task0_actv2_continue_${tag}_20261030/*/save_*_chkpt 2>/dev/null); do
      [[ -f $ckpt/checkpoint_complete.json ]] || continue
      step=$(basename "$ckpt" | tr -dc 0-9); out=~/offline_eval/results/actv2_${tag}_${step}
      [[ -f $out/summary.json || -f $out.done ]] && continue
      log "$tag $step: gate start"
      bash ~/offline_eval/gate.sh "$ckpt" ${GATE_GPU[$tag]} ${GATE_PORT[$tag]} "$out" > "$out.log" 2>&1
      touch "$out.done"
      log "RESULT $tag $step $(grep -h '^GATE' "$out.log")"
    done
  done
  sleep 120
done
