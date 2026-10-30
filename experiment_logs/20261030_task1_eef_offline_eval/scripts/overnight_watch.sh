#!/usr/bin/env bash
# Overnight watchdog on mac_20 (tmux claude_overnight): every 2 min
#  1. relaunch a crashed act-v2 continuation run from its newest checkpoint (max 3 restarts per run);
#  2. start all50noise on all 8 GPUs once all50 and holdout5 both reached save_4000;
#  3. gate every new checkpoint with the transformers 5.3 env, one at a time.
# Log: ~/offline_eval/results/overnight.log
set -u
P=/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA
R=/data2/lyh/vla/runs
LOG=~/offline_eval/results/overnight.log
export GATE_PYTHON=$P/.conda/qwenvla/bin/python
ALL50=/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0
TAGS=(all50 holdout5 all50noise)
declare -A ENVS=(
  [all50]="TAG=all50 DATA_ROOT=$ALL50 GPUS=0,1,2,3 MASTER_PORT=29711"
  [holdout5]="TAG=holdout5 DATA_ROOT=/data2/lyh/vla/claude_data/holdout5/Task0 GPUS=4,5,6,7 MASTER_PORT=29712"
  [all50noise]="TAG=all50noise STATE_AUG=noise DATA_ROOT=$ALL50 GPUS=0,1,2,3,4,5,6,7 NUM_PROCESSES=8 GRAD_ACCUM=1 MASTER_PORT=29713")
declare -A GATE_GPU=([all50]=1 [holdout5]=5 [all50noise]=1)
declare -A GATE_PORT=([all50]=8141 [holdout5]=8142 [all50noise]=8143)
declare -A RESTARTS=([all50]=0 [holdout5]=0 [all50noise]=0)
log() { echo "$(date '+%m-%d %T') $*" >> "$LOG"; }
newest_save() { ls -d $R/franka_task0_actv2_continue_$1_20261030/*/save_*_chkpt 2>/dev/null \
  | while read -r d; do [[ -f $d/checkpoint_complete.json ]] && echo "$(basename "$d" | tr -dc 0-9) $d"; done | sort -n | tail -1 | cut -d' ' -f2; }
finished() { [[ "$(newest_save $1)" == *save_4000_chkpt ]] && ! tmux has-session -t claude_actv2_$1 2>/dev/null; }
launch() { tmux new-session -d -s claude_actv2_$1 "cd $P && ${ENVS[$1]} RESUME_PATH=$2 bash train-scripts/run_franka_task0_actv2_continue_20.sh"; }
log "watchdog start (with all50noise queued)"
while true; do
  for tag in "${TAGS[@]}"; do
    newest=$(newest_save $tag)
    if [[ $tag == all50noise && -z "$newest" ]] && ! tmux has-session -t claude_actv2_$tag 2>/dev/null; then
      # Queued run: needs every GPU, so start only after both 4-GPU runs are done.
      if finished all50 && finished holdout5 && (( RESTARTS[$tag] == 0 )); then
        RESTARTS[$tag]=1; log "$tag: both 4-GPU runs finished; starting on 8 GPUs"; launch $tag ""
      elif (( RESTARTS[$tag] > 0 && RESTARTS[$tag] < 4 )); then
        RESTARTS[$tag]=$((RESTARTS[$tag] + 1)); log "$tag: died before first save; restart from init"; launch $tag ""
      fi
    elif [[ -n "$newest" || $tag != all50noise ]] && [[ "$newest" != *save_4000_chkpt ]] \
        && ! tmux has-session -t claude_actv2_$tag 2>/dev/null; then
      if (( RESTARTS[$tag] >= 4 )); then log "$tag: down, restart budget exhausted"
      else RESTARTS[$tag]=$((RESTARTS[$tag] + 1)); log "$tag: training session gone; restart from ${newest:-init}"; launch $tag "$newest"; fi
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
