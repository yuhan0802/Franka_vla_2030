#!/usr/bin/env bash
# Task3 retrain on corrected gripper labels (claude, 2026-11-03). Run in tmux claude_task3_gripfix.
#   P1   : from ownstate (8-task joint finetune, save_20000), 8k steps, lr 1e-4, no state aug, save every 1k
#   N8   : from P1 save_8000, lr 5e-5, state noise, 2k steps, save every 500
#   eval : teacher-forced (3D approach error + close timing) on every save, 8 GPUs
# Data: /data2/lyh/claude_data/Franka_vr_task3_v5gripfix_20261103 (see docs/task3_gripper_label_bug_20261103.md)
set -u
P=/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA
LAUNCH=$P/train-scripts/run_franka_task0_actv2_continue_20.sh
D=/data2/lyh/claude_data/Franka_vr_task3_v5gripfix_20261103
R=/data2/lyh/vla/runs/claude_task3_gripfix_20261103
GPUS=0,1,2,3,4,5,6,7
COMMON="ACTIONCODEC_PATH=/data2/lyh/vla/runs/actioncodec_franka_all_real_finetune/20261028_063653/step_020000 ACTION_NORMALIZATION=franka_quantile ACTION_NORM_STATS_PATH=/data2/lyh/vla/runs/actioncodec_franka_all_real_finetune/20261028_063653/step_020000/action_norm_stats.json STATE_STATS_PATH=/data2/lyh/vla/config/franka_all_real_eef_state_q01q99_20261028.json"
OWNSTATE=/data2/lyh/vla/runs/franka_joint8_newcodec_ownstate_qwen_20k_20261028/Qwen3VL2B+Franka-Joint+lr0.0001+joint8_v5_codecft20k_ownstate_qwen_first5_20k_b256/save_20000_chkpt
mkdir -p "$R"
log() { echo "$(date '+%m-%d %T') $*" >> "$R/chain.log"; }
newest() { ls -d "$1"/*/save_*_chkpt 2>/dev/null | while read -r d; do [[ -f $d/checkpoint_complete.json ]] && echo "$(basename "$d" | tr -dc 0-9) $d"; done | sort -n | tail -1 | cut -d' ' -f2; }
save_of() { local d; d=$(ls -d "$1"/*/save_$2_chkpt 2>/dev/null | head -1); [[ -n $d && -f $d/checkpoint_complete.json ]] && echo "$d"; }

# run_phase <label> <root> <final_step> <port> <env...>; resumes from the newest save, 3 tries
run_phase() {
  local label=$1 root=$2 final=$3 port=$4; shift 4
  local try=0 last
  while [[ -z "$(save_of "$root" "$final")" ]]; do
    (( try >= 3 )) && { log "$label: FAILED after $try tries"; return 1; }
    try=$((try + 1)); last=$(newest "$root")
    log "$label: start try $try on GPUs $GPUS from ${last:-init}"
    env $COMMON "$@" TAG="$label" RUN_ROOT="$root" GPUS="$GPUS" NUM_PROCESSES=8 GRAD_ACCUM=1 MASTER_PORT="$port" \
      DATA_ROOT=$D/Task3 TASKS_PATH=$D/tasks.json RESUME_PATH="$last" bash "$LAUNCH" > "$root.try$try.out" 2>&1
  done
  log "$label: done ($(save_of "$root" "$final"))"
}

log "start; data $D; init $OWNSTATE"
run_phase task3gripfixp1 "$R/p1_8k" 8000 29811 \
  INIT_CHECKPOINT="$OWNSTATE" STATE_AUG=none LEARNING_RATE=1e-4 MAX_STEPS=8000 SAVE_STEPS=1000 || exit 1
run_phase task3gripfixn8000 "$R/noise_from8000" 2000 29812 \
  INIT_CHECKPOINT="$(save_of "$R/p1_8k" 8000)" STATE_AUG=noise LEARNING_RATE=5e-5 MAX_STEPS=2000 SAVE_STEPS=500 || exit 1
log "TRAIN_DONE"

SPECS=(); for s in 1000 2000 3000 4000 5000 6000 7000 8000; do SPECS+=(p1_8k:$s); done
for s in 500 1000 1500 2000; do SPECS+=(noise_from8000:$s); done
log "teacher-forced eval of ${#SPECS[@]} checkpoints"
RUN_DIR=$R EVAL_DATA_ROOT=$D bash ~/claude_scripts/tf_run.sh gripfix "0 1 2 3 4 5 6 7" "${SPECS[@]}"
log "ALL_DONE (results ~/offline_eval/results/task3_tf_gripfix/tf.log)"
