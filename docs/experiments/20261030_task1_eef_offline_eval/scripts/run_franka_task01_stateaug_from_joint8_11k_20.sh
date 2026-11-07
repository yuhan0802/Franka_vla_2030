#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA}"
PYTHON="${PYTHON:-/home/xmu/anaconda3/envs/qwenvla/bin/python}"
DATA_ROOT="${DATA_ROOT:-/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028}"
TASKS_PATH="${TASKS_PATH:-$DATA_ROOT/tasks.json}"
INIT_CHECKPOINT="${INIT_CHECKPOINT:-/data2/lyh/vla/runs/franka_joint8_newcodec_ownstate_qwen_20k_20261028/Qwen3VL2B+Franka-Joint+lr0.0001+joint8_v5_codecft20k_ownstate_qwen_first5_20k_b256/save_11000_chkpt}"
ACTIONCODEC_PATH="${ACTIONCODEC_PATH:-/data2/lyh/vla/runs/actioncodec_franka_all_real_finetune/20261028_063653/step_020000}"
VLA_PATH="${VLA_PATH:-/mnt/82_store/lyh/Qwen_weights/Qwen3-VL-2B}"
STATE_STATS_PATH="${STATE_STATS_PATH:-/data2/lyh/vla/config/franka_all_real_eef_state_q01q99_20261028.json}"
STATE_AUG="${STATE_AUG:?set STATE_AUG=none|mask|drop|noise}"
RUN_ROOT="${RUN_ROOT:-/data2/lyh/vla/runs/franka_task01_stateaug_${STATE_AUG}_from_joint8_11k_20261030}"
RUN_ID_NOTE="${RUN_ID_NOTE:-task01_stateaug_${STATE_AUG}_from_joint8step11k_4k_b32}"
GPUS="${GPUS:?set GPUS, e.g. 0,1}"
NUM_PROCESSES="${NUM_PROCESSES:-2}"
MASTER_PORT="${MASTER_PORT:?set MASTER_PORT}"
MAX_STEPS="${MAX_STEPS:-4000}"
JOINT_TASK_NAMES="Task0,Task1"
ACTION_NORM_STATS_PATH="$ACTIONCODEC_PATH/action_norm_stats.json"

export FRANKA_CACHE_VERSION=v5
export FRANKA_STATE_AUG="$STATE_AUG"
export FRANKA_WINDOW_FILTER_SEMANTICS=state_first5_activity_no_gripper_hold_v5
export PYTHONPATH="$PROJECT_ROOT/third_party/python:$PROJECT_ROOT:${PYTHONPATH:-}"
export HF_HOME="${HF_HOME:-/data2/lyh/vla/cache/huggingface}"
export PYTHONNOUSERSITE=1 USE_TF=0 TRANSFORMERS_NO_TF=1 TF_CPP_MIN_LOG_LEVEL=3
export WANDB_MODE="${WANDB_MODE:-disabled}"

for path in "$PYTHON" "$TASKS_PATH" "$INIT_CHECKPOINT/checkpoint_complete.json" \
  "$ACTIONCODEC_PATH/checkpoint_complete.json" "$ACTION_NORM_STATS_PATH" \
  "$VLA_PATH" "$STATE_STATS_PATH" "$DATA_ROOT/Task0" "$DATA_ROOT/Task1"; do
  [[ -e "$path" ]] || { echo "Missing required path: $path" >&2; exit 1; }
done

"$PYTHON" - "$DATA_ROOT" "$ACTIONCODEC_PATH" "$ACTION_NORM_STATS_PATH" <<'PY'
import glob,json,sys,numpy as np
root,codec,stats_path=sys.argv[1:]
stats=json.load(open(stats_path)); cfg=json.load(open(codec+'/config.json'))
assert stats['method']=='franka_real_q01q99_symmetric_v1'
assert {'Task0','Task1'}.issubset(stats['tasks'])
assert cfg['n_tokens']==48 and cfg['n_quantizers']==3 and cfg['vq_codebook_size']==2048
for task,expected in [('Task0',50),('Task1',30)]:
    paths=glob.glob(f'{root}/{task}/*/data/franka_measured_eef_base_15hz_v5.npz')
    assert len(paths)==expected,(task,len(paths),expected)
    for path in paths:
        with np.load(path,allow_pickle=False) as cache:
            assert str(cache['valid_start_semantics'])=='state_first5_activity_no_gripper_hold_v5',path
            assert float(cache['gripper_hold_seconds'])==0.0,path
print('Task0/Task1 cache, codec, and quantile protocol verified')
PY

mkdir -p "$RUN_ROOT/logs" "$HF_HOME"
LOG="$RUN_ROOT/logs/task01_stateaug_${STATE_AUG}_$(date +%Y%m%d_%H%M%S).log"
echo "Initialization checkpoint: $INIT_CHECKPOINT (fresh optimizer)"
echo "Tasks: Task0,Task1; data=$DATA_ROOT"
echo "ActionCodec: $ACTIONCODEC_PATH; quantile stats=$ACTION_NORM_STATS_PATH"
echo "Protocol: measured world-frame EEF delta, left-multiply rotation, 15 Hz, gripper open=1"
echo "Training: $NUM_PROCESSES GPUs x 16 x grad-accum 1; state_aug=$STATE_AUG; new steps=$MAX_STEPS"
echo "LR: 1e-4 cosine to 5e-6; save/validate every 1000; no token reset"
echo "Log: $LOG"
[[ "${PREFLIGHT_ONLY:-0}" != 1 ]] || exit 0

exec env CUDA_VISIBLE_DEVICES="$GPUS" "$PYTHON" -m torch.distributed.run \
  --nproc_per_node="$NUM_PROCESSES" --master_port="$MASTER_PORT" \
  "$PROJECT_ROOT/train/franka_task0_qwen_actioncodec_droid_pretrained_bar_stateaug.py" \
  --base_model qwen3vl2b --vla_path "$VLA_PATH" \
  --actioncodec_path "$ACTIONCODEC_PATH" \
  --data_root_dir "$DATA_ROOT" --data_mix franka_task0 \
  --joint_task_names "$JOINT_TASK_NAMES" --tasks_path "$TASKS_PATH" \
  --state_stats_path "$STATE_STATS_PATH" --action_embodiment_id 2 \
  --action_prediction_mode block_ar_16 --action_normalization franka_quantile \
  --action_norm_stats_path "$ACTION_NORM_STATS_PATH" \
  --predict_future_state true --prediction_order state_then_action \
  --init_checkpoint_path "$INIT_CHECKPOINT" --resume false \
  --reset_action_token_embeddings false --reset_new_token_embeddings false \
  --image_size 256 --use_image_aug true --use_random_resized_crop false \
  --primary_only_random_resized_crop true --use_robust_image_aug true \
  --batch_size 16 --grad_accumulation_steps 1 --dataloader_num_workers 0 \
  --learning_rate 1e-4 --min_learning_rate 5e-6 --max_steps "$MAX_STEPS" \
  --save_steps 1000 --val_steps 1000 --val_batches 50 \
  --gripper_transition_weight 2 --validation_episodes 2 --train_all_episodes true \
  --trajectory_sampling true --run_root_dir "$RUN_ROOT" --run_id_note "$RUN_ID_NOTE" \
  2>&1 | tee "$LOG"
