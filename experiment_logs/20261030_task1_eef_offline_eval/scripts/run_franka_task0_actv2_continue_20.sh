#!/usr/bin/env bash
# Continue act-v2 (v5 maxabs save_8000, the checkpoint that succeeded on the real robot 2026-10-25/30)
# on the clean 50-episode Task0 set. act-v2 saw only episodes 001-029 and misses unseen tube
# positions offline (035/044: 14 cm); more positions is the targeted fix.
# Same code path, protocol, normalization stats and transformers 5.3 env as act-v2.
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA}"
PYTHON="${PYTHON:-$PROJECT_ROOT/.conda/qwenvla/bin/python}"
DATA_ROOT="${DATA_ROOT:-/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0}"
TASKS_PATH="${TASKS_PATH:-/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/tasks.json}"
INIT_CHECKPOINT="${INIT_CHECKPOINT:-/data2/lyh/vla/runs/franka_task0_droid_bar_v5_maxabs/Qwen3VL2B+Franka-Task0+lr0.0001+v5_maxabs_first15_all29_no_crop_10k_8gpu_b256/save_8000_chkpt}"
ACTIONCODEC_PATH="${ACTIONCODEC_PATH:-/data2/lyh/weights/ActionCodec-Base-RVQft}"
ACTION_NORM_STATS_PATH="${ACTION_NORM_STATS_PATH:-/data2/lyh/vla/config/task0_v5_action_maxabs.json}"
VLA_PATH="${VLA_PATH:-/mnt/82_store/lyh/Qwen_weights/Qwen3-VL-2B}"
STATE_STATS_PATH="${STATE_STATS_PATH:-/data2/lyh/vla/config/droid_eef_axis_angle_state_stats.json}"
STATE_AUG="${STATE_AUG:-none}"
TAG="${TAG:?set TAG, e.g. all50}"
RUN_ROOT="${RUN_ROOT:-/data2/lyh/vla/runs/franka_task0_actv2_continue_${TAG}_20261030}"
RUN_ID_NOTE="${RUN_ID_NOTE:-actv2_8k_continue_${TAG}_stateaug_${STATE_AUG}}"
GPUS="${GPUS:?set GPUS, e.g. 0,1,2,3}"
NUM_PROCESSES="${NUM_PROCESSES:-4}"
GRAD_ACCUM="${GRAD_ACCUM:-2}"
MASTER_PORT="${MASTER_PORT:?set MASTER_PORT}"
MAX_STEPS="${MAX_STEPS:-4000}"
LEARNING_RATE="${LEARNING_RATE:-5e-5}"
# JOINT_TASK_NAMES=Task0,Task1 co-trains sibling task dirs of DATA_ROOT (stats must list the tasks).
JOINT_TASK_NAMES="${JOINT_TASK_NAMES:-}"
DATA_ARGS=(--data_root_dir "$DATA_ROOT")
[[ -z "$JOINT_TASK_NAMES" ]] || DATA_ARGS=(--data_root_dir "$(dirname "$DATA_ROOT")" --joint_task_names "$JOINT_TASK_NAMES")

export FRANKA_CACHE_VERSION=v5
export FRANKA_STATE_AUG="$STATE_AUG"
export FRANKA_WINDOW_FILTER_SEMANTICS=state_first5_activity_no_gripper_hold_v5
export PYTHONPATH="$PROJECT_ROOT/third_party/python:$PROJECT_ROOT:${PYTHONPATH:-}"
export HF_HOME="${HF_HOME:-/data2/lyh/vla/cache/huggingface}"
export PYTHONNOUSERSITE=1 USE_TF=0 TRANSFORMERS_NO_TF=1 TF_CPP_MIN_LOG_LEVEL=3
export WANDB_MODE="${WANDB_MODE:-disabled}"

for path in "$PYTHON" "$TASKS_PATH" "$INIT_CHECKPOINT/model.safetensors" "$ACTIONCODEC_PATH/model.safetensors" \
  "$ACTION_NORM_STATS_PATH" "$VLA_PATH" "$STATE_STATS_PATH" "$DATA_ROOT"; do
  [[ -e "$path" ]] || { echo "Missing required path: $path" >&2; exit 1; }
done
"$PYTHON" - "$INIT_CHECKPOINT" "$ACTION_NORM_STATS_PATH" <<'PY'
import json, sys
init, stats_path = sys.argv[1:]
trained = json.load(open(f"{init}/training_config.json")).get("action_normalization_stats")
stats = json.load(open(stats_path))
# A DROID-pretrained init (act-v2 recipe from scratch) has no Franka stats to compare against.
if trained is None:
    print("init has no action stats (DROID-pretrained); training fresh with", stats_path)
else:
    assert stats["arm_scale"] == trained["arm_scale"], "maxabs stats differ from the act-v2 checkpoint"
    print("maxabs stats match the initialization checkpoint")
PY
echo "Episodes: $(ls -d "$DATA_ROOT"/*/data/franka_measured_eef_base_15hz_v5.npz | wc -l) in $DATA_ROOT"

# RESUME_PATH (a save_N_chkpt of this run) continues after a crash; the overnight watchdog sets it.
START_ARGS=(--init_checkpoint_path "$INIT_CHECKPOINT")
[[ -z "${RESUME_PATH:-}" ]] || START_ARGS=(--resume true --resume_path "$RESUME_PATH")
mkdir -p "$RUN_ROOT/logs" "$HF_HOME"
# FRANKA_TARGETS_JSON (tube/basket xy in the instruction) is copied next to the run for gate.sh.
[[ -z "${FRANKA_TARGETS_JSON:-}" ]] || { cp "$FRANKA_TARGETS_JSON" "$RUN_ROOT/targets.json"; export FRANKA_TARGETS_JSON; echo "Targets: $FRANKA_TARGETS_JSON"; }
LOG="$RUN_ROOT/logs/task0_actv2_continue_${TAG}_$(date +%Y%m%d_%H%M%S).log"
echo "Init: $INIT_CHECKPOINT; state_aug=$STATE_AUG; $NUM_PROCESSES GPUs x 16 x accum $GRAD_ACCUM; lr $LEARNING_RATE; steps $MAX_STEPS"
echo "Log: $LOG"
[[ "${PREFLIGHT_ONLY:-0}" != 1 ]] || exit 0

exec env CUDA_VISIBLE_DEVICES="$GPUS" "$PYTHON" -m torch.distributed.run \
  --nproc_per_node="$NUM_PROCESSES" --master_port="$MASTER_PORT" \
  "$PROJECT_ROOT/train/franka_task0_qwen_actioncodec_droid_pretrained_bar_stateaug.py" \
  --base_model qwen3vl2b --vla_path "$VLA_PATH" \
  --actioncodec_path "$ACTIONCODEC_PATH" \
  "${DATA_ARGS[@]}" --data_mix franka_task0 \
  --tasks_path "$TASKS_PATH" --state_stats_path "$STATE_STATS_PATH" \
  --action_embodiment_id 2 --action_prediction_mode block_ar_16 \
  --action_normalization task0_maxabs --action_norm_stats_path "$ACTION_NORM_STATS_PATH" \
  --predict_future_state true --prediction_order state_then_action \
  "${START_ARGS[@]}" \
  --image_size 256 --use_image_aug true \
  --use_random_resized_crop false --primary_only_random_resized_crop true \
  --use_robust_image_aug true \
  --batch_size 16 --grad_accumulation_steps "$GRAD_ACCUM" --dataloader_num_workers 0 \
  --learning_rate "$LEARNING_RATE" --min_learning_rate 5e-6 --max_steps "$MAX_STEPS" \
  --save_steps 1000 --val_steps 1000 --val_batches 50 \
  --gripper_transition_weight 3 --validation_episodes 2 --train_all_episodes true \
  --trajectory_sampling true \
  --run_root_dir "$RUN_ROOT" --run_id_note "$RUN_ID_NOTE" \
  2>&1 | tee "$LOG"
