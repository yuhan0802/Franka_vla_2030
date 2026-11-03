#!/usr/bin/env bash
set -euo pipefail
P='/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA'
PY="$P/.conda/qwenvla/bin/python"; [[ -x "$PY" ]] || PY=/home/xmu/anaconda3/envs/qwenvla/bin/python
cd ~/franka_eef_server
export CUDA_VISIBLE_DEVICES='3' PYTHONPATH="$P/third_party/python:$P" VLA_PROJECT_ROOT="$P"
export HF_HOME="$P/actioncodec_libero/.cache/huggingface" USE_TF=0 TRANSFORMERS_NO_TF=1 TF_CPP_MIN_LOG_LEVEL=3 TOKENIZERS_PARALLELISM=false
"$PY" -u deploy_franka_task0_qwen_actioncodec_eef.py --checkpoint '/data2/lyh/vla/runs/franka_task01_newcodec_from_joint8_11k_10k_20261029/Qwen3VL2B+Franka-Joint+lr0.0001+task01_newcodec_ownstate_from_joint8step11k_10k_b256/save_10000_chkpt' --actioncodec '/data2/lyh/vla/runs/actioncodec_franka_all_real_finetune/20261028_063653/step_020000' --state-stats '/data2/lyh/vla/config/franka_all_real_eef_state_q01q99_20261028.json' --port '8111' --rotation-execution 'relative_rotvec_left_multiply'  2>&1 | tee -a claude_eval_8111.log
