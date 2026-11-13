#!/usr/bin/env bash
# Evaluate each saved step of the four state-aug runs with gate.sh as soon as all four are complete.
set -u
R=/data2/lyh/vla/runs
augs=(mask drop noise none); gpus=(0 2 4 6)
for step in 1000 2000 3000 4000; do
  for aug in "${augs[@]}"; do
    until ls $R/franka_task01_stateaug_${aug}_from_joint8_11k_20261030/*/save_${step}_chkpt/checkpoint_complete.json >/dev/null 2>&1; do sleep 60; done
  done
  echo "$(date +%T) step $step saved for all runs"
  pids=()
  for i in 0 1 2 3; do
    aug=${augs[$i]}
    ckpt=$(dirname $(ls $R/franka_task01_stateaug_${aug}_from_joint8_11k_20261030/*/save_${step}_chkpt/checkpoint_complete.json))
    [[ $aug == mask ]] && echo '{"state_input": "gripper_only"}' > $ckpt/state_input.json
    out=~/offline_eval/results/stateaug_${aug}_${step}
    bash ~/offline_eval/gate.sh $ckpt ${gpus[$i]} $((8121+i)) $out > $out.log 2>&1 &
    pids+=($!)
  done
  wait "${pids[@]}"
  for aug in "${augs[@]}"; do echo "RESULT $aug $step $(grep -h '^GATE\|state_input' ~/offline_eval/results/stateaug_${aug}_${step}.log | tr '\n' ' ')"; done
done
echo ALL_DONE
