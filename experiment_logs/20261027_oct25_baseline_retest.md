# 2026-10-27 Oct-25 baseline retest

Command configuration:

- task/model: `task1 act-v2`
- temporary config: `/tmp/eef_oct25_baseline.57JNhxmz.json`
- execute horizon: 15
- limits: 60 chunks / 900 actions
- action gain: `[1, 1, 1, 1, 1, 1]`
- maximum TF age: 0.15 s
- camera: left/calibrated plus wrist
- checkpoint: `save_8000_chkpt` on `mac_63:8095`

## Attempt 1

- log root: `/media/mac/Elements1/franka_vr_docker/logs/eef_task1_act-v2_20261027_205206_311742`
- dry-run: passed
- execute: safety fault after chunk 4 / 60 actions
- fault: `world -> fr3_link8 is 0.186s old`, exceeding the 0.15 s limit
- gripper error: none
- reset logs:
  - `/media/mac/Elements1/franka_vr_docker/logs/fci_init_20261027_205317_2yUwlz/homing.jsonl`
  - `/media/mac/Elements1/franka_vr_docker/logs/fci_arm_reset_20261027_205327_Yj2O5R/arm.jsonl`

## Attempt 2

- log root: `/media/mac/Elements1/franka_vr_docker/logs/eef_task1_act-v2_20261027_205338_314917`
- dry-run: passed
- execute: gripper close requested after chunk 11; close completed at width 0.000163 m
- execute: safety fault during chunk 15 / 225 actions
- fault: target led measured pose by 0.0601 m / 0.1488 rad
- gripper error: none
- reset logs:
  - `/media/mac/Elements1/franka_vr_docker/logs/fci_init_20261027_205517_vQdwQW/homing.jsonl`
  - `/media/mac/Elements1/franka_vr_docker/logs/fci_arm_reset_20261027_205525_fYbVrJ/arm.jsonl`
- final reset: successful; gripper homed open and arm returned to saved pose

## Current state

- direct controller stopped
- robot reset completed
- inference server on `mac_63:8095` remains available
- a third attempt requires restoring the physical task scene first

## Attempt 3

- log root: `/media/mac/Elements1/franka_vr_docker/logs/eef_task1_act-v2_20261027_205650_318951`
- dry-run: passed
- execute: gripper close requested after chunk 13; close completed at width 0.001039 m
- execute: safety fault during chunk 18 / 270 actions
- fault: measured EEF pose jumped 0.0054 m / 0.0155 rad in 0.004 s; TF source classified unstable
- gripper error: none
- reset logs:
  - `/media/mac/Elements1/franka_vr_docker/logs/fci_init_20261027_205831_MdQ5HU/homing.jsonl`
  - `/media/mac/Elements1/franka_vr_docker/logs/fci_arm_reset_20261027_205840_P0YhVF/arm.jsonl`
- final reset: successful; gripper homed open and arm returned to saved pose

## Current state after attempt 3

- direct controller stopped
- robot reset completed
- inference server on `mac_63:8095` remains available
- the three attempts show three separate safety stops: stale TF, tracking lead, and a one-sample measured-pose jump

## Attempt 4: local-NVMe log-root A/B test

- command override: `FRANKA_LOG_ROOT=/home/mac/franka_vr_docker/logs`
- axis-test log: `/home/mac/franka_vr_docker/logs/eef_task1_act-v2_axis_+z_20261027_210601_325179`
- execute log root: `/home/mac/franka_vr_docker/logs/eef_task1_act-v2_20261027_210710_327179`
- dry-run: passed
- execute: software completed normally after 34 chunks / 510 actions, but the physical task failed
- gripper: close/open cycles completed without errors; final command left it closed at width 0.001552 m
- physical result: missed the test tube during the first grasp; the tube remained in the rack when the policy reached the basket
- old successful grasp: close after action 118 at measured XYZ `[0.69482, -0.20336, 0.27056]`
- this failed grasp: close after action 191 at measured XYZ `[0.69258, -0.20831, 0.23845]` (about 32 mm lower)
- scene mismatch visible in observation 0: the current tube protrudes substantially farther from the rack, and the rack/basket image positions differ from the Oct-25 successful run
- 100 Hz executor period: median 10.0 ms, p99 10.2 ms, maximum 11.3 ms; no samples above 15 ms
- maximum joint tracking error: 0.006891 rad
- policy latency: median 0.835 s, maximum 1.35 s
- software faults: none; physical task success: false
- direct stack remains parked at zero velocity; no automatic reset was performed
