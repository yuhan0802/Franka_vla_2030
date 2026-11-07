# pi0.5 tube_pick_v5 training/execution audit

Evidence: `logs/pi05_training_audit/roundtrip.json` and
`scripts/audit_pi05_training_roundtrip.py` (read-only audit on server 66).

## Recorded data to training input

- The recorder stores actual sent 7-joint velocities in rad/s, measured finger
  width in metres, and commanded gripper target width in metres.
- `prepare_franka_pi05_droid.py` subsamples to 15 Hz. Its conversion manifest
  specifies a velocity divisor of 3.0 and maximum gripper width of 0.08 m.
- Dataset state: joint positions in joint1..7 order + `1 - width / 0.08`.
- Dataset action: `clip(sent_velocity / 3, -1, 1)` +
  `clip(1 - target_width / 0.08, 0, 1)`.
- Training randomly selects either exterior camera and always uses the front
  (wrist) camera. The third image slot is zero and masked false.
- PI05 normalizes state and actions using q01/q99 from norm_stats.json. Images
  are resized/padded once by the server's training-equivalent JAX transform.

## Model output to robot target

1. Server Unnormalize: `(z + 1) / 2 * (q99 - q01 + 1e-6) + q01`.
2. DroidOutputs selects the first 8 dimensions of the 32-dimensional output.
3. Client `--action-units droid` (default) restores `velocity = action[:7] * 3`.
4. Gripper target: `width = (1 - clip(action[7], 0, 1)) * 0.08`.

State statistics must not be applied to actions. Server denormalization and
dataset unit conversion are separate inverses, each applied once.
`--action-units direct-rad-s` retains the previous no-multiplier experiment;
it is not the inverse of the recorded dataset conversion.

## Actual data verification

All 29 converted episodes / 8357 frames were checked against original source
recordings. Maximum recovered velocity error: 7.45058e-9 rad/s; no source
velocity values were clipped during dataset conversion. Maximum gripper
target reconstruction error: 1.78814e-9 m. Quantile roundtrip error:
1.11022e-16. Checkpoint 5k/10k/15k/20k norm_stats files have identical SHA256.

## Remaining execution differences

This establishes matching data semantics, not identical physical trajectories.
Safety envelopes, MoveIt Servo smoothing, servo/controller tracking and
feedback stopping still apply. Each action log now records policy values,
decoded physical velocity and guarded velocity separately.
The user-requested serial policy runs all 16 steps before acquiring new
observations. It stops between chunks; this differs from continuous human
teleoperation and is not an execution schedule specified by training alone.
Gripper targets retain width semantics; execution still waits for Move/Grasp
results and uses force limits plus a 2 mm target-change deadband.
Live RGB images differ from training's encoded/decoded video frames; training
augmentation and random exterior-camera selection are not inference actions.

## 2026-10-15 aligned 15k trial

The corrected RGB/state pipeline passed live dry-run. Serial execution used
0.15 rad/s and 0.5 rad/s² safety envelopes, compared with 0.06/0.20 previously.
It sent 100 actions, including a model-requested grasp with closedness 0.872468
(target width approximately 0.0102 m). Grasp returned success=false; execution
latched a fault, commanded zero, and stopped without retry. Post-test feedback
showed total finger width approximately 0.000628 m, with the tube still in its
rack. This is consistent with an empty grasp, not proof of a model-only cause.
There were 13 velocity-clipped and 14 acceleration-clipped commands, so actual
execution still differs from the decoded policy. FCI success rate was 1.0 and
Servo status was No warnings. The robot stack was shut down normally.
See `logs/pi05_training_audit/execute.log`, `fci_after.log`, and `after.jpg`.
