"""Read-only audit, run with the training environment on server 66."""
import hashlib
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

root = Path('/mnt/82_store/vla/Franka_vr/tube_pick_v5_pi05_droid')
ckpt_root = Path('/mnt/82_store/lyh/VLA/openpi-main/checkpoints/pi05_franka_tube_pick_v5_finetune/tube_pick_v5_full_bs32_20k')
manifest = json.loads((root / 'conversion_manifest.json').read_text())
scale = float(manifest['joint_action_divisor_rad_per_s'])
width = float(manifest['max_gripper_width_m'])
stats_path = ckpt_root / '15000/assets/droid/norm_stats.json'
stats = json.loads(stats_path.read_text())['norm_stats']
report = dict(action_divisor=scale, gripper_width_m=width,
              episodes=len(manifest['episodes']), frames=0,
              physical_velocity_roundtrip_max_error=0.0,
              stored_action_conversion_max_error=0.0,
              gripper_state_max_error=0.0, gripper_target_max_error=0.0,
              quantile_roundtrip_max_error=0.0, clipped_source_values=0,
              state_quantile_roundtrip_max_error=0.0,
              stats_sha256={})
for step in (5000, 10000, 15000, 20000):
    path = ckpt_root / str(step) / 'assets/droid/norm_stats.json'
    report['stats_sha256'][str(step)] = hashlib.sha256(path.read_bytes()).hexdigest()
for item in manifest['episodes']:
    source = Path(item['source'])
    raw = pq.read_table(next(source.glob('data/**/*.parquet'))).to_pydict()
    source_fps = json.loads((source / 'meta/info.json').read_text())['fps']
    indices = np.arange(item['first_source_frame'], len(raw['joint_position']), source_fps // manifest['control_hz'])
    ep = int(item['output_episode_index'])
    stored = pq.read_table(root / f'data/chunk-000/episode_{ep:06d}.parquet').to_pydict()
    actions = np.asarray(stored['actions'], dtype=np.float64)
    velocity = np.asarray(raw['joint_velocity_command_sent'], dtype=np.float64)[indices]
    original_width = np.asarray(raw['gripper_width'], dtype=np.float64)[indices, 0]
    target = np.asarray(raw['gripper_target_width'], dtype=np.float64)[:, 0]
    target[:item['first_valid_gripper_target_frame']] = manifest['initial_gripper_target_width_m']
    expected = np.clip(velocity / scale, -1, 1)
    def maximum(key, value):
        report[key] = max(report[key], float(np.max(np.abs(value))))
    maximum('stored_action_conversion_max_error', actions[:, :7] - expected)
    maximum('physical_velocity_roundtrip_max_error', actions[:, :7] * scale - velocity)
    maximum('gripper_state_max_error', np.asarray(stored['gripper_position']).reshape(-1) - np.clip(1 - original_width / width, 0, 1))
    maximum('gripper_target_max_error', (1 - actions[:, 7]) * width - target[indices])
    lo = np.asarray(stats['actions']['q01'])[:8]
    hi = np.asarray(stats['actions']['q99'])[:8]
    normalized = (actions - lo) / (hi - lo + 1e-6) * 2 - 1
    restored = (normalized + 1) / 2 * (hi - lo + 1e-6) + lo
    maximum('quantile_roundtrip_max_error', restored - actions)
    state = np.column_stack([np.asarray(stored['joint_position']),
                             np.asarray(stored['gripper_position']).reshape(-1)])
    slo = np.asarray(stats['state']['q01'])[:8]
    shi = np.asarray(stats['state']['q99'])[:8]
    state_z = (state - slo) / (shi - slo + 1e-6) * 2 - 1
    maximum('state_quantile_roundtrip_max_error',
            (state_z + 1) / 2 * (shi - slo + 1e-6) + slo - state)
    report['clipped_source_values'] += int(np.count_nonzero(np.abs(velocity / scale) > 1))
    report['frames'] += len(actions)
assert report['stored_action_conversion_max_error'] < 1e-6, report
assert report['gripper_state_max_error'] < 1e-6, report
assert report['gripper_target_max_error'] < 1e-6, report
assert report['quantile_roundtrip_max_error'] < 1e-10, report
assert report['state_quantile_roundtrip_max_error'] < 1e-10, report
print(json.dumps(report, indent=2))
