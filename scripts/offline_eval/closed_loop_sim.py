"""Offline closed-loop rollout on demonstration scenes.

Each rollout starts from the demonstration's own first state. Actions are executed H steps at a time
exactly like eef_pose_policy_adapter (world-frame translation, left-multiplied rotation, perfect
tracking) and the integrated state is fed back. Images are the demonstration's frames at the same
elapsed time (2 frames per action step), so a policy that follows images should reproduce that
demonstration; the images do not react to the simulated arm (limitation).
Metric: distance between where the policy first closes and where the demonstration closed.
"""
import argparse
import json

import numpy as np
from scipy.spatial.transform import Rotation

from eef_eval_common import ANGULAR_SCALE_RAD, CLOSE_THRESHOLD, LINEAR_SCALE_M, Episode, Policy

DEFAULT_EPISODES = ("TS_PB_v5_011_cxc,TS_PB_v5_044_cxc,TS_PB_v5_012_cxc,TS_PB_v5_006_cxc,TS_PB_v5_035_cxc,"
                    "TS_PB_v5_046_cxc,TS_PB_v5_001_cxc,TS_PB_v5_020_cxc,TS_PB_v5_041_cxc,TS_PB_v5_048_cxc")

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, required=True)
parser.add_argument("--horizon", type=int, default=15)
parser.add_argument("--episodes", default=DEFAULT_EPISODES)
parser.add_argument("--max-chunks", type=int, default=40)
parser.add_argument("--close-max-z", type=float, default=None,
                    help="deployment guard under test: ignore close commands while the EEF is above this z (m)")
parser.add_argument("--output", required=True)
args = parser.parse_args()

policy = Policy(args.port)
rows = []
for name in args.episodes.split(","):
    episode = Episode(name)
    start = episode.states[0]
    position, rotation, gripper = start[:3].copy(), Rotation.from_rotvec(start[3:6]), start[6]
    closed_at, steps, trajectory = None, 0, [position.tolist()]
    for _ in range(args.max_chunks):
        frame = min(2 * steps, len(episode.left) - 1)
        state = np.concatenate([position, rotation.as_rotvec(), [gripper]])
        chunk = policy.infer(state, episode.left[frame], episode.wrist[frame], episode.instruction)
        for row in chunk[:args.horizon]:
            position = position + row[:3] * LINEAR_SCALE_M
            rotation = Rotation.from_rotvec(row[3:6] * ANGULAR_SCALE_RAD) * rotation
            steps += 1
            trajectory.append(position.tolist())
            if (closed_at is None and row[6] < CLOSE_THRESHOLD
                    and (args.close_max_z is None or position[2] <= args.close_max_z)):
                closed_at, gripper = position.copy(), 0.0
        if closed_at is not None:
            break
    error = None if closed_at is None else float(np.linalg.norm(closed_at[:2] - episode.grasp_position[:2]))
    rows.append(dict(episode=name, demo_grasp=episode.grasp_position.tolist(),
                     closed_at=None if closed_at is None else closed_at.tolist(),
                     xy_error_m=error, steps=steps, trajectory=trajectory))
    print(f"{name}: demo grasp {np.round(episode.grasp_position, 3)} closed at "
          f"{None if closed_at is None else np.round(closed_at, 3)} xy error "
          f"{'never closed' if error is None else f'{1000 * error:.0f} mm'}", flush=True)
errors = [r["xy_error_m"] for r in rows]
closed = [e for e in errors if e is not None]
summary = dict(
    episodes=len(rows), never_closed=errors.count(None),
    median_xy_error_mm=1000 * float(np.median(closed)) if closed else None,
    within_20mm=sum(e <= 0.02 for e in closed), over_30mm=sum(e > 0.03 for e in closed) + errors.count(None),
)
json.dump(dict(summary=summary, rows=rows), open(args.output, "w"), indent=1)
print(json.dumps(summary))
