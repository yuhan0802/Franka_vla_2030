"""Teacher-forced (per-frame) accuracy on demonstrations.

At demo time t the policy gets the demo's own state and images (frame 2t), so there is no compounding
and no stale-image problem of closed_loop_sim.py. Errors are 3D. For each of the first --grasps closes
of the demonstration (Task3: 1 = lid handle, 2 = bottle):
  approach: |state[t] + integrated 15-step chunk - state[t+15]| for t in the 3 s (45 steps) before it;
  close timing: for t in the 15 steps before it, the chunk closes at step close-t (+-3 steps).
endpoint: the same 15-step error over the whole demonstration up to the last evaluated close.
"""
import argparse
import json

import numpy as np

from eef_eval_common import LINEAR_SCALE_M, Episode, Policy, first_close_step

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, required=True)
parser.add_argument("--episodes", required=True)
parser.add_argument("--grasps", type=int, default=2)
parser.add_argument("--stride", type=int, default=5)
parser.add_argument("--output", required=True)
args = parser.parse_args()

policy = Policy(args.port)
cache = {}


def infer(ep, t):
    if (ep.name, t) not in cache:
        frame = min(2 * t, len(ep.left) - 1)
        cache[ep.name, t] = policy.infer(ep.states[t], ep.left[frame], ep.wrist[frame], ep.instruction)
    return cache[ep.name, t]


def endpoint_error(ep, t):
    pred = ep.states[t, :3] + infer(ep, t)[:15, :3].sum(axis=0) * LINEAR_SCALE_M
    return 1000 * float(np.linalg.norm(pred - ep.states[t + 15, :3]))


rows, endpoint = [], []
approach = {g: [] for g in range(args.grasps)}
timing = {g: [] for g in range(args.grasps)}
for name in args.episodes.split(","):
    ep = Episode(name)
    closed = ep.gripper_target_open < 0.5
    closes = [int(i) + 1 for i in np.flatnonzero(~closed[:-1] & closed[1:])]
    if closed[0]:
        closes.insert(0, 0)
    closes = closes[:args.grasps]
    for g, c in enumerate(closes):
        for t in range(max(0, c - 45), c, args.stride):
            err = endpoint_error(ep, t)
            approach[g].append(err)
            rows.append(dict(episode=name, grasp=g, t=t, close=c, endpoint_mm=err))
        for t in range(max(0, c - 15), c):
            step = first_close_step(infer(ep, t))
            timing[g].append(bool(step >= 0 and abs(step - (c - t)) <= 3))
    for t in range(0, closes[-1], 3 * args.stride):
        endpoint.append(endpoint_error(ep, t))
    print(name, "closes", closes, flush=True)
summary = dict(endpoint_median_mm=float(np.median(endpoint)))
for g in range(args.grasps):
    if approach[g]:
        summary[f"grasp{g + 1}_approach_median_mm"] = float(np.median(approach[g]))
        summary[f"grasp{g + 1}_approach_p90_mm"] = float(np.percentile(approach[g], 90))
        summary[f"grasp{g + 1}_close_timing_ok"] = float(np.mean(timing[g]))
json.dump(dict(summary=summary, rows=rows), open(args.output, "w"), indent=1)
print("TF", json.dumps(summary))
