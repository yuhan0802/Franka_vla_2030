"""Does a target-conditioned policy follow the coordinates in its instruction?

Same state and images; the tube target in the instruction is shifted by +-5 cm in x and y.
Reports how far the predicted 1 s displacement moves per shift, relative to the shift.
"""
import argparse
import json

import numpy as np

from eef_eval_common import INSTRUCTION, TARGETS, Episode, Policy, chunk_displacement, target_instruction

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, required=True)
parser.add_argument("--episodes", default="TS_PB_v5_011_cxc,TS_PB_v5_035_cxc,TS_PB_v5_046_cxc,TS_PB_v5_001_cxc")
args = parser.parse_args()
policy = Policy(args.port)
shifts = [(0.05, 0), (-0.05, 0), (0, 0.05), (0, -0.05)]
for name in args.episodes.split(","):
    episode = Episode(name)
    target = TARGETS[name]
    for fraction in (0.2, 0.5):
        t = int(episode.close_index * fraction)
        base = chunk_displacement(policy.infer(episode.states[t], episode.left[t], episode.wrist[t],
                                               target_instruction(INSTRUCTION, target["tube"], target["basket"])))
        moves = []
        for dx, dy in shifts:
            tube = [target["tube"][0] + dx, target["tube"][1] + dy]
            p = chunk_displacement(policy.infer(episode.states[t], episode.left[t], episode.wrist[t],
                                                target_instruction(INSTRUCTION, tube, target["basket"])))
            delta = p[:2] - base[:2]
            moves.append(f"shift({dx * 100:+.0f},{dy * 100:+.0f})cm->move({delta[0] * 1000:+.0f},{delta[1] * 1000:+.0f})mm")
        print(name[9:12], f"t={fraction:.1f}", "base 1s disp", np.round(base[:2] * 1000).tolist(), " ".join(moves), flush=True)
