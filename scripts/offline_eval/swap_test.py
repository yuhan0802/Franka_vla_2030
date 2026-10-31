"""Image-reliance test: state of episode A with images of episode B.

image_reliance = |p(A state, B images) - p(A)| / |p(B) - p(A)| over 1 s displacements:
0 means the policy ignores images, 1 means it follows them completely.
"""
import argparse
import json

import numpy as np

from eef_eval_common import Episode, Policy, chunk_displacement, first_close_step

DEFAULT_PAIRS = ("TS_PB_v5_011_cxc:TS_PB_v5_048_cxc,TS_PB_v5_044_cxc:TS_PB_v5_041_cxc,"
                 "TS_PB_v5_006_cxc:TS_PB_v5_046_cxc")

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, required=True)
parser.add_argument("--pairs", default=DEFAULT_PAIRS, help="A:B pairs; each is also run as B:A")
parser.add_argument("--output", required=True)
args = parser.parse_args()

policy = Policy(args.port)
episodes, rows = {}, []
for pair in args.pairs.split(","):
    first, second = pair.split(":")
    for name in (first, second):
        episodes.setdefault(name, Episode(name))
    for a, b in ((episodes[first], episodes[second]), (episodes[second], episodes[first])):
        for fraction in np.linspace(0.2, 1.0, 9):
            ta, tb = int(a.close_index * fraction), int(b.close_index * fraction)
            own = policy.infer(a.states[ta], a.left[ta], a.wrist[ta], a.instruction)
            swap = policy.infer(a.states[ta], b.left[tb], b.wrist[tb], b.instruction)
            other = policy.infer(b.states[tb], b.left[tb], b.wrist[tb], b.instruction)
            p_own, p_swap, p_other = map(chunk_displacement, (own, swap, other))
            rows.append(dict(A=a.name, B=b.name, fraction=float(fraction), p_own=p_own.tolist(),
                             p_swap=p_swap.tolist(), p_other=p_other.tolist(),
                             close_own=first_close_step(own), close_swap=first_close_step(swap),
                             close_other=first_close_step(other)))
moved = [np.linalg.norm(np.subtract(r["p_swap"], r["p_own"])) for r in rows]
gap = [np.linalg.norm(np.subtract(r["p_other"], r["p_own"])) for r in rows]
informative = [(m, g) for m, g in zip(moved, gap) if g > 0.005]
summary = dict(
    samples=len(rows),
    mean_swap_shift_mm=1000 * float(np.mean(moved)),
    mean_other_gap_mm=1000 * float(np.mean(gap)),
    image_reliance=float(np.mean([m / g for m, g in informative])) if informative else None,
    informative_samples=len(informative),
)
json.dump(dict(summary=summary, rows=rows), open(args.output, "w"), indent=1)
print(json.dumps(summary))
