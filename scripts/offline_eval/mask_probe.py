"""Second control: ridge probe on the colour detector's red-pixel mask (downsampled), which contains the
tube location exactly. Shows what a linear probe can reach with 45 training scenes on a spatial map."""
import sys

import cv2
import numpy as np

sys.path.insert(0, "/home/xmu/claude_tube")
sys.path.insert(0, "/home/xmu/offline_eval")
from targets import red_mask  # noqa: E402
from eef_eval_common import DATA_ROOT, Episode  # noqa: E402

FRACTIONS = (0.0, 0.15, 0.3, 0.45)
names = sorted(p.name for p in (DATA_ROOT / "Task0").iterdir() if p.name.startswith("TS_PB"))
rows = {"mask16": [], "mask32": []}
meta = []
for name in names:
    ep = Episode(name)
    for f in FRACTIONS:
        t = int(ep.close_index * f)
        m = red_mask(cv2.cvtColor(ep.left[t], cv2.COLOR_RGB2BGR)).astype(np.float32) / 255
        for s in (16, 32):
            rows[f"mask{s}"].append(cv2.resize(m, (s, s), interpolation=cv2.INTER_AREA).reshape(-1))
        meta.append((name, f, *ep.grasp_position[:2]))
np.savez("probe_mask.npz", names=np.array([m[0] for m in meta]), frac=np.array([m[1] for m in meta]),
         grasp=np.array([m[2:4] for m in meta]), **{k: np.stack(v) for k, v in rows.items()})
