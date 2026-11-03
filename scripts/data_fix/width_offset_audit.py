"""Read-only: per-episode gripper width offset (unhomed gripper) vs the task's typical held/open widths."""
import glob, os, sys
import numpy as np, pyarrow.parquet as pq
sys.path.insert(0, os.path.expanduser("~/claude_scripts"))
from gripper_labels_deadband import deadband_labels
root = sys.argv[1]
for task in sorted(glob.glob(os.path.join(root, "Task*"))):
    eps = {}
    for ep in sorted(glob.glob(task + "/*/data/chunk-000/episode_000000.parquet")):
        w = np.array(pq.read_table(ep, columns=["gripper_width"]).column(0).to_pylist())[:, 0]
        L = deadband_labels(w)
        held = float(np.percentile(w[L == 0], 10)) if (L == 0).sum() > 30 else np.nan   # firmly closed level
        top = float(np.percentile(w, 99))                                               # open level
        eps[ep.split("/")[-4]] = (held, top)
    H = np.nanmedian([v[0] for v in eps.values()]); T = np.nanmedian([v[1] for v in eps.values()])
    out = [(k, round((h - H) * 1000, 1), round((t - T) * 1000, 1)) for k, (h, t) in eps.items()
           if (np.isfinite(h) and abs(h - H) > 0.005) and abs(t - T) > 0.005]
    print(f"{os.path.basename(task)}: n={len(eps)} typical held {H*1000:.1f} mm, open {T*1000:.1f} mm; "
          f"offset episodes (held & open both shifted >5 mm): {out}", flush=True)
