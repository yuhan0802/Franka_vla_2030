"""Read-only: deadband labels vs hysteresis physical state: sequence + timing (label leads physical by 0-30 frames)."""
import glob, os, sys
import numpy as np, pyarrow.parquet as pq
sys.path.insert(0, os.path.expanduser("~/claude_scripts"))
from label_audit import physical
from gripper_labels_deadband import deadband_labels
for task in sys.argv[2:]:
    n = bad = 0; leads = []; ex = []
    for ep in sorted(glob.glob(os.path.join(sys.argv[1], task, "*/data/chunk-000/episode_000000.parquet"))):
        w = np.array(pq.read_table(ep, columns=["gripper_width"]).column(0).to_pylist(), np.float64).reshape(-1)
        L, P = deadband_labels(w), physical(w); n += 1
        lc = np.flatnonzero(np.diff(L)) + 1; pc = np.flatnonzero(np.diff(P)) + 1
        ok = True
        for p in pc:
            prior = lc[(lc <= p) & (lc >= p - 30) & (L[lc] == P[p])]
            if len(prior): leads.append(p - prior[-1])
            else: ok = False
        extra = [c for c in lc if not np.any((pc >= c) & (pc <= c + 30))]
        if extra or not ok:
            bad += 1; ex.append((ep.split("/")[-4], [int(x) for x in pc], [int(x) for x in lc]))
    print(f"{task}: episodes={n} problem={bad} lead_frames median={np.median(leads) if leads else -1} max={max(leads) if leads else -1}", ex[:2], flush=True)
