"""Read-only: v5 measured-direction gripper labels vs physical open/close, for eps values."""
import glob, os, sys
import numpy as np, pyarrow.parquet as pq

OPEN_W, CLOSED_W = 0.08, 0.01

def labels(state_open, eps):
    out = np.empty(len(state_open), np.float32); last = 1.0
    for i in range(len(state_open)):
        if i + 1 < len(state_open):
            d = float(state_open[i + 1] - state_open[i])
            if d > eps: last = 1.0
            elif d < -eps: last = 0.0
        out[i] = last
    return out

def physical(width):
    # hysteresis open/closed state from measured width
    s, cur = [], 1 if width[0] > 0.04 else 0
    for w in width:
        if cur and w < 0.03: cur = 0
        elif not cur and w > 0.06: cur = 1
        s.append(cur)
    return np.array(s)

def seq(x): return "".join("O" if x[i] else "C" for i in range(1, len(x)) if x[i] != x[i - 1])

if __name__ == "__main__":
    root = sys.argv[1]
    for task in sorted(glob.glob(os.path.join(root, "Task*"))):
        eps_bad = {}
        n = 0
        for ep in sorted(glob.glob(os.path.join(task, "*/data/chunk-000/episode_000000.parquet"))):
            w = np.array(pq.read_table(ep, columns=["gripper_width"]).column(0).to_pylist(), np.float64).reshape(-1)
            so = np.clip((w - CLOSED_W) / (OPEN_W - CLOSED_W), 0, 1)
            ph = seq(physical(w)); n += 1
            for eps in (1e-4, 0.007):
                if seq(labels(so, eps)) != ph: eps_bad[eps] = eps_bad.get(eps, 0) + 1
        print(f"{os.path.basename(task)}: episodes={n} wrong_label_seq eps1e-4={eps_bad.get(1e-4,0)} eps0.007={eps_bad.get(0.007,0)}", flush=True)
