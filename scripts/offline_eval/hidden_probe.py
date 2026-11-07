"""Is the tube position linearly decodable from the VLA's hidden states?

Separates "the model does not see where the tube is" from "it sees it but the action head snaps to a
memorized trajectory". For each demo, 4 frames before the grasp (tube still in the rack) go through
the deployed prompt; ridge regression maps hidden states to the demo's grasp xy. Fit on the 45
holdout5 training scenes, test on the 5 held-out ones; also leave-one-scene-out over the 45.
Usage: hidden_probe.py extract CKPT OUT.npz | hidden_probe.py analyze A.npz [B.npz ...]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HELD = ["TS_PB_v5_044_cxc", "TS_PB_v5_035_cxc", "TS_PB_v5_046_cxc", "TS_PB_v5_041_cxc", "TS_PB_v5_048_cxc"]
LAYERS = (8, 14, 20, 28)
FRACTIONS = (0.0, 0.15, 0.3, 0.45)


def extract(checkpoint, out):
    import torch
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.path.insert(0, str(Path.home() / "offline_eval"))
    from deploy_franka_task0_qwen_actioncodec_eef import ROTATION_EXECUTION, Task0EefPolicy
    from eef_eval_common import DATA_ROOT, Episode
    policy = Task0EefPolicy(argparse.Namespace(device="cuda:0", rotation_execution=ROTATION_EXECUTION, checkpoint=checkpoint,
                                               actioncodec="", state_stats=""))
    pad = policy.processor.tokenizer.convert_tokens_to_ids("<|image_pad|>")
    names = sorted(p.name for p in (DATA_ROOT / "Task0").iterdir() if p.name.startswith("TS_PB"))
    rows = {f"act{l}": [] for l in LAYERS} | {f"img{l}": [] for l in LAYERS}
    meta = []
    for name in names:
        ep = Episode(name)
        for f in FRACTIONS:
            t = int(ep.close_index * f)
            raw = {"eef_state": ep.states[t].astype(np.float32), "exterior_image": ep.left[t], "wrist_image": ep.wrist[t],
                   "instruction": ""}
            encoded, end, state, _ = policy._prompt(raw)
            ids = encoded.input_ids[:, :end].to(policy.device)
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
                output = policy.model(input_ids=ids, states=state.to(policy.device), attention_mask=torch.ones_like(ids),
                                      pixel_values=encoded.pixel_values.to(policy.device, dtype=torch.bfloat16),
                                      image_grid_thw=encoded.image_grid_thw.to(policy.device), output_hidden_states=True)
            is_pad = (ids[0] == pad).cpu().numpy()
            # first contiguous image-pad run = main (exterior) camera
            start = int(np.argmax(is_pad)); stop = start + int(np.argmin(is_pad[start:]))
            for l in LAYERS:
                h = output.hidden_states[l][0].float().cpu().numpy()
                rows[f"act{l}"].append(h[-1].astype(np.float16))
                rows[f"img{l}"].append(h[start:stop].reshape(-1).astype(np.float16))
            meta.append((name, f, *ep.grasp_position[:2], *ep.states[t, :2]))
        print(name, ep.close_index, stop - start, "image tokens", flush=True)
    np.savez(out, names=np.array([m[0] for m in meta]), frac=np.array([m[1] for m in meta]),
             grasp=np.array([m[2:4] for m in meta]), eef=np.array([m[4:6] for m in meta]), **{k: np.stack(v) for k, v in rows.items()})


def ridge_predict(Xtr, ytr, Xte, alpha):
    mu, sd = Xtr.mean(0), Xtr.std(0) + 1e-6
    A, B = (Xtr - mu) / sd, (Xte - mu) / sd
    ym = ytr.mean(0)
    K = A @ A.T
    w = np.linalg.solve(K + alpha * np.trace(K) / len(K) * np.eye(len(K)), ytr - ym)
    return (B @ A.T) @ w + ym


def analyze(paths):
    for path in paths:
        d = np.load(path)
        names, grasp = d["names"], d["grasp"]
        held = np.isin(names, HELD)
        train_eps = sorted(set(names[~held]))
        # how far each held scene is from its nearest training grasp (the memorization floor)
        print(f"== {path}")
        for key in [k for k in d.files if k.startswith(("act", "img", "mask"))]:
            X = d[key].astype(np.float32)
            best = None
            for alpha in (0.01, 0.1, 1.0, 10.0):
                loo = []
                for e in train_eps:
                    tr, te = (~held) & (names != e), names == e
                    p = ridge_predict(X[tr], grasp[tr], X[te], alpha)
                    loo.append(np.linalg.norm(p - grasp[te], axis=1).mean() * 1000)
                if best is None or np.median(loo) < best[1]:
                    best = (alpha, np.median(loo))
            alpha = best[0]
            p = ridge_predict(X[~held], grasp[~held], X[held], alpha)
            err = {e: np.linalg.norm(p[names[held] == e] - grasp[held][names[held] == e], axis=1).mean() * 1000 for e in HELD}
            # nearest training scene in feature space (cosine), the "recognize the scene" readout
            A = X / np.linalg.norm(X, axis=1, keepdims=True)
            nn = (A[held] @ A[~held].T).argmax(1)
            nn_err = {e: np.linalg.norm(grasp[~held][nn][names[held] == e] - grasp[held][names[held] == e], axis=1).mean() * 1000
                      for e in HELD}
            print(f"{key:6s} alpha {alpha:5g}  LOO-45 median {best[1]:5.1f} mm | held ridge "
                  + " ".join(f"{err[e]:5.1f}" for e in HELD) + f"  mean {np.mean(list(err.values())):5.1f}"
                  + " | held NN " + " ".join(f"{nn_err[e]:5.1f}" for e in HELD), flush=True)
    g = {}
    for n, xy in zip(names, grasp):
        g[n] = xy
    tr = np.array([g[n] for n in train_eps])
    print("nearest training grasp (oracle memorization floor):",
          " ".join(f"{np.linalg.norm(tr - g[e], axis=1).min() * 1000:5.1f}" for e in HELD),
          " | predict-mean baseline:", " ".join(f"{np.linalg.norm(tr.mean(0) - g[e]) * 1000:5.1f}" for e in HELD))


if __name__ == "__main__":
    if sys.argv[1] == "extract":
        extract(sys.argv[2], sys.argv[3])
    else:
        analyze(sys.argv[2:])
