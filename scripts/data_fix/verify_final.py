"""Read-only final verification of a corrected task copy vs the NFS original.
Usage: verify_final.py NEW_TASK_DIR RAW_TASK_DIR [--seq COCOCO]  (--seq: required gripper label sequence)"""
import glob, hashlib, os, sys
import numpy as np, pyarrow.parquet as pq, cv2
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.expanduser("~/claude_scripts"))
from fk_eef_pose import fk
NEW, RAW = sys.argv[1], sys.argv[2]
SEQ = sys.argv[sys.argv.index("--seq") + 1] if "--seq" in sys.argv else None
ALLOWED = {"gripper_target_width", "eef_position", "eef_quaternion", "gripper_width", "gripper_position", "observation.state"}
md5 = lambda p: hashlib.md5(open(p, "rb").read()).hexdigest()
P = []; stats = {"windows": 0, "episodes": 0}
def longest(m):
    r = b = 0
    for x in m: r = r + 1 if x else 0; b = max(b, r)
    return b
for ep in sorted(os.listdir(RAW)):
    n_, r_ = f"{NEW}/{ep}", f"{RAW}/{ep}"; stats["episodes"] += 1
    for f in glob.glob(f"{r_}/**/*", recursive=True):
        if os.path.isdir(f) or f.endswith(".parquet") or f.endswith("meta/info.json"): continue
        if md5(f) != md5(f.replace(r_, n_)): P.append(f"{ep}: file differs {f}")
    for sub in ("data", "actions"):
        a = pq.read_table(f"{r_}/{sub}/chunk-000/episode_000000.parquet"); b = pq.read_table(f"{n_}/{sub}/chunk-000/episode_000000.parquet")
        if a.schema != b.schema or a.num_rows != b.num_rows: P.append(f"{ep} {sub}: schema/rows differ")
        for c in a.column_names:
            if c in ALLOWED: continue
            x, y = a.column(c).to_pylist(), b.column(c).to_pylist()
            if c == "action": x = [r[:7] for r in x]; y = [r[:7] for r in y]
            if c == "observation.state": x = [r[:7] for r in x]; y = [r[:7] for r in y]
            if x != y: P.append(f"{ep} {sub}: column {c} differs")
    t = pq.read_table(f"{n_}/data/chunk-000/episode_000000.parquet", columns=["joint_position", "eef_position", "eef_quaternion", "gripper_width"]).to_pylist()
    q = np.array([r["joint_position"] for r in t]); p = np.array([r["eef_position"] for r in t]); qu = np.array([r["eef_quaternion"] for r in t])
    T = [fk(x) for x in q]
    if max(np.linalg.norm(T_[:3, 3] - pp) for T_, pp in zip(T, p)) > 1e-5: P.append(f"{ep}: eef_position != FK")
    if max((Rot.from_matrix(T_[:3, :3]).inv() * Rot.from_quat(qq)).magnitude() for T_, qq in zip(T, qu)) > 1e-5: P.append(f"{ep}: eef_quaternion != FK")
    for cam in ("front", "left", "right"):
        if int(cv2.VideoCapture(f"{n_}/videos/chunk-000/observation.images.{cam}/episode_000000.mp4").get(cv2.CAP_PROP_FRAME_COUNT)) != len(t): P.append(f"{ep}: video {cam} frame count")
    z = np.load(f"{n_}/data/franka_measured_eef_base_15hz_v5.npz"); s = z["eef_state"]; ch = z["droid_action_chunks"]; g = z["gripper_target_open"]; v = z["valid_start_indices"]
    stats["windows"] += len(v)
    if not np.allclose(s[:, :3], p, atol=1e-6): P.append(f"{ep}: npz state position != parquet")
    if not np.allclose(Rot.from_rotvec(s[:, 3:6]).as_matrix(), Rot.from_quat(qu).as_matrix(), atol=1e-5): P.append(f"{ep}: npz state rotation != parquet")
    w = np.array([r["gripper_width"][0] for r in t])
    if not np.allclose(s[:, 6], np.clip((w - .01) / .07, 0, 1), atol=1e-6): P.append(f"{ep}: npz gripper state != width")
    R_ = Rot.from_rotvec(s[:, 3:6]); idx = np.arange(len(ch))[:, None] + 2 * np.arange(15)[None]
    if not (np.allclose(ch[..., :3], ((s[2:, :3] - s[:-2, :3]) / .075)[idx], atol=1e-4) and np.allclose(ch[..., 3:6], ((R_[2:] * R_[:-2].inv()).as_rotvec() / .15)[idx], atol=1e-4)):
        P.append(f"{ep}: arm actions != state deltas")
    okc = (ch[..., 6] == g[idx]) | (ch[..., 6] == g[idx + 1]) | (ch[..., 6] == g[idx + 2])
    if not okc.all(): P.append(f"{ep}: chunk gripper inconsistent")
    seq = "".join("O" if g[i] else "C" for i in np.flatnonzero(np.diff(g)) + 1)
    if SEQ and seq != SEQ: P.append(f"{ep}: label sequence {seq}")
    if w.max() > 0.08: P.append(f"{ep}: gripper width {w.max():.4f} m > 0.08 (unhomed offset left)")
    raw_w = np.array(pq.read_table(f"{r_}/data/chunk-000/episode_000000.parquet", columns=["gripper_width"]).column(0).to_pylist())[:, 0]
    import json as _json
    offset_fixed = "gripper_width_offset_fix" in _json.load(open(f"{n_}/meta/info.json"))
    if not offset_fixed and not np.array_equal(raw_w, w): P.append(f"{ep}: gripper width changed without an offset fix")
    if offset_fixed:
        d = raw_w - w; d = d[w > 0]
        if d.size == 0 or np.ptp(d) > 1e-6: P.append(f"{ep}: offset fix is not a constant shift")
    sw = np.array(pq.read_table(f"{n_}/data/chunk-000/episode_000000.parquet", columns=["observation.state"]).column(0).to_pylist())[:, 7]
    if not np.allclose(sw, w): P.append(f"{ep}: observation.state[7] != gripper_width")
    if longest((g < .5) & (s[:, 6] > .85)) > 30 or longest((g > .5) & (s[:, 6] < .15)) > 45: P.append(f"{ep}: label contradicts measured gripper")
    jumps = np.flatnonzero(np.abs(np.diff(q, axis=0)).max(1) > 0.2)  # > 6 rad/s: not physical motion
    for j in jumps:
        covered = v[(v <= j) & (v + 30 >= j + 1)]
        if len(covered): P.append(f"{ep}: joint jump at frame {j}->{j+1} inside {len(covered)} training windows")
    if len(jumps): print(f"note {ep}: joint jump(s) at {jumps.tolist()} (excluded from all windows: {not any('joint jump' in x and ep in x for x in P)})")
print(stats); print("PROBLEMS:", len(P)); print("\n".join(P[:40]))
