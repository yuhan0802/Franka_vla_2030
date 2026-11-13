"""Read-only: are recorded state (EEF pose, gripper) and v5 actions correct?
Usage: verify_state_action.py TASK_DIR [--npz]   (prints per-episode stats and a summary)"""
import glob, os, sys
import numpy as np, pyarrow.parquet as pq, cv2
from scipy.spatial.transform import Rotation as Rot

# FR3 modified-DH (Craig), flange = fr3_link8
A = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0]
D = [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107]
AL = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, 0]
def fk(q):
    T = np.eye(4)
    for i in range(8):
        th = q[i] if i < 7 else 0.0
        ca, sa, ct, st = np.cos(AL[i]), np.sin(AL[i]), np.cos(th), np.sin(th)
        T = T @ np.array([[ct, -st, 0, A[i]], [st*ca, ct*ca, -sa, -D[i]*sa], [st*sa, ct*sa, ca, D[i]*ca], [0, 0, 0, 1]])
    return T

root = sys.argv[1]; use_npz = "--npz" in sys.argv
print("| episode | FK pos err mm (med/max) | FK rot err deg (max) | eef age ms (p50/p99) | joint age ms p99 | img-state ms p99 | frame gap max ms | stale eef frames | video frames ok | npz action ok |")
print("|---|---|---|---|---|---|---|---|---|---|")
summary = []
for ep in sorted(p for p in glob.glob(os.path.join(root, "*")) if os.path.isdir(p)):
    t = pq.read_table(f"{ep}/data/chunk-000/episode_000000.parquet",
                      columns=["timestamp", "joint_position", "eef_position", "eef_quaternion", "timestamps"]).to_pylist()
    ts = np.array([r["timestamp"] for r in t]); q = np.array([r["joint_position"] for r in t])
    p = np.array([r["eef_position"] for r in t]); quat = np.array([r["eef_quaternion"] for r in t])
    st = {k: np.array([r["timestamps"][k] for r in t]) for k in ("eef", "joint_state", "exterior_image_left", "exterior_image_right", "wrist_image")}
    Ts = [fk(x) for x in q]
    pe = np.array([np.linalg.norm(T[:3, 3] - pp) for T, pp in zip(Ts, p)]) * 1000
    re = np.array([np.degrees((Rot.from_matrix(T[:3, :3]).inv() * Rot.from_quat(qq)).magnitude()) for T, qq in zip(Ts, quat)])
    eef_age = (ts - st["eef"]) * 1000; joint_age = (ts - st["joint_state"]) * 1000
    img_dt = np.max([np.abs(st[k] - st["eef"]) for k in ("exterior_image_left", "exterior_image_right", "wrist_image")], axis=0) * 1000
    gap = np.diff(ts).max() * 1000
    dq = np.abs(np.diff(q, axis=0)).max(1); dp = np.linalg.norm(np.diff(p, axis=0), axis=1)
    stale = int(((dq > 2e-3) & (dp < 1e-6)).sum())
    vid_ok = True
    for cam in ("front", "left", "right"):
        n = int(cv2.VideoCapture(f"{ep}/videos/chunk-000/observation.images.{cam}/episode_000000.mp4").get(cv2.CAP_PROP_FRAME_COUNT))
        vid_ok &= n == len(t)
    act_ok = "-"
    if use_npz:
        z = np.load(f"{ep}/data/franka_measured_eef_base_15hz_v5.npz"); s = z["eef_state"]; ch = z["droid_action_chunks"]
        R_ = Rot.from_rotvec(s[:, 3:6]); n = len(ch); k = np.arange(15)
        dpos = (s[2:, :3] - s[:-2, :3]) / 0.075; drot = (R_[2:] * R_[:-2].inv()).as_rotvec() / 0.15
        idx = np.arange(n)[:, None] + 2 * k[None]
        act_ok = bool(np.allclose(ch[..., :3], dpos[idx], atol=1e-4) and np.allclose(ch[..., 3:6], drot[idx], atol=1e-4)
                      and np.allclose(s[:, :3], p, atol=1e-6))
    row = (os.path.basename(ep), np.median(pe), pe.max(), re.max(), np.median(eef_age), np.percentile(eef_age, 99),
           np.percentile(joint_age, 99), np.percentile(img_dt, 99), gap, stale, vid_ok, act_ok)
    summary.append(row)
    print("| %s | %.1f / %.1f | %.2f | %.0f / %.0f | %.0f | %.0f | %.0f | %d | %s | %s |" % row, flush=True)
a = np.array([r[1:10] for r in summary], float)
print(f"\nSUMMARY {len(summary)} episodes: FK pos max {a[:,1].max():.1f} mm, rot max {a[:,2].max():.2f} deg, "
      f"eef age p99 max {a[:,4].max():.0f} ms, img-state p99 max {a[:,6].max():.0f} ms, frame gap max {a[:,7].max():.0f} ms, "
      f"stale frames {int(a[:,8].sum())}, videos ok {all(r[10] for r in summary)}, actions ok {set(r[11] for r in summary)}")
