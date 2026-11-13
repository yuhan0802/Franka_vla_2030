"""Offline closed-loop rollout: integrate model actions exactly like the adapter (H steps, world left-multiply),
feeding back the integrated state. Images come from a chosen source; the arm 'executes' perfectly."""
import sys, json, glob, functools
import numpy as np, cv2, msgpack, websockets.sync.client as wsc
from scipy.spatial.transform import Rotation as Rot
def _pack(v):
    if isinstance(v, np.ndarray): return {b"__ndarray__": True, b"data": v.tobytes(), b"dtype": v.dtype.str, b"shape": v.shape}
    return v
def _unpack(v):
    if b"__ndarray__" in v: return np.ndarray(buffer=v[b"data"], dtype=np.dtype(v[b"dtype"]), shape=v[b"shape"])
    return v
PACK = functools.partial(msgpack.packb, default=_pack, use_bin_type=True)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack, raw=False)
ROOT = "/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0/"
port, H, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
sources = sys.argv[4].split(",")
ws = wsc.connect(f"ws://127.0.0.1:{port}", max_size=None, proxy=None); ws.recv(timeout=10)
REAL = "/tmp/claude_real/eef_task1_task01-newcodec-10k_20261030_153212_64316"
real_obs = [json.loads(l) for l in open(REAL + "/policy.jsonl")]
real_obs = [r for r in real_obs if r["event"] == "observation"]
start = np.array(real_obs[0]["eef_state"], dtype=np.float64)
def demo_frames(ep):
    z = np.load(ROOT + ep + "/data/franka_measured_eef_base_15hz_v5.npz")
    def fr(cam):
        c = cv2.VideoCapture(f"{ROOT}{ep}/videos/chunk-000/observation.images.{cam}/episode_000000.mp4"); o = []
        while True:
            ok, f = c.read()
            if not ok: break
            o.append(np.ascontiguousarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)))
        return o
    return fr("left"), fr("front"), z["eef_state"][int(np.argmax(z["gripper_target_open"] < 0.5)), :3]
results = {}
for src in sources:
    if src == "real":
        L = [cv2.cvtColor(cv2.imread(f"{REAL}/obs_{i:04d}_exterior.jpg"), cv2.COLOR_BGR2RGB) for i in range(len(real_obs))]
        W = [cv2.cvtColor(cv2.imread(f"{REAL}/obs_{i:04d}_wrist.jpg"), cv2.COLOR_BGR2RGB) for i in range(len(real_obs))]
        img = lambda k: (L[min(k, len(L) - 1)], W[min(k, len(W) - 1)]); demo_grasp = None
    else:
        L, W, demo_grasp = demo_frames(src)
        img = lambda k, L=L, W=W: (L[min(k * 2 * H, len(L) - 1)], W[min(k * 2 * H, len(W) - 1)])
    for seed in range(int(sys.argv[5]) if len(sys.argv) > 5 else 1):
        pos = start[:3].copy(); R = Rot.from_rotvec(start[3:6]); grip = start[6]; traj = [pos.tolist()]; close_at = None; steps = 0
        for k in range(40):
            e, w = img(k)
            st = np.concatenate([pos, R.as_rotvec(), [grip]]).astype(np.float32)
            ws.send(PACK({"eef_state": st, "exterior_image": e, "wrist_image": w, "instruction": "place the red test tube in the red basket"}))
            a = np.asarray(UNPACK(ws.recv(timeout=60))["action"], dtype=np.float64)
            for row in a[:H]:
                pos = pos + row[:3] * 0.075; R = Rot.from_rotvec(row[3:6] * 0.15) * R; steps += 1
                if close_at is None and row[6] < 0.4: close_at = pos.copy(); grip = 0.0
                traj.append(pos.tolist())
            if close_at is not None and steps > 0 and k > 0 and np.linalg.norm(pos - close_at) > 0.08: break
        key = f"{src}#{seed}"
        results[key] = dict(close=None if close_at is None else close_at.tolist(), traj=traj,
                            demo_grasp=None if demo_grasp is None else demo_grasp.tolist())
        print(key, "close at", None if close_at is None else np.round(close_at, 3), "demo grasp", None if demo_grasp is None else np.round(demo_grasp, 3), flush=True)
json.dump(results, open(out, "w")); print("WROTE")
