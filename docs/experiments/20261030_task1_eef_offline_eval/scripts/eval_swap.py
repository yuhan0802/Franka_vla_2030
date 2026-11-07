"""Swap test: state from episode A, images from episode B (same relative progress). Does prediction follow images or state?"""
import sys, json, functools
import numpy as np, cv2, msgpack, websockets.sync.client as wsc
def _pack(v):
    if isinstance(v, np.ndarray): return {b"__ndarray__": True, b"data": v.tobytes(), b"dtype": v.dtype.str, b"shape": v.shape}
    return v
def _unpack(v):
    if b"__ndarray__" in v: return np.ndarray(buffer=v[b"data"], dtype=np.dtype(v[b"dtype"]), shape=v[b"shape"])
    return v
PACK = functools.partial(msgpack.packb, default=_pack, use_bin_type=True)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack, raw=False)
ROOT = "/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0/"
ws = wsc.connect(f"ws://127.0.0.1:{int(sys.argv[1])}", max_size=None, proxy=None); ws.recv(timeout=10)
def load(ep):
    z = np.load(ROOT + ep + "/data/franka_measured_eef_base_15hz_v5.npz")
    def fr(cam):
        c = cv2.VideoCapture(f"{ROOT}{ep}/videos/chunk-000/observation.images.{cam}/episode_000000.mp4"); out = []
        while True:
            ok, f = c.read()
            if not ok: break
            out.append(np.ascontiguousarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)))
        return out
    g = z["gripper_target_open"]
    return dict(st=z["eef_state"], close=int(np.argmax(g < 0.5)), L=fr("left"), W=fr("front"))
def infer(st, L, W):
    ws.send(PACK({"eef_state": st.astype(np.float32), "exterior_image": L, "wrist_image": W,
                  "instruction": "place the red test tube in the red basket"}))
    a = np.asarray(UNPACK(ws.recv(timeout=60))["action"], dtype=np.float64)
    return a[:, :3].sum(0) * 0.075, (int(np.argmax(a[:, 6] < 0.5)) if (a[:, 6] < 0.5).any() else -1)
pairs = [p.split(":") for p in sys.argv[3].split(",")]
rows = []
for A, B in pairs:
    a, b = load(A), load(B)
    print(A, "grasp", a["st"][a["close"], :3], B, "grasp", b["st"][b["close"], :3], flush=True)
    for frac in np.linspace(0.2, 1.0, 9):
        ta, tb = int(a["close"] * frac), int(b["close"] * frac)
        ta = min(ta, a["close"]); tb = min(tb, b["close"])
        p_own, c_own = infer(a["st"][ta], a["L"][ta], a["W"][ta])
        p_swap, c_swap = infer(a["st"][ta], b["L"][tb], b["W"][tb])     # A state, B images
        p_bown, c_bown = infer(b["st"][tb], b["L"][tb], b["W"][tb])
        gtA = a["st"][ta + 30, :3] - a["st"][ta, :3]; gtB = b["st"][tb + 30, :3] - b["st"][tb, :3]
        rows.append(dict(A=A, B=B, frac=float(frac), p_own=p_own.tolist(), p_swap=p_swap.tolist(), p_bown=p_bown.tolist(),
                         gtA=gtA.tolist(), gtB=gtB.tolist(), c_own=c_own, c_swap=c_swap, c_bown=c_bown,
                         dA_grasp=(a["st"][a["close"], :3] - a["st"][ta, :3]).tolist(), dB_grasp=(b["st"][b["close"], :3] - b["st"][tb, :3]).tolist()))
json.dump(rows, open(sys.argv[2], "w")); print("WROTE")
