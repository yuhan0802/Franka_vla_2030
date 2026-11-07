"""Open-loop eval: feed demo frames to the EEF server, compare 1 s predicted vs measured displacement."""
import sys, json, glob, functools, time
import numpy as np, cv2, msgpack, websockets.sync.client as wsc

def _pack(v):
    if isinstance(v, np.ndarray): return {b"__ndarray__": True, b"data": v.tobytes(), b"dtype": v.dtype.str, b"shape": v.shape}
    return v
def _unpack(v):
    if b"__ndarray__" in v: return np.ndarray(buffer=v[b"data"], dtype=np.dtype(v[b"dtype"]), shape=v[b"shape"])
    return v
PACK = functools.partial(msgpack.packb, default=_pack, use_bin_type=True)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack, raw=False)

port, stride, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
eps = sys.argv[4].split(",")
ROOT = "/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0/"
ws = wsc.connect(f"ws://127.0.0.1:{port}", max_size=None, proxy=None, open_timeout=10)
meta = UNPACK(ws.recv(timeout=10))
rows = []
def frames(path):
    c = cv2.VideoCapture(path); fs = []
    while True:
        ok, f = c.read()
        if not ok: break
        fs.append(np.ascontiguousarray(cv2.cvtColor(cv2.resize(f, (640, 480), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)))
    return fs
for ep in eps:
    d = ROOT + ep
    z = np.load(d + "/data/franka_measured_eef_base_15hz_v5.npz")
    st = z["eef_state"]; gopen = z["gripper_target_open"]
    L = frames(d + "/videos/chunk-000/observation.images.left/episode_000000.mp4")
    W = frames(d + "/videos/chunk-000/observation.images.front/episode_000000.mp4")
    n = min(len(st), len(L), len(W)) - 31
    close_idx = int(np.argmax(gopen < 0.5)) if (gopen < 0.5).any() else -1
    for t in range(0, n, stride):
        obs = {"eef_state": st[t].astype(np.float32), "exterior_image": L[t], "wrist_image": W[t],
               "instruction": "place the red test tube in the red basket"}
        t0 = time.time(); ws.send(PACK(obs)); r = UNPACK(ws.recv(timeout=60)); dt = time.time() - t0
        a = np.asarray(r.get("action", r.get("actions")), dtype=np.float64)
        pred = a[:, :3].sum(0) * 0.075
        gt = (st[t + 30, :3] - st[t, :3]).astype(np.float64)
        pred_close = int(np.argmax(a[:, 6] < 0.5)) if (a[:, 6] < 0.5).any() else -1
        rows.append(dict(ep=ep, t=t, close_idx=close_idx, pred=pred.tolist(), gt=gt.tolist(),
                         gt_grip=gopen[t:t + 30:2].tolist(), pred_grip=a[:, 6].tolist(), pred_close=pred_close,
                         state=st[t].tolist(), lat=dt))
    print(ep, "done", len(rows), flush=True)
json.dump(rows, open(out, "w"))
print("WROTE", out)
