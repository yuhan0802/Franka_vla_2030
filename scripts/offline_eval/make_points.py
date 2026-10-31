"""Per-episode tube-cap / basket pixel targets (Qwen 0-1000 coords) in the left and right exterior cameras.

Written for the auxiliary pointing objective (FRANKA_POINTS_JSON): before the grasp the target is the tube
cap, afterwards the basket. Frames are resized (no crop) in training, so relative coords survive resizing.
"""
import glob, json, sys

import cv2
import numpy as np

sys.path.insert(0, "/home/xmu/claude_tube")
from targets import detect  # noqa: E402

ROOT = "/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0/"
out, fails = {}, []
for d in sorted(glob.glob(ROOT + "*/")):
    name = d.split("/")[-2]
    o = np.load(d + "data/franka_measured_eef_base_15hz_v5.npz")["gripper_target_open"]
    row = {"close": int(np.argmax(o < 0.5))}
    for cam in ("left", "right"):
        ok, f = cv2.VideoCapture(f"{d}videos/chunk-000/observation.images.{cam}/episode_000000.mp4").read()
        cap, basket = detect(f) if ok else (None, None)
        if cap is None or basket is None:
            fails.append((name, cam)); continue
        h, w = f.shape[:2]
        row[cam] = {k: [int(round(1000 * p[0] / w)), int(round(1000 * p[1] / h))] for k, p in (("tube", cap), ("basket", basket))}
    out[name] = row
    print(name, row, flush=True)
print("fails", fails)
json.dump(out, open("/data2/lyh/vla/config/claude_points_all50.json", "w"), indent=1)
