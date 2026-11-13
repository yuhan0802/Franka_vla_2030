"""Detect tube cap (small red blob) and basket (largest red blob) in an exterior frame; map to base xy.

fit: homographies from the first frames of the demos to grasp xy (tube) and release xy (basket),
leave-one-out report, writes targets json {episode: {tube:[x,y], basket:[x,y]}} with LOO-free fits
over the training episodes only (--exclude for held-out ones, which get predictions from that fit).
"""
import argparse, cv2, glob, json, numpy as np
def red_mask(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
def detect(img_bgr):
    n, lab, st, cen = cv2.connectedComponentsWithStats(red_mask(img_bgr))
    comps = [(st[i, 4], st[i, 2], st[i, 3], cen[i]) for i in range(1, n)]
    caps = [c for c in comps if 15 <= c[0] <= 600 and max(c[1], c[2]) < 40]
    big = [c for c in comps if c[0] > 3000]
    cap = max(caps, key=lambda c: c[0])[3] if caps else None
    basket = max(big, key=lambda c: c[0])[3] if big else None
    return cap, basket
def labels(d):
    c = np.load(d + "data/franka_measured_eef_base_15hz_v5.npz"); s = c["eef_state"]; o = c["gripper_target_open"]
    i = int(np.argmax(o < 0.5)); j = i + int(np.argmax(o[i:] > 0.5)) if (o[i:] > 0.5).any() else len(s) - 1
    return s[i, :2], s[j, :2]
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--root", default="/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028/Task0/")
    ap.add_argument("--exclude", default=""); ap.add_argument("--out", default=""); a = ap.parse_args()
    ex = set(a.exclude.split(",")) - {""}
    rows = []
    for d in sorted(glob.glob(a.root + "*/")):
        ok, f = cv2.VideoCapture(d + "videos/chunk-000/observation.images.left/episode_000000.mp4").read()
        cap, basket = detect(f); g, r = labels(d); name = d.split("/")[-2]
        if cap is None or basket is None: print("detect fail", name); continue
        rows.append((name, cap, basket, g, r))
    out = {}
    for k, key in ((1, "tube"), (2, "basket")):
        P = np.float32([r[k] for r in rows]); G = np.float32([r[k + 2] for r in rows])
        err = []
        for j in range(len(P)):
            m = np.arange(len(P)) != j; H, _ = cv2.findHomography(P[m], G[m], 0)
            err.append(np.linalg.norm(cv2.perspectiveTransform(P[j:j+1, None], H)[0, 0] - G[j]) * 1000)
        err = np.array(err); print(f"{key}: LOO median {np.median(err):.1f} mm, 90% {np.percentile(err,90):.1f}, max {err.max():.1f}")
        train = np.array([r[0].split("_")[3] not in ex for r in rows])
        H, _ = cv2.findHomography(P[train], G[train], 0)
        for r, p in zip(rows, cv2.perspectiveTransform(P[:, None], H)[:, 0]): out.setdefault(r[0], {})[key] = [round(float(v), 4) for v in p]
        out.setdefault("_homography", {})[key] = H.tolist()
    if a.out: json.dump(out, open(a.out, "w"), indent=1); print("wrote", a.out, len(out) - 1)
