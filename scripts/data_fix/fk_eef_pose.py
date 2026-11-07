#!/usr/bin/env python3
"""Replace eef_position / eef_quaternion in a COPY of raw Franka_vr episodes with FR3 forward
kinematics of the recorded joint_position (world == fr3_link0, flange = fr3_link8).

Why: the recorder samples TF world->fr3_link8, which only updates every 60 ms, so ~40% of 30 fps
frames repeat a stale pose (up to 60 ms old) and the v5 2-frame deltas vary 0.73-1.71x for the
same motion; some episodes' frame 0 even holds the previous pose. joint_position is ~1 ms fresh,
and FK matches time-aligned TF to 0.04 mm / 0.005 deg (median).
Usage: fk_eef_pose.py TASK_DIR   (refuses paths under /mnt/82_store/vla)
"""
import json, os, sys
from pathlib import Path
import numpy as np, pyarrow as pa, pyarrow.parquet as pq
from scipy.spatial.transform import Rotation as Rot

A = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0]
D = [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107]
AL = [0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2, 0]


def fk(q):
    T = np.eye(4)
    for i in range(8):
        th = q[i] if i < 7 else 0.0
        ca, sa, ct, st = np.cos(AL[i]), np.sin(AL[i]), np.cos(th), np.sin(th)
        T = T @ np.array([[ct, -st, 0, A[i]], [st * ca, ct * ca, -sa, -D[i] * sa],
                          [st * sa, ct * sa, ca, D[i] * ca], [0, 0, 0, 1]])
    return T


if __name__ == "__main__":
    root = Path(sys.argv[1]).resolve()
    assert not str(root).startswith("/mnt/82_store/vla"), "never write the canonical raw data"
    for ep in sorted(p for p in root.iterdir() if p.is_dir()):
        path = ep / "data/chunk-000/episode_000000.parquet"
        table = pq.read_table(path)
        q = np.array(table.column("joint_position").to_pylist(), np.float64)
        old_q = np.array(table.column("eef_quaternion").to_pylist(), np.float64)
        T = [fk(x) for x in q]
        pos = np.array([t[:3, 3] for t in T])
        quat = Rot.from_matrix(np.array([t[:3, :3] for t in T])).as_quat()  # xyzw like the recorder
        quat[np.sum(quat * old_q, axis=1) < 0] *= -1                        # keep the recorded sign convention
        for name, value in (("eef_position", pos), ("eef_quaternion", quat)):
            i = table.schema.get_field_index(name); f = table.schema.field(i)
            table = table.set_column(i, f, pa.array(value.astype(np.float32).tolist(), type=f.type))
        tmp = path.with_suffix(".tmp.parquet"); pq.write_table(table, tmp); os.replace(tmp, path)
        info_path = ep / "meta/info.json"; info = json.loads(info_path.read_text())
        info["eef_pose_source"] = ("2026-11-03: eef_position/eef_quaternion = FR3 FK of joint_position "
                                   "(TF world->fr3_link8 updated only every 60 ms); script fk_eef_pose.py")
        info_path.write_text(json.dumps(info, indent=2))
        print(ep.name, "frames", len(q), flush=True)
