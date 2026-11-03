#!/usr/bin/env python3
"""Rewrite gripper_target_width / action[7] in a COPY of raw Franka_vr episodes.

The 2026-11-01 teleop node published CLOSE (0.01) for squeezes that physically
opened the gripper (failed Grasp kept gripper_state "open"; see
docs/task3_gripper_label_bug_20261103.md). Each received gripper message keeps its
receipt time; its value is set to what the gripper actually did next: the first
deadband open/close transition of the measured width within 2 s after receipt
(0.08 open / 0.01 close), or the current measured state if it did not move.
Usage: fix_raw_gripper_targets.py TASK_DIR   (refuses paths under /mnt/82_store/vla)
"""
import json, os, sys
from pathlib import Path
import numpy as np, pyarrow as pa, pyarrow.parquet as pq
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gripper_labels_deadband import deadband_labels

OPEN, CLOSED, LOOKAHEAD = 0.08, 0.01, 60
root = Path(sys.argv[1]).resolve()
assert not str(root).startswith("/mnt/82_store/vla"), "never write the canonical raw data"


def rewrite(path, values):
    table = pq.read_table(path)
    for name, new in values.items():
        i = table.schema.get_field_index(name)
        table = table.set_column(i, table.schema.field(i), pa.array(new, type=table.schema.field(i).type))
    tmp = path.with_suffix(".tmp.parquet")
    pq.write_table(table, tmp)
    os.replace(tmp, path)


report = []
for ep in sorted(p for p in root.iterdir() if p.is_dir()):
    data = ep / "data/chunk-000/episode_000000.parquet"
    rows = pq.read_table(data, columns=["timestamp", "gripper_width", "gripper_target_width", "action", "timestamps"]).to_pylist()
    t = np.array([r["timestamp"] for r in rows])
    width = np.array([r["gripper_width"][0] for r in rows])
    label = deadband_labels(width)
    changes = np.flatnonzero(np.diff(label) != 0) + 1

    def value_at(receipt_time):
        i = min(int(np.searchsorted(t, receipt_time)), len(t) - 1)
        nxt = changes[(changes >= i - 5) & (changes <= i + LOOKAHEAD)]
        return OPEN if (label[nxt[0]] if len(nxt) else label[i]) == 1.0 else CLOSED

    def fixed(rs):
        cmd, act, n = [], [], 0
        for r in rs:
            rt = r["timestamps"]["gripper_target_width"]
            c = r["gripper_target_width"][0]
            if rt and rt > 0 and c is not None and np.isfinite(c):
                v = value_at(rt); n += v != c; c = v
            a = list(r["action"]); a[7] = c
            cmd.append([c]); act.append(a)
        return cmd, act, n

    cmd, act, changed = fixed(rows)
    rewrite(data, {"gripper_target_width": cmd, "action": act})
    apath = ep / "actions/chunk-000/episode_000000.parquet"
    arows = pq.read_table(apath, columns=["gripper_target_width", "action", "timestamps"]).to_pylist()
    acmd, aact, achanged = fixed(arows)
    rewrite(apath, {"gripper_target_width": acmd, "action": aact})
    info_path = ep / "meta/info.json"
    info = json.loads(info_path.read_text())
    info["gripper_target_fix"] = ("2026-11-03: gripper_target_width/action[7] re-derived from the measured width "
                                  "outcome of each gripper message (teleop failed-Grasp state bug); "
                                  "script fix_raw_gripper_targets.py")
    info_path.write_text(json.dumps(info, indent=2))
    report.append((ep.name, changed, achanged))
    print(ep.name, "frames changed", changed, "action rows changed", achanged, flush=True)
