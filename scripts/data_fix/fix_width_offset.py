#!/usr/bin/env python3
"""Subtract an unhomed-gripper width offset in a COPY of raw Franka_vr episodes.

An unhomed FR3 gripper reports width + offset (e.g. 142 mm open, 71 mm when firmly closed on
the object). The offset is found by width_offset_audit.py (held and open levels both shifted).
Fixes gripper_width, gripper_position and observation.state[7] (fr3 joints + measured width).
Usage: fix_width_offset.py TASK_DIR EPISODE:OFFSET_M [...]   (refuses paths under /mnt/82_store/vla)
"""
import json, os, sys
from pathlib import Path
import numpy as np, pyarrow as pa, pyarrow.parquet as pq

root = Path(sys.argv[1]).resolve()
assert not str(root).startswith("/mnt/82_store/vla"), "never write the canonical raw data"
for spec in sys.argv[2:]:
    name, offset = spec.split(":"); offset = float(offset)
    path = root / name / "data/chunk-000/episode_000000.parquet"
    table = pq.read_table(path)
    width = np.array(table.column("gripper_width").to_pylist(), np.float64)[:, 0]
    fixed = np.clip(width - offset, 0.0, None)
    state = np.array(table.column("observation.state").to_pylist(), np.float64)
    assert np.allclose(state[:, 7], width), "observation.state[7] is not the measured width"
    state[:, 7] = fixed
    for col, value in (("gripper_width", fixed[:, None]), ("gripper_position", fixed[:, None]), ("observation.state", state)):
        i = table.schema.get_field_index(col); f = table.schema.field(i)
        table = table.set_column(i, f, pa.array(value.tolist(), type=f.type))
    tmp = path.with_suffix(".tmp.parquet"); pq.write_table(table, tmp); os.replace(tmp, path)
    info_path = root / name / "meta/info.json"; info = json.loads(info_path.read_text())
    info["gripper_width_offset_fix"] = f"2026-11-03: subtracted {offset:.4f} m (unhomed gripper); script fix_width_offset.py"
    info_path.write_text(json.dumps(info, indent=2))
    print(f"{name}: width {width.min()*1000:.1f}-{width.max()*1000:.1f} mm -> {fixed.min()*1000:.1f}-{fixed.max()*1000:.1f} mm")
