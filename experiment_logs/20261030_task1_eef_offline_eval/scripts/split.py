"""Split gate closed-loop errors into episodes seen by act-v2 (001-029) and the 5 held out of holdout5."""
import json
import statistics
import sys

HELD = {"TS_PB_v5_035_cxc", "TS_PB_v5_041_cxc", "TS_PB_v5_044_cxc", "TS_PB_v5_046_cxc", "TS_PB_v5_048_cxc"}
for name, path in zip(sys.argv[1::2], sys.argv[2::2]):
    try:
        out = json.load(open(f"{path}/summary.json"))
        rows = json.load(open(f"{path}/closed_loop.json"))["rows"]
    except Exception as e:
        print(f"{name:22s} missing ({e.__class__.__name__})"); continue
    def err(r):
        e = r.get("xy_error_m"); e = None if e is None else e * 1000
        return 999.0 if e is None else e
    seen = [err(r) for r in rows if r["episode"] not in HELD]
    held = [err(r) for r in rows if r["episode"] in HELD]
    loop = out["closed_loop"]
    print(f"{name:22s} img {out['swap']['image_reliance']:.2f}  med {loop['median_xy_error_mm'] or 999:6.1f}  "
          f"<=20 {loop['within_20mm']:2d}  >30 {loop['over_30mm']:2d}  never {loop['never_closed']:2d} | "
          f"seen {' '.join(f'{x:4.0f}' for x in seen)} | held {' '.join(f'{x:4.0f}' for x in held)}")
