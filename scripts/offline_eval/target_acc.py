"""How accurate is a point-first checkpoint's own Target line? Compares generated (u, v) with the label."""
import argparse, json, re, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path.home() / "offline_eval"))
from deploy_franka_task0_qwen_actioncodec_eef import ROTATION_EXECUTION, Task0EefPolicy
from eef_eval_common import Episode
ck = sys.argv[1]
points = json.load(open("/data2/lyh/vla/config/claude_points_all50.json"))
policy = Task0EefPolicy(argparse.Namespace(device="cuda:0", rotation_execution=ROTATION_EXECUTION, checkpoint=ck, actioncodec="", state_stats=""))
held = ["044", "035", "046", "041", "048"]
seen = ["011", "012", "006", "001", "020"]
for group, ids in (("held", held), ("seen", seen)):
    errs = []
    for i in ids:
        name = f"TS_PB_v5_{i}_cxc"
        ep = Episode(name)
        label = np.array(points[name]["left"]["tube"])
        for f in (0.0, 0.3):
            t = int(ep.close_index * f)
            text = policy._point({"eef_state": ep.states[t], "exterior_image": ep.left[t], "wrist_image": ep.wrist[t], "instruction": ""})
            uv = np.array([int(x) for x in re.findall(r"\d+", text)[:2]])
            errs.append(np.linalg.norm(uv - label))
            print(group, i, f, text.strip(), "label", label.tolist(), "err(0-1000)", round(errs[-1], 1), flush=True)
    print(group, "mean err (0-1000 units)", round(float(np.mean(errs)), 1), flush=True)
