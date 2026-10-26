#!/usr/bin/env python3
"""Validate and store the measured FR3 reset pose (no hardware access)."""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import tempfile

NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
LOWER = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
UPPER = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]


def validate(q):
    if (not isinstance(q, list) or len(q) != 7
            or any(not isinstance(v, (int, float)) or not math.isfinite(v)
                   or not lo + .02 <= v <= hi - .02 for v, lo, hi in zip(q, LOWER, UPPER))):
        raise ValueError("reset pose must contain seven finite FR3 joint angles within limits")
    return q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("capture", "target"))
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("robot_ip")
    args = parser.parse_args()
    if args.mode == "capture":
        records = [json.loads(line) for line in args.source.read_text().splitlines() if line.startswith("{")]
        state = next(r for r in records if r.get("event") == "arm_initial_state")
        q = validate(state["q"])
        dq = state["dq"]
        if state["robot_mode"] != 1 or len(dq) != 7 or any(not math.isfinite(v) or abs(v) > .02 for v in dq):
            raise ValueError("capture requires an idle, stationary arm")
        pose = {"version": 1, "robot_ip": args.robot_ip,
                "captured_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "joint_names": NAMES, "q_rad": q, "measured_dq_rad_s": dq}
        args.destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode="w", dir=args.destination.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(pose, stream, indent=2)
            stream.write("\n")
        os.replace(temporary, args.destination)
        print(f"Saved measured reset pose: {args.destination}\nq_rad={q}")
    else:
        pose = json.loads(args.source.read_text())
        if pose.get("version") != 1 or pose.get("robot_ip") != args.robot_ip or pose.get("joint_names") != NAMES:
            raise ValueError("reset pose robot identity or joint order does not match")
        q = validate(pose["q_rad"])
        args.destination.write_text(" ".join(format(v, ".17g") for v in q) + "\n")
        print(f"Return target q_rad={q}")


if __name__ == "__main__":
    main()
