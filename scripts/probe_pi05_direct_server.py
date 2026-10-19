#!/usr/bin/env python3
"""Read-only synthetic probe for the pi0.5-DROID direct policy server."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "workspace/src/franka_vr/franka_vr/scripts"
sys.path.insert(0, str(SCRIPTS))
from pi05_direct_policy_adapter import PolicyClient  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="10.24.116.66")
    parser.add_argument("--port", type=int, default=8094)
    parser.add_argument("--instruction", default="put the red test tube into the red basket")
    args = parser.parse_args()
    client = PolicyClient(args.host, args.port)
    try:
        print("METADATA " + json.dumps(client.metadata, default=str, sort_keys=True))
        observation = {
            "observation/exterior_image_1_left": np.zeros((480, 640, 3), dtype=np.uint8),
            "observation/wrist_image_left": np.zeros((480, 640, 3), dtype=np.uint8),
            "observation/joint_position": np.zeros(7, dtype=np.float32),
            "observation/gripper_position": np.zeros(1, dtype=np.float32),
            "prompt": args.instruction,
        }
        started = time.monotonic()
        actions = client.infer(observation)
        elapsed = time.monotonic() - started
        print(
            f"ACTION shape={list(actions.shape)} dtype={actions.dtype} finite={bool(np.isfinite(actions).all())} "
            f"min={actions.min():.7g} max={actions.max():.7g} latency_s={elapsed:.3f}"
        )
        print("READ_ONLY_PI05_DIRECT_PROBE_OK robot_command_sent=false")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
