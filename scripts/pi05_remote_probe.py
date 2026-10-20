#!/usr/bin/env python3
"""不接触机器人接口的 π0.5-DROID WebSocket 合成观测探针。"""

from __future__ import annotations

import argparse
import json

import numpy as np
from openpi_client import msgpack_numpy
import websockets.sync.client


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8094)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()

    # 首次 JAX/XLA 编译可能超过默认的 20 秒 WebSocket 心跳。探针使用一个
    # 明确的总接收超时，不用心跳把正常的首次编译误判成断线。
    connection = websockets.sync.client.connect(
        f"ws://{args.host}:{args.port}",
        compression=None,
        max_size=None,
        open_timeout=10,
        close_timeout=2,
        ping_interval=None,
    )
    try:
        metadata = msgpack_numpy.unpackb(connection.recv(timeout=10))
        observation = {
            "observation/exterior_image_1_left": np.zeros((224, 224, 3), dtype=np.uint8),
            "observation/exterior_image_2_left": np.zeros((224, 224, 3), dtype=np.uint8),
            "observation/wrist_image_left": np.zeros((224, 224, 3), dtype=np.uint8),
            "observation/joint_position": np.zeros((7,), dtype=np.float32),
            "observation/gripper_position": np.zeros((1,), dtype=np.float32),
            "prompt": args.instruction,
        }
        connection.send(msgpack_numpy.Packer().pack(observation))
        response = msgpack_numpy.unpackb(connection.recv(timeout=args.timeout))
    finally:
        connection.close()

    actions = np.asarray(response["actions"])
    if actions.shape != (16, 8):
        raise SystemExit(f"Refusing: expected actions shape (16, 8), got {actions.shape}")
    if not np.isfinite(actions).all():
        raise SystemExit("Refusing: action response contains NaN or Inf")
    if np.any(actions < -1.0001) or np.any(actions > 1.0001):
        raise SystemExit("Refusing: normalized action is outside [-1,1]")
    print(json.dumps({
        "metadata": metadata,
        "actions_shape": list(actions.shape),
        "actions_min": float(actions.min()),
        "actions_max": float(actions.max()),
        "action_semantics": "normalized joint velocity[7] + gripper closedness[1]",
        "gripper_semantics": "0=open, 1=closed",
        "robot_command_sent": False,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
