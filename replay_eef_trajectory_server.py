#!/usr/bin/env python3
"""Replay a recorded Franka EEF trajectory through the EEF policy-server protocol.

Drop-in replacement for deploy_franka_task0_qwen_actioncodec_eef.py: the adapter
connects exactly as it does to the model server, but every response is the next
slice of a preprocessed real-robot cache (franka_measured_eef_base_15hz_v4/v5.npz)
instead of a model prediction. No model, GPU or images are used.

The cache stores one 15-step chunk per 30 Hz source frame; chunk i spans frames
i..i+30 and each step is a measured 2-frame world-frame displacement. The server
serves chunk 0, then advances by 2 * execute_horizon source frames per request, so
chunks line up with what the adapter executed (rotation: world, left multiply).
The 30 frames not covered by the last full stride are served as the tail of the
final chunk (steps already executed zeroed); afterwards the server answers with idle
arm actions and the final gripper command. --verify integrates the chunks back into
poses and compares them with the cache's measured eef_state (the inverse of
preprocess_franka_task_measured_eef_v4/v5.py).

run_eef_pose.sh passes the model flags; --checkpoint is the .npz path here and
--actioncodec / --state-stats are accepted and ignored. The replay position is
process-wide (a reconnect never rewinds it); restart the server to replay again.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import threading
from pathlib import Path

import numpy as np
import torch

PROJECT_ROOT = Path(os.environ.get("VLA_PROJECT_ROOT", "/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA"))
spec = importlib.util.spec_from_file_location(
    "franka_network_utils", PROJECT_ROOT / "omnigibson/learning/utils/network_utils.py")
network_utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(network_utils)

# Must equal the constants the adapter checks (eef_pose_safety.py).
STATE_PROTOCOL = "droid_eef_axis_angle6_gripper_open1_singleturn_v1"
ACTION_REPRESENTATION = "droid_base_cartesian_velocity_plus_inverted_gripper_v1"
MODEL_IMAGE_SIZE = 256
CHUNK_STEPS = 15
SOURCE_FRAMES_PER_STEP = 2


class TrajectoryReplayPolicy:
    def __init__(self, path: Path, execute_horizon: int):
        if not path.is_file():
            raise FileNotFoundError(f"trajectory cache not found: {path}")
        if not 1 <= execute_horizon <= CHUNK_STEPS:
            raise ValueError(f"execute-horizon must be in [1, {CHUNK_STEPS}]")
        with np.load(path, allow_pickle=False) as cache:
            self.chunks = np.asarray(cache["droid_action_chunks"], dtype=np.float32)
            self.states = np.asarray(cache["eef_state"], dtype=np.float64)
            self.protocol = str(cache["protocol"]) if "protocol" in cache.files else "unknown"
            self.rotation_execution = str(cache["rotation_execution"])
        if self.chunks.ndim != 3 or self.chunks.shape[1:] != (CHUNK_STEPS, 7) or not np.isfinite(self.chunks).all():
            raise ValueError(f"unexpected action chunks {self.chunks.shape} in {path}")
        if self.rotation_execution not in ("relative_rotvec_left_multiply", "relative_rotvec_right_multiply"):
            raise ValueError(f"unknown rotation_execution {self.rotation_execution!r} in {path}")
        self.path = path
        self.stride = SOURCE_FRAMES_PER_STEP * execute_horizon
        self.index = 0
        self.lock = threading.Lock()

    def chunk_at(self, index: int) -> np.ndarray | None:
        """Chunk covering source frames index..index+30, or None past the end of the recording.

        Past the last chunk start, the tail is cut from the last chunk whose start has the same
        parity as `index` (steps are 2 frames long); the already executed steps are dropped and the
        chunk is padded with idle arm actions holding the final gripper command.
        """
        last = len(self.chunks) - 1
        if index <= last:
            return self.chunks[index].copy()
        if index - last >= CHUNK_STEPS * SOURCE_FRAMES_PER_STEP:
            return None
        start = last - ((last - index) % SOURCE_FRAMES_PER_STEP)
        skip = (index - start) // SOURCE_FRAMES_PER_STEP
        chunk = np.zeros((CHUNK_STEPS, 7), dtype=np.float32)
        chunk[:, 6] = self.chunks[start, -1, 6]
        chunk[:CHUNK_STEPS - skip] = self.chunks[start, skip:]
        return chunk

    def act(self, observation) -> torch.Tensor:
        with self.lock:
            chunk = self.chunk_at(self.index)
            if chunk is not None:
                print(f"[replay] chunk source frame {self.index}/{len(self.chunks) - 1}", flush=True)
            else:
                chunk = np.zeros((CHUNK_STEPS, 7), dtype=np.float32)
                chunk[:, 6] = self.chunks[-1, -1, 6]
                print("[replay] trajectory finished; serving idle actions", flush=True)
            self.index += self.stride
        return torch.from_numpy(chunk)

    def reset(self) -> None:
        pass

    def verify(self) -> None:
        """Integrate the chunks (world-frame left multiply, as the adapter does) and compare with eef_state."""
        from scipy.spatial.transform import Rotation
        left = self.rotation_execution == "relative_rotvec_left_multiply"
        position, rotation = self.states[0, :3].copy(), Rotation.from_rotvec(self.states[0, 3:6])
        frame, index, worst_position, worst_rotation, steps = 0, 0, 0.0, 0.0, 0
        while frame + SOURCE_FRAMES_PER_STEP < len(self.states):
            chunk = self.chunk_at(index)
            if chunk is None: break
            for step in range(CHUNK_STEPS):
                if frame + SOURCE_FRAMES_PER_STEP >= len(self.states): break
                position = position + chunk[step, :3].astype(np.float64) * 0.075
                delta = Rotation.from_rotvec(chunk[step, 3:6].astype(np.float64) * 0.15)
                rotation = delta * rotation if left else rotation * delta
                frame += SOURCE_FRAMES_PER_STEP; steps += 1
                truth = Rotation.from_rotvec(self.states[frame, 3:6])
                worst_position = max(worst_position, float(np.abs(position - self.states[frame, :3]).max()))
                worst_rotation = max(worst_rotation, float((rotation * truth.inv()).magnitude()))
            index += CHUNK_STEPS * SOURCE_FRAMES_PER_STEP
        print(f"[verify] {self.path.name}: {steps} steps to frame {frame}/{len(self.states) - 1}; "
              f"max position error {worst_position:.2e} m, max rotation error {worst_rotation:.2e} rad "
              f"(rotation_execution={self.rotation_execution})", flush=True)
        if worst_position > 1e-4 or worst_rotation > 1e-3:
            sys.exit("[verify] FAILED: chunks do not reconstruct the measured poses")

    def metadata(self, port: int, actioncodec: str, state_stats: str) -> dict:
        return {
            "model_family": "franka_eef_trajectory_replay",
            "checkpoint": str(self.path),  # the adapter cross-checks this against --checkpoint
            "actioncodec": actioncodec, "state_stats": state_stats,
            "state_protocol": STATE_PROTOCOL,
            "action_representation": ACTION_REPRESENTATION,
            "action_shape": [CHUNK_STEPS, 7], "action_hz": 15,
            "gripper_convention": "0_closed_1_open",
            "gripper_state_source": "measured",
            "rotation_execution": self.rotation_execution,
            "observation_format": "franka_task0_main_wrist_640x480_eef7_v1",
            "model_image_size": [MODEL_IMAGE_SIZE, MODEL_IMAGE_SIZE],
            "replay_protocol": self.protocol, "replay_chunks": int(len(self.chunks)),
            "port": port,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True, help="path of the franka_measured_eef_base_15hz_*.npz cache")
    parser.add_argument("--actioncodec", default=""); parser.add_argument("--state-stats", default="")
    parser.add_argument("--rotation-execution", default="", help="ignored: taken from the cache")
    parser.add_argument("--execute-horizon", type=int, default=CHUNK_STEPS,
                        help="steps the adapter executes per chunk (must equal its --execute-horizon)")
    parser.add_argument("--verify", action="store_true", help="check the cache reconstructs eef_state, then exit")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8098)
    args = parser.parse_args()
    policy = TrajectoryReplayPolicy(Path(args.checkpoint), args.execute_horizon)
    if args.rotation_execution and args.rotation_execution != policy.rotation_execution:
        sys.exit(f"cache rotation_execution={policy.rotation_execution!r} contradicts "
                 f"--rotation-execution {args.rotation_execution!r}")
    if args.verify:
        policy.verify(); return
    metadata = policy.metadata(args.port, args.actioncodec, args.state_stats)
    print(f"[replay] loaded {policy.path} ({len(policy.chunks)} chunks, protocol {policy.protocol}); "
          f"serving ws://{args.host}:{args.port} stride={policy.stride} frames", flush=True)
    network_utils.WebsocketPolicyServer(policy=policy, host=args.host, port=args.port, metadata=metadata).serve_forever()


if __name__ == "__main__":
    main()
