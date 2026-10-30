"""Shared helpers for offline EEF policy checks against a running deploy server."""
from __future__ import annotations

import functools
from pathlib import Path

import cv2
import msgpack
import numpy as np
import websockets.sync.client as wsc

DATA_ROOT = Path("/data2/lyh/vla/Franka_vr_all_real_actioncodec_clean_20261028")
INSTRUCTION = "place the red test tube in the red basket"
LINEAR_SCALE_M, ANGULAR_SCALE_RAD = 0.075, 0.15
# Gripper commands below this value close, matching the adapter's hysteresis close threshold.
CLOSE_THRESHOLD = 0.4


def _pack(value):
    if isinstance(value, np.ndarray):
        return {b"__ndarray__": True, b"data": value.tobytes(), b"dtype": value.dtype.str, b"shape": value.shape}
    return value


def _unpack(value):
    if b"__ndarray__" in value:
        return np.ndarray(buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]), shape=value[b"shape"])
    return value


PACK = functools.partial(msgpack.packb, default=_pack, use_bin_type=True)
UNPACK = functools.partial(msgpack.unpackb, object_hook=_unpack, raw=False)


class Policy:
    def __init__(self, port: int, host: str = "127.0.0.1"):
        self.ws = wsc.connect(f"ws://{host}:{port}", max_size=None, proxy=None, open_timeout=10)
        self.metadata = UNPACK(self.ws.recv(timeout=10))

    def infer(self, state, exterior, wrist) -> np.ndarray:
        self.ws.send(PACK({"eef_state": np.asarray(state, dtype=np.float32), "exterior_image": exterior,
                           "wrist_image": wrist, "instruction": INSTRUCTION}))
        return np.asarray(UNPACK(self.ws.recv(timeout=60))["action"], dtype=np.float64)


class Episode:
    """A Task0 demonstration: 30 Hz states, gripper targets and RGB frames (left exterior + wrist)."""

    def __init__(self, name: str, task: str = "Task0"):
        root = DATA_ROOT / task / name
        cache = np.load(root / "data/franka_measured_eef_base_15hz_v5.npz")
        self.name, self.states = name, cache["eef_state"].astype(np.float64)
        opened = cache["gripper_target_open"]
        self.close_index = int(np.argmax(opened < 0.5)) if (opened < 0.5).any() else -1
        self.left = self._frames(root / "videos/chunk-000/observation.images.left/episode_000000.mp4")
        self.wrist = self._frames(root / "videos/chunk-000/observation.images.front/episode_000000.mp4")

    @staticmethod
    def _frames(path: Path) -> list:
        capture, frames = cv2.VideoCapture(str(path)), []
        while True:
            ok, frame = capture.read()
            if not ok:
                return frames
            frames.append(np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))

    @property
    def grasp_position(self) -> np.ndarray:
        return self.states[self.close_index, :3]


def chunk_displacement(chunk: np.ndarray) -> np.ndarray:
    return chunk[:, :3].sum(axis=0) * LINEAR_SCALE_M


def first_close_step(chunk: np.ndarray) -> int:
    closing = chunk[:, 6] < CLOSE_THRESHOLD
    return int(np.argmax(closing)) if closing.any() else -1
