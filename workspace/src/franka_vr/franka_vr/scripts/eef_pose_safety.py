#!/usr/bin/env python3
"""ROS-independent protocol and safety core for end-effector pose policies.

Protocol (see franka_eef_pose_control_adapter.md):
  state  = [x, y, z, rotvec(3), gripper_open]   world -> fr3_link8, absolute rotvec
  action = [linear_cmd(3), angular_cmd(3), gripper_open] per 15 Hz step
           delta_position = linear_cmd * 0.075 m, delta_rotvec = angular_cmd * 0.15 rad
           rotation frame declared by the server metadata "rotation_execution":
             relative_rotvec_left_multiply  -> R_target = R_delta * R_current (world)
             relative_rotvec_right_multiply -> R_target = R_current * R_delta (tool)
           Translation is always a world/base-frame increment.

NOTE: lerobot_episode_recorder.target_delta() computes inv(q_prev) * q_cur,
i.e. a TOOL-frame delta, although its comment says q_cur * inv(q_prev).
Models trained on its eef_delta_rotation_command columns need right multiply.

Quaternions are xyzw everywhere. The rotation-vector convention matches
scipy.spatial.transform.Rotation.as_rotvec() and lerobot_episode_recorder.py:
w >= 0 and angle in [0, pi]. Only numpy is required, so this also runs in the
franka-vr image, which does not ship scipy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

STATE_PROTOCOL = "droid_eef_axis_angle6_gripper_open1_singleturn_v1"
ACTION_REPRESENTATION = "droid_base_cartesian_velocity_plus_inverted_gripper_v1"
ACTION_SHAPE = (15, 7)
ACTION_HZ = 15.0
# DROID controller command -> one physical 15 Hz step. Must equal training.
LINEAR_SCALE_M = 0.075
ANGULAR_SCALE_RAD = 0.15
# Training gripper target widths (start_franka_vr.py: Grasp 0.01, open 0.08).
GRIPPER_CLOSED_WIDTH = 0.01
GRIPPER_OPEN_WIDTH = 0.08
# The rotvec flips sign as the rotation angle crosses pi. A top-down FR3
# flange sits close to that boundary, so the adapter reports it explicitly.
ROTVEC_FLIP_WARNING_RAD = 2.9
ROTATION_EXECUTION = {"relative_rotvec_left_multiply": "world", "relative_rotvec_right_multiply": "tool"}


def normalize_quaternion(quaternion: object) -> np.ndarray:
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quaternion must be finite xyzw with shape (4,)")
    norm = float(np.linalg.norm(q))
    if not 0.5 < norm < 1.5:
        raise ValueError(f"invalid quaternion norm {norm:.6f}")
    return q / norm


def quaternion_multiply(a: object, b: object) -> np.ndarray:
    """Hamilton product a * b for xyzw quaternions (applies b first, then a)."""
    ax, ay, az, aw = np.asarray(a, dtype=np.float64)
    bx, by, bz, bw = np.asarray(b, dtype=np.float64)
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quaternion_to_rotvec(quaternion: object) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    if q[3] < 0:
        q = -q
    vector = q[:3]
    norm = float(np.linalg.norm(vector))
    angle = 2.0 * np.arctan2(norm, q[3])
    if norm < 1e-12:
        return 2.0 * vector / q[3]
    return vector * (angle / norm)


def rotvec_to_quaternion(rotvec: object) -> np.ndarray:
    r = np.asarray(rotvec, dtype=np.float64)
    if r.shape != (3,) or not np.isfinite(r).all():
        raise ValueError("rotation vector must be finite with shape (3,)")
    angle = float(np.linalg.norm(r))
    if angle < 1e-8:
        scale = 0.5 - angle * angle / 48.0
    else:
        scale = np.sin(angle / 2.0) / angle
    return np.array([*(r * scale), np.cos(angle / 2.0)])


def rotation_angle_between(a: object, b: object) -> float:
    """Smallest rotation angle (rad) between two xyzw quaternions."""
    a = normalize_quaternion(a)
    relative = quaternion_multiply(a * [-1, -1, -1, 1], normalize_quaternion(b))
    return 2.0 * float(np.arctan2(np.linalg.norm(relative[:3]), abs(relative[3])))


def gripper_open_from_width(width: float, closed_width: float = GRIPPER_CLOSED_WIDTH,
                            open_width: float = GRIPPER_OPEN_WIDTH) -> float:
    """Measured total finger opening -> 0=closed / 1=open, clipped."""
    if not np.isfinite(width) or not open_width > closed_width:
        raise ValueError("gripper width and calibration must be finite with open > closed")
    return float(np.clip((width - closed_width) / (open_width - closed_width), 0.0, 1.0))


def build_state(position: object, quaternion_xyzw: object, gripper_open: float) -> np.ndarray:
    p = np.asarray(position, dtype=np.float64)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError("position must be finite with shape (3,)")
    if not 0.0 <= gripper_open <= 1.0:
        raise ValueError("gripper_open must be in [0, 1]")
    rotvec = quaternion_to_rotvec(quaternion_xyzw)
    return np.concatenate([p, rotvec, [gripper_open]]).astype(np.float32)


def validate_action_chunk(value: object, shape: tuple = ACTION_SHAPE) -> np.ndarray:
    actions = np.asarray(value, dtype=np.float64)
    if actions.shape != tuple(shape):
        raise ValueError(f"expected action shape {tuple(shape)}, got {actions.shape}")
    if not np.isfinite(actions).all():
        raise ValueError("action chunk contains NaN or Inf")
    return actions.copy()


@dataclass(frozen=True)
class PhysicalStep:
    delta_position: np.ndarray
    delta_rotvec: np.ndarray
    gripper_open: float
    translation_clipped: bool
    rotation_clipped: bool


def physical_step(action: object, max_translation: float, max_rotation: float) -> PhysicalStep:
    """Decode one 7D action into a norm-limited physical increment.

    Norm (not per-axis) limiting keeps the commanded direction unchanged.
    """
    a = np.asarray(action, dtype=np.float64)
    if a.shape != (7,) or not np.isfinite(a).all():
        raise ValueError("action must be finite with shape (7,)")
    if max_translation <= 0 or max_rotation <= 0:
        raise ValueError("step limits must be positive")
    delta_position = a[:3] * LINEAR_SCALE_M
    delta_rotvec = a[3:6] * ANGULAR_SCALE_RAD
    translation = float(np.linalg.norm(delta_position))
    rotation = float(np.linalg.norm(delta_rotvec))
    translation_clipped = translation > max_translation
    rotation_clipped = rotation > max_rotation
    if translation_clipped:
        delta_position *= max_translation / translation
    if rotation_clipped:
        delta_rotvec *= max_rotation / rotation
    return PhysicalStep(delta_position, delta_rotvec, float(np.clip(a[6], 0.0, 1.0)),
                        translation_clipped, rotation_clipped)


class ChunkTargetIntegrator:
    """Accumulates pose targets inside one chunk from a single start pose.

    Re-reading the lagging measured pose at every step would shorten the
    trajectory because the Servo tracker is a P controller (gain 2 /s).
    """

    def __init__(self, rotation_frame: str = "world") -> None:
        if rotation_frame not in ("world", "tool"):
            raise ValueError("rotation_frame must be 'world' or 'tool'")
        self.rotation_frame = rotation_frame
        self.position: Optional[np.ndarray] = None
        self.quaternion: Optional[np.ndarray] = None

    def start(self, position: object, quaternion_xyzw: object) -> None:
        p = np.asarray(position, dtype=np.float64)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("start position must be finite with shape (3,)")
        self.position = p.copy()
        self.quaternion = normalize_quaternion(quaternion_xyzw)

    def step(self, delta_position: object, delta_rotvec: object) -> tuple[np.ndarray, np.ndarray]:
        if self.position is None or self.quaternion is None:
            raise RuntimeError("integrator has not been started")
        self.position = self.position + np.asarray(delta_position, dtype=np.float64)
        delta = rotvec_to_quaternion(delta_rotvec)
        if self.rotation_frame == "world":
            self.quaternion = quaternion_multiply(delta, self.quaternion)
        else:
            self.quaternion = quaternion_multiply(self.quaternion, delta)
        self.quaternion /= np.linalg.norm(self.quaternion)
        if self.quaternion[3] < 0:
            self.quaternion = -self.quaternion
        return self.position.copy(), self.quaternion.copy()


def inside_workspace(position: object, minimum: object, maximum: object) -> bool:
    p = np.asarray(position, dtype=np.float64)
    return bool(np.all(p >= np.asarray(minimum)) and np.all(p <= np.asarray(maximum)))


class GripperHysteresis:
    """Model gripper value (1=open) -> open/close request with a dead band."""

    def __init__(self, open_threshold: float = 0.6, close_threshold: float = 0.4) -> None:
        if not 0.0 <= close_threshold < open_threshold <= 1.0:
            raise ValueError("gripper thresholds must satisfy 0 <= close < open <= 1")
        self.open_threshold = open_threshold
        self.close_threshold = close_threshold

    def desired_open(self, value: float, current_open: Optional[bool]) -> Optional[bool]:
        """Return True/False to request open/close, or None to keep the state."""
        if value >= self.open_threshold:
            desired = True
        elif value <= self.close_threshold:
            desired = False
        else:
            return None
        return None if desired == current_open else desired


AXIS_TESTS = {
    "+x": (0, 1.0), "-x": (0, -1.0), "+y": (1, 1.0), "-y": (1, -1.0), "+z": (2, 1.0), "-z": (2, -1.0),
    "+rx": (3, 1.0), "-rx": (3, -1.0), "+ry": (4, 1.0), "-ry": (4, -1.0), "+rz": (5, 1.0), "-rz": (5, -1.0),
}


def axis_test_chunk(axis: str, magnitude: float, steps: int, gripper_open: float = 1.0) -> np.ndarray:
    """Hand-built chunk for six-direction commissioning (no policy involved)."""
    if axis not in AXIS_TESTS:
        raise ValueError(f"axis must be one of {sorted(AXIS_TESTS)}")
    if not 0 < magnitude <= 1.0 or not 1 <= steps <= ACTION_SHAPE[0]:
        raise ValueError("magnitude must be in (0, 1] and steps in [1, 15]")
    index, sign = AXIS_TESTS[axis]
    chunk = np.zeros((steps, 7))
    chunk[:, index] = sign * magnitude
    chunk[:, 6] = gripper_open
    return chunk


def joints_consistent(reference: dict, observed: dict, names: tuple, tolerance: float) -> bool:
    """True when `observed` has every joint in `names` within `tolerance` of `reference`."""
    return all(name in reference and name in observed and abs(reference[name] - observed[name]) <= tolerance
               for name in names)


class PoseStabilityWindow:
    """Reports stable once the pose stays near one sample for `duration` seconds.

    Used before arming: right after launch, TF can be driven by
    joint_state_publisher's default joints before real Franka feedback arrives.
    """

    def __init__(self, duration: float = 1.0, max_position: float = 0.001, max_rotation: float = 0.01) -> None:
        if duration <= 0 or max_position <= 0 or max_rotation <= 0:
            raise ValueError("stability window parameters must be positive")
        self.duration, self.max_position, self.max_rotation = duration, max_position, max_rotation
        self.reset()

    def reset(self) -> None:
        self.start: Optional[tuple] = None

    def add(self, t: float, position: object, quaternion_xyzw: object) -> bool:
        p, q = np.asarray(position, dtype=np.float64), normalize_quaternion(quaternion_xyzw)
        if (self.start is None or np.linalg.norm(p - self.start[1]) > self.max_position
                or rotation_angle_between(q, self.start[2]) > self.max_rotation):
            self.start = (t, p, q)
            return False
        return t - self.start[0] >= self.duration


class PoseJumpMonitor:
    """Flags measured-pose changes faster than the arm can physically move.

    MoveIt Servo caps the tracker at 0.30 m/s and 0.975 rad/s; the defaults
    allow twice that plus a small absolute slack for TF jitter.
    """

    def __init__(self, max_speed: float = 0.6, max_angular_speed: float = 2.0,
                 slack_position: float = 0.003, slack_rotation: float = 0.03) -> None:
        self.max_speed, self.max_angular_speed = max_speed, max_angular_speed
        self.slack_position, self.slack_rotation = slack_position, slack_rotation
        self.reset()

    def reset(self) -> None:
        self.last: Optional[tuple] = None

    def check(self, t: float, position: object, quaternion_xyzw: object) -> Optional[str]:
        p, q = np.asarray(position, dtype=np.float64), normalize_quaternion(quaternion_xyzw)
        reason = None
        if self.last is not None:
            dt = max(0.0, t - self.last[0])
            moved = float(np.linalg.norm(p - self.last[1]))
            turned = rotation_angle_between(q, self.last[2])
            if moved > self.max_speed * dt + self.slack_position or turned > self.max_angular_speed * dt + self.slack_rotation:
                reason = f"{moved:.4f} m / {turned:.4f} rad in {dt:.3f} s"
        self.last = (t, p, q)
        return reason
