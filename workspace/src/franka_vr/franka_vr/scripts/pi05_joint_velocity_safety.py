#!/usr/bin/env python3
"""Safety helpers for server-denormalized pi0.5-DROID actions.

The server applies checkpoint action statistics, yielding DROID dataset units.
The verified converter stores physical velocity / (0.2 rad * 15 Hz). Invert
that dataset conversion exactly once after server-side Unnormalize.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


JOINT_NAMES = tuple(f"fr3_joint{index}" for index in range(1, 8))
JOINT_LOWER = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
JOINT_UPPER = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
EXPECTED_ACTION_SHAPE = (16, 8)
DROID_ACTION_DIVISOR_RAD_PER_SEC = 0.2 * 15
GRIPPER_MAX_WIDTH_M = 0.08


def gripper_closedness(width: float) -> float:
    return float(np.clip(1.0 - width / GRIPPER_MAX_WIDTH_M, 0.0, 1.0))


def gripper_target_width(closedness: float) -> float:
    return (1.0 - float(np.clip(closedness, 0.0, 1.0))) * GRIPPER_MAX_WIDTH_M


def validate_action_chunk(value: object) -> np.ndarray:
    actions = np.asarray(value, dtype=np.float64)
    if actions.shape != EXPECTED_ACTION_SHAPE:
        raise ValueError(f"expected action shape {EXPECTED_ACTION_SHAPE}, got {actions.shape}")
    if not np.isfinite(actions).all():
        raise ValueError("action chunk contains NaN or Inf")
    return actions.copy()


@dataclass
class GuardResult:
    velocity: np.ndarray
    raw_velocity: np.ndarray
    gripper_closedness: float
    velocity_clipped: bool
    acceleration_clipped: bool
    joint_limit_blocked: bool


class JointVelocityGuard:
    """Apply an independent conservative envelope before MoveIt Servo."""

    def __init__(
        self,
        *,
        max_velocity: float = 0.15,
        max_acceleration: float = 0.5,
        joint_limit_margin: float = 0.15,
        action_units: str = "droid",
    ) -> None:
        if not 0.0 < max_velocity <= 2.0:
            raise ValueError("max_velocity must be in (0, 2.0]")
        if not 0.0 < max_acceleration <= 3.75:
            raise ValueError("max_acceleration must be in (0, 3.75]")
        if not 0.05 <= joint_limit_margin <= 0.4:
            raise ValueError("joint_limit_margin must be in [0.05, 0.4]")
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.joint_limit_margin = float(joint_limit_margin)
        if action_units not in ("droid", "direct-rad-s"):
            raise ValueError("unknown action units")
        self.action_units = action_units
        self.previous_velocity = np.zeros(7, dtype=np.float64)

    def reset(self) -> None:
        self.previous_velocity.fill(0.0)

    def apply(self, policy_action: object, joint_positions: object, dt: float) -> GuardResult:
        action = np.asarray(policy_action, dtype=np.float64)
        positions = np.asarray(joint_positions, dtype=np.float64)
        if action.shape != (8,) or not np.isfinite(action).all():
            raise ValueError("each action must be finite with shape (8,)")
        if positions.shape != (7,) or not np.isfinite(positions).all():
            raise ValueError("joint positions must be finite with shape (7,)")
        if not 0.0 < dt <= 0.2:
            raise ValueError("control dt must be in (0, 0.2]")
        raw = action[:7].copy()
        if self.action_units == "droid":
            raw *= DROID_ACTION_DIVISOR_RAD_PER_SEC
        velocity = np.clip(raw, -self.max_velocity, self.max_velocity)
        velocity_clipped = not np.allclose(raw, velocity)

        max_delta = self.max_acceleration * dt
        accelerated = np.clip(
            velocity,
            self.previous_velocity - max_delta,
            self.previous_velocity + max_delta,
        )
        acceleration_clipped = not np.allclose(velocity, accelerated)
        velocity = accelerated

        lower = JOINT_LOWER + self.joint_limit_margin
        upper = JOINT_UPPER - self.joint_limit_margin
        outward = ((positions <= lower) & (velocity < 0.0)) | ((positions >= upper) & (velocity > 0.0))
        predicted = positions + velocity * dt
        outward |= (predicted < lower) | (predicted > upper)
        joint_limit_blocked = bool(np.any(outward))
        velocity = velocity.copy()
        velocity[outward] = 0.0
        self.previous_velocity = velocity

        return GuardResult(
            velocity=velocity,
            raw_velocity=raw,
            gripper_closedness=float(np.clip(action[7], 0.0, 1.0)),
            velocity_clipped=velocity_clipped,
            acceleration_clipped=acceleration_clipped,
            joint_limit_blocked=joint_limit_blocked,
        )
