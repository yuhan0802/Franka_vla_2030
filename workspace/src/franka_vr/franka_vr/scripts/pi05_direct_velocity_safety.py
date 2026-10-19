#!/usr/bin/env python3
"""ROS-independent reference safety core for pi0.5 direct joint velocity."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
JOINT_LOWER = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
JOINT_UPPER = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
ACTION_SHAPE = (16, 8)
DROID_TO_RAD_S = 3.0


def validate_joint_names(names: object) -> None:
    if tuple(names) != JOINT_NAMES:
        raise ValueError(f"joint names/order must be exactly {JOINT_NAMES}")


def validate_action_chunk(value: object) -> np.ndarray:
    actions = np.asarray(value, dtype=np.float64)
    if actions.shape != ACTION_SHAPE:
        raise ValueError(f"expected action shape {ACTION_SHAPE}, got {actions.shape}")
    if not np.isfinite(actions).all():
        raise ValueError("action chunk contains NaN or Inf")
    return actions.copy()


def decode_droid_action(action: object) -> tuple[np.ndarray, float]:
    value = np.asarray(action, dtype=np.float64)
    if value.shape != (8,) or not np.isfinite(value).all():
        raise ValueError("action must be finite with shape (8,)")
    return value[:7] * DROID_TO_RAD_S, float(np.clip(value[7], 0.0, 1.0))


class RecedingHorizonBuffer:
    """Replacement-only buffer; a gripper transition invalidates the chunk."""

    def __init__(self, horizon: int = 4) -> None:
        if horizon < 1 or horizon > ACTION_SHAPE[0]:
            raise ValueError("horizon must be in [1, 16]")
        self.horizon = horizon
        self._actions: list[np.ndarray] = []

    def replace(self, chunk: object) -> None:
        self._actions = [row.copy() for row in validate_action_chunk(chunk)[: self.horizon]]

    def clear_for_gripper(self) -> None:
        self._actions.clear()

    def pop(self) -> np.ndarray:
        if not self._actions:
            raise IndexError("no action available")
        return self._actions.pop(0)

    def __len__(self) -> int:
        return len(self._actions)


@dataclass(frozen=True)
class StepResult:
    velocity: np.ndarray
    acceleration: np.ndarray
    position: np.ndarray
    velocity_clipped: bool
    acceleration_clipped: bool
    jerk_clipped: bool
    joint_limit_scaled: bool
    tracking_scale: float
    fault: str | None


class DirectVelocitySafetyCore:
    """Deterministic reference implementation mirrored by the C++ executor."""

    def __init__(self, *, max_velocity: float = 0.6, max_acceleration: float = 2.0,
                 joint_limit_margin: float = 0.15,
                 soft_tracking_error: float = 0.03, hard_tracking_error: float = 0.08) -> None:
        if min(max_velocity, max_acceleration) <= 0:
            raise ValueError("velocity and acceleration limits must be positive")
        if not 0 < soft_tracking_error < hard_tracking_error:
            raise ValueError("tracking thresholds are invalid")
        self.vmax = float(max_velocity)
        self.amax = float(max_acceleration)
        self.margin = float(joint_limit_margin)
        self.soft_error = float(soft_tracking_error)
        self.hard_error = float(hard_tracking_error)
        self.velocity = np.zeros(7)
        self.acceleration = np.zeros(7)
        self.position: np.ndarray | None = None

    def reset(self, actual_position: object) -> None:
        actual = np.asarray(actual_position, dtype=np.float64)
        if actual.shape != (7,) or not np.isfinite(actual).all():
            raise ValueError("actual position must be finite with shape (7,)")
        self.position = actual.copy()
        self.velocity.fill(0)
        self.acceleration.fill(0)

    def step(self, target_velocity: object, actual_position: object, dt: float) -> StepResult:
        target = np.asarray(target_velocity, dtype=np.float64)
        actual = np.asarray(actual_position, dtype=np.float64)
        if target.shape != (7,) or actual.shape != (7,) or not np.isfinite(target).all() or not np.isfinite(actual).all():
            raise ValueError("target and feedback must be finite with shape (7,)")
        if not 0 < dt <= 0.1:
            raise ValueError("dt must be in (0, 0.1]")
        if self.position is None:
            self.reset(actual)

        error = self.position - actual
        peak_error = float(np.max(np.abs(error)))
        if peak_error > self.hard_error:
            return StepResult(self.velocity.copy(), self.acceleration.copy(), self.position.copy(),
                              False, False, False, False, 0.0, "hard tracking error")
        tracking_scale = 1.0
        if peak_error > self.soft_error:
            tracking_scale = (self.hard_error - peak_error) / (self.hard_error - self.soft_error)

        effective_vmax = max(0.0, self.vmax - self.amax * dt)
        requested = np.clip(target, -effective_vmax, effective_vmax) * tracking_scale
        velocity_clipped = not np.allclose(requested, target)
        desired_accel = (requested - self.velocity) / dt
        limited_accel = np.clip(desired_accel, -self.amax, self.amax)
        acceleration_clipped = not np.allclose(desired_accel, limited_accel)
        accel = limited_accel
        velocity = self.velocity + accel * dt
        crossed_target = (velocity - requested) * (self.velocity - requested) <= 0
        velocity[crossed_target] = requested[crossed_target]
        accel[crossed_target] = (velocity[crossed_target] - self.velocity[crossed_target]) / dt
        jerk_clipped = False

        lower, upper = JOINT_LOWER + self.margin, JOINT_UPPER - self.margin
        if np.any(actual <= lower) or np.any(actual >= upper):
            return StepResult(self.velocity.copy(), self.acceleration.copy(), self.position.copy(),
                              velocity_clipped, acceleration_clipped, jerk_clipped, True, tracking_scale,
                              "feedback inside joint-limit margin")
        predicted = self.position + velocity * dt
        blocked = ((predicted < lower) & (velocity < 0)) | ((predicted > upper) & (velocity > 0))
        joint_limit_scaled = bool(np.any(blocked))
        velocity[blocked] = 0
        accel[blocked] = 0
        self.velocity, self.acceleration = velocity, accel
        self.position = np.clip(self.position + velocity * dt, lower, upper)
        return StepResult(velocity.copy(), accel.copy(), self.position.copy(), velocity_clipped,
                          acceleration_clipped, jerk_clipped, joint_limit_scaled, tracking_scale, None)
