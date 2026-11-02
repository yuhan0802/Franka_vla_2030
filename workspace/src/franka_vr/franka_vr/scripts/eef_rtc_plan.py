"""Real-time chunking (RTC) plan math for the EEF pose adapter; numpy only, no ROS.

The next chunk is requested while the current one still executes. Each chunk is integrated
into absolute waypoints from the target that was commanded when its observation was taken:
waypoint i is the target of the (i+1)-th step after that observation. When the chunk arrives
d steps later its first d waypoints are already in the past and are skipped, and the next
blend_steps steps cross-fade from the previous chunk (linear position, slerp rotation), so the
target path stays continuous across the hand-over.

Black, Galliker & Levine 2025 ("Real-Time Execution of Action Chunking Flow Policies") freeze
the committed prefix and inpaint the rest during flow sampling; the ActionCodec token policy
cannot be steered that way, so the overlap is blended instead (as LeRobot async inference
aggregates overlapping chunks).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from eef_pose_safety import ChunkTargetIntegrator, normalize_quaternion, physical_step, quaternion_multiply


@dataclass
class RtcPlan:
    obs_step: int             # steps executed when the observation was taken
    arrival_step: int         # steps executed when the chunk arrived
    start_position: np.ndarray
    start_quaternion: np.ndarray
    positions: np.ndarray     # (H, 3) absolute targets
    quaternions: np.ndarray   # (H, 4) xyzw
    grippers: np.ndarray      # (H,) model gripper value, 1 = open
    clipped: list = field(default_factory=list)
    fade_steps: int = 0       # steps after arrival that cross-fade from fade_from / the previous plan
    fade_from: Optional[tuple] = None   # held target when the previous plan had run out

    def index(self, step: int) -> Optional[int]:
        i = step - self.obs_step
        return i if 0 <= i < len(self.positions) else None

    @property
    def translation(self) -> float:
        return float(np.linalg.norm(self.positions[-1] - self.start_position))


def build_plan(chunk: np.ndarray, start_position, start_quaternion, rotation_frame: str,
               action_gain: Sequence[float], max_translation: float, max_rotation: float,
               obs_step: int, arrival_step: int) -> RtcPlan:
    """Integrate a (H, 7) chunk into absolute waypoints exactly as the serial adapter does."""
    integrator = ChunkTargetIntegrator(rotation_frame)
    integrator.start(start_position, start_quaternion)
    positions, quaternions, grippers, clipped = [], [], [], []
    for action in np.asarray(chunk, dtype=float):
        effective = action.copy(); effective[:6] *= np.asarray(action_gain, dtype=float)
        step = physical_step(effective, max_translation, max_rotation)
        position, quaternion = integrator.step(step.delta_position, step.delta_rotvec)
        positions.append(position); quaternions.append(quaternion); grippers.append(step.gripper_open)
        clipped.append((step.translation_clipped, step.rotation_clipped))
    return RtcPlan(obs_step, arrival_step, np.asarray(start_position, float).copy(),
                   normalize_quaternion(start_quaternion), np.array(positions), np.array(quaternions),
                   np.array(grippers), clipped)


def slerp(a, b, weight: float) -> np.ndarray:
    a, b = normalize_quaternion(a), normalize_quaternion(b)
    dot = float(np.dot(a, b))
    if dot < 0.0: b, dot = -b, -dot
    if dot > 0.9995: return normalize_quaternion(a + weight * (b - a))
    theta = math.acos(dot)
    return (math.sin((1.0 - weight) * theta) * a + math.sin(weight * theta) * b) / math.sin(theta)


def hand_over(previous: Optional[RtcPlan], new: RtcPlan, blend_steps: int, held: Optional[tuple]) -> None:
    """Fix how `new` takes over at its arrival step.

    It fades from the previous plan over at most the steps that plan still covers, or from the held
    target when the previous plan ran out before the answer arrived; the first plan does not fade.
    """
    if previous is not None:
        remaining = len(previous.positions) - (new.arrival_step - previous.obs_step)
        if remaining > 0:
            new.fade_steps = min(blend_steps, remaining); return
    if previous is not None and held is not None:
        new.fade_steps, new.fade_from = blend_steps, held


def target_at(plans: Sequence[RtcPlan], step: int):
    """(position, quaternion, gripper, info) for `step`, or None when no plan covers it.

    plans is [previous, active] or [active]; the active plan's gripper value always wins.
    """
    if not plans: return None
    active = plans[-1]
    i = active.index(step)
    if i is None: return None
    position, quaternion = active.positions[i], active.quaternions[i]
    info = dict(rtc_obs_step=active.obs_step, rtc_index=i, rtc_blend_weight=1.0)
    since_arrival = step - active.arrival_step
    if 0 <= since_arrival < active.fade_steps:
        weight = (since_arrival + 1) / (active.fade_steps + 1)
        if active.fade_from is None:
            j = plans[-2].index(step) if len(plans) > 1 else None
            if j is not None:
                position = (1.0 - weight) * plans[-2].positions[j] + weight * position
                quaternion = slerp(plans[-2].quaternions[j], quaternion, weight)
                info.update(rtc_blend_weight=weight, rtc_blend_source="previous")
        else:
            # The arm holds a target the new plan already passed or not yet reached; fade out only
            # that offset, so a plan that continues from the held target keeps its own speed.
            anchor = active.arrival_step - active.obs_step - 1
            anchor_position, anchor_quaternion = ((active.positions[anchor], active.quaternions[anchor]) if anchor >= 0
                                                  else (active.start_position, active.start_quaternion))
            held_position, held_quaternion = active.fade_from
            position = position + (1.0 - weight) * (np.asarray(held_position) - anchor_position)
            correction = quaternion_multiply(normalize_quaternion(held_quaternion), anchor_quaternion * [-1, -1, -1, 1])
            quaternion = quaternion_multiply(slerp([0.0, 0.0, 0.0, 1.0], correction, 1.0 - weight), quaternion)
            info.update(rtc_blend_weight=weight, rtc_blend_source="held")
    return np.asarray(position, float), normalize_quaternion(quaternion), float(active.grippers[i]), info


def delay_steps(latency_s: float, action_hz: float, margin: int) -> int:
    """Steps that pass while one inference runs, plus a safety margin."""
    return int(math.ceil(max(latency_s, 0.0) * action_hz)) + margin


def observe_due(active: Optional[RtcPlan], executed: int, execute_horizon: int, expected_delay: int,
                observe_after: int = 0) -> bool:
    """Whether to ask for the next chunk now.

    observe_after > 0: once the active chunk has run that many steps since it took over (the arm keeps
    following it while the next one is inferred). 0: so the answer lands about execute_horizon steps
    after the active chunk's observation.
    """
    if active is None: return True
    if observe_after > 0: return executed - active.arrival_step >= observe_after
    return executed - active.obs_step >= execute_horizon - expected_delay
