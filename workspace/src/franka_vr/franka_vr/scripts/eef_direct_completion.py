#!/usr/bin/env python3
"""ROS-independent completion and post-completion observation gates for EEF direct."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional


@dataclass(frozen=True)
class CompletionSample:
    sequence: int
    received_at: float
    position_error: float
    rotation_error: float
    linear_speed: float
    angular_speed: float
    valid: bool = True


@dataclass(frozen=True)
class CompletionDecision:
    complete: bool
    reason: str
    stable_for: float
    waited_for: float
    consumed: bool

    @property
    def must_stop(self) -> bool:
        return self.reason in ("timeout", "feedback_stale")


class StrictCompletionGate:
    """Require consecutive qualifying *new* feedback; timeout is always a fault."""

    def __init__(self, started_at: float, position_tolerance: float, rotation_tolerance: float,
                 linear_speed_tolerance: float, angular_speed_tolerance: float,
                 stable_duration: float, feedback_max_age: float, timeout: float) -> None:
        values = (position_tolerance, rotation_tolerance, linear_speed_tolerance,
                  angular_speed_tolerance, stable_duration, feedback_max_age, timeout)
        if any(value <= 0 for value in values):
            raise ValueError("strict completion parameters must be positive")
        self.started_at = started_at
        self.position_tolerance = position_tolerance
        self.rotation_tolerance = rotation_tolerance
        self.linear_speed_tolerance = linear_speed_tolerance
        self.angular_speed_tolerance = angular_speed_tolerance
        self.stable_duration = stable_duration
        self.feedback_max_age = feedback_max_age
        self.timeout = timeout
        self.last_sequence: Optional[int] = None
        self.stable_since: Optional[float] = None

    def evaluate(self, now: float, sample: Optional[CompletionSample]) -> CompletionDecision:
        waited = max(0.0, now - self.started_at)
        fresh = sample is not None and sample.valid and 0.0 <= now - sample.received_at <= self.feedback_max_age
        new = fresh and sample.sequence != self.last_sequence
        consumed = bool(new)
        if new:
            self.last_sequence = sample.sequence
            qualified = (sample.position_error <= self.position_tolerance
                         and sample.rotation_error <= self.rotation_tolerance
                         and sample.linear_speed <= self.linear_speed_tolerance
                         and sample.angular_speed <= self.angular_speed_tolerance)
            if qualified:
                if self.stable_since is None:
                    self.stable_since = sample.received_at
            else:
                self.stable_since = None
        elif not fresh:
            # A gap in valid feedback breaks continuity. Re-evaluating the same
            # still-fresh sample neither advances nor resets the feedback clock.
            self.stable_since = None
        stable_for = 0.0 if self.stable_since is None else max(0.0, sample.received_at - self.stable_since)
        if new and stable_for >= self.stable_duration:
            return CompletionDecision(True, "reached", stable_for, waited, consumed)
        if not fresh:
            return CompletionDecision(False, "feedback_stale", stable_for, waited, consumed)
        if waited >= self.timeout:
            return CompletionDecision(False, "timeout", stable_for, waited, consumed)
        return CompletionDecision(False, "waiting", stable_for, waited, consumed)


class FreshObservationBarrier:
    """Checks acquisition timestamps in one declared ROS clock domain."""

    def __init__(self, completed_ros_ns: int, completed_monotonic: float, max_age: float) -> None:
        if completed_ros_ns <= 0 or max_age <= 0:
            raise ValueError("completion timestamp and max_age must be positive")
        self.completed_ros_ns = completed_ros_ns
        self.completed_monotonic = completed_monotonic
        self.max_age = max_age

    def timed_out(self, now: float, timeout: float) -> bool:
        if timeout <= 0:
            raise ValueError("observation timeout must be positive")
        return now - self.completed_monotonic >= timeout

    def check(self, now: float, acquisition_ros_ns: Mapping[str, Optional[int]],
              received_monotonic: Mapping[str, float]) -> tuple[bool, str]:
        for name, stamp in acquisition_ros_ns.items():
            if stamp is None or stamp <= self.completed_ros_ns:
                return False, f"{name} was not acquired after physical completion"
            received = received_monotonic.get(name, 0.0)
            if received <= self.completed_monotonic:
                return False, f"{name} was not received after physical completion"
            if now - received > self.max_age or received > now:
                return False, f"{name} is stale or has an invalid receive time"
        return True, "fresh"
