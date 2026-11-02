#!/usr/bin/env python3
"""EEF direct adapter with real-time chunking (RTC): asynchronous inference, blended hand-over.

The serial direct adapter (eef_pose_direct_adapter.py) stops after every chunk, waits for the
arm to settle and only then observes, so the arm pauses about a second per chunk. Here the next
observation goes to the policy while the current chunk still executes, timed so that its answer
lands about --execute-horizon steps after the previous observation; eef_rtc_plan skips the steps
that passed during inference and cross-fades the overlap. The arm only stops at the end of the
run, or when a late answer leaves no planned step (it then holds the last target).

Everything else is EefDirectAdapter unchanged: protocol and stale-server checks, the in-process
tracker, workspace/tracking/jump checks and the gripper rules. Dry-run keeps the serial loop.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eef_pose_direct_adapter as direct_base  # noqa: E402
import eef_pose_policy_adapter as base  # noqa: E402
from eef_pose_safety import advance_action_deadline, inside_workspace, rotation_angle_between  # noqa: E402
from eef_rtc_plan import build_plan, delay_steps, hand_over, observe_due, target_at  # noqa: E402


class EefRtcAdapter(direct_base.EefDirectAdapter):
    def __init__(self, args: argparse.Namespace, direct: argparse.Namespace, rtc: argparse.Namespace) -> None:
        self.rtc = rtc
        self.plans: list = []                       # [previous, active] or [active]
        self.observed_step = 0; self.observed_start: Optional[tuple] = None
        self.latency_estimate = rtc.initial_latency
        self.last_target: Optional[tuple] = None
        self.starved_since: Optional[float] = None
        self.finishing: Optional[str] = None
        super().__init__(args, direct)
        self.rtc_active = bool(args.execute) and not args.test_axis
        self.get_logger().warning(
            f"RTC chunking {'ON' if self.rtc_active else 'OFF (serial loop)'}: execute-horizon {args.execute_horizon}, "
            f"blend {rtc.blend_steps} steps, delay margin {rtc.delay_margin} steps")
        self.log("rtc_start", rtc=vars(rtc), active=self.rtc_active)

    # ----------------------------------------------------------------- loop
    def tick(self) -> None:
        if not self.rtc_active: return super().tick()
        if not self.armed or self.finished or self.fault: return
        self.checked_pose()
        if self.fault: return
        now = time.monotonic()
        if not self.gripper_verified(now): return
        if self.finishing is not None:
            # settled() latches a fault itself when the arm does not come to rest in time.
            if self.settled(now): base.EefAdapter.finish(self, self.finishing)
            return
        if self.future is not None:
            if self.future.done():
                self.receive_chunk(now)
                if self.fault or self.finishing is not None: return
            elif self.args.max_policy_latency > 0 and now - self.inference_started > self.args.max_policy_latency:
                self.latch("policy latency timeout"); return
        expected = delay_steps(self.latency_estimate, self.args.action_hz, self.rtc.delay_margin)
        if (self.future is None and not self.chunk_budget_spent()
                and observe_due(self.plans[-1] if self.plans else None, self.executed,
                                self.args.execute_horizon, expected)):
            self.request_chunk(now)
        if self.plans and now >= self.next_action_at: self.rtc_step(now)

    def gripper_verified(self, now: float) -> bool:
        """Same open/close width verification as EefAdapter.tick."""
        if self.verify_open_deadline is not None:
            width = self.gripper_width()
            if width is not None and width >= self.open_midpoint():
                self.verify_open_deadline = None; self.log("gripper_verified", open=True, measured_width=width)
            elif now > self.verify_open_deadline:
                self.latch(f"gripper reported OPEN but width stays {width} m; the fingers did not move"); return False
        if self.verify_close_deadline is not None:
            width = self.gripper_width()
            if width is not None and width < self.open_midpoint():
                self.verify_close_deadline = None; self.log("gripper_verified", open=False, measured_width=width)
            elif now > self.verify_close_deadline:
                self.latch(f"gripper reported CLOSED but width stays {width} m; the fingers did not close "
                           "or the finger feedback is wrong"); return False
        return True

    def chunk_budget_spent(self) -> bool:
        return self.args.max_chunks > 0 and self.chunks_done >= self.args.max_chunks

    def chunk_start_pose(self) -> Optional[tuple]:
        """Target commanded right now, or the measured pose when it is too far off (as --chunk-start)."""
        pose = self.checked_pose()
        if pose is None: return None
        if (self.args.chunk_start == "last_target" and self.last_target is not None
                and np.linalg.norm(self.last_target[0] - pose[0]) <= self.args.max_target_lead
                and rotation_angle_between(self.last_target[1], pose[1]) <= self.args.max_target_lead_rotation):
            return self.last_target[0].copy(), self.last_target[1].copy()
        return pose

    def request_chunk(self, now: float) -> None:
        observation = self.observation()
        if observation is None:
            if self.observation_wait_since is None: self.observation_wait_since = now
            waited = now - self.observation_wait_since
            if waited > 5.0 and now - self.observation_warned_at > 5.0:
                self.observation_warned_at = now
                self.get_logger().warning(f"no observation for {waited:.0f} s: {self.observation_blocker}")
                self.log("observation_waiting", waited_s=waited, blocker=self.observation_blocker)
            return
        start = self.chunk_start_pose()
        if start is None: return
        self.observation_wait_since = None
        self.observed_step, self.observed_start = self.executed, start
        self.inference_started = now; self.future = self.pool.submit(self.policy.infer, observation)

    def receive_chunk(self, now: float) -> None:
        future, self.future = self.future, None
        try: chunk = future.result()
        except Exception as error: self.latch(f"policy inference failed: {error}"); return
        latency = now - self.inference_started
        self.latency_estimate = latency if not self.plans else 0.5 * (self.latency_estimate + latency)
        try:
            plan = build_plan(chunk, *self.observed_start, self.integrator.rotation_frame, self.args.action_gain,
                              self.args.max_translation, self.args.max_rotation, self.observed_step, self.executed)
        except ValueError as error: self.latch(f"invalid action: {error}"); return
        self.chunks_done += 1
        self.log("chunk", chunk=np.asarray(chunk).tolist(), policy_latency=latency,
                 start_position=plan.start_position.tolist(), start_quaternion_xyzw=plan.start_quaternion.tolist(),
                 rtc_obs_step=plan.obs_step, rtc_arrival_step=plan.arrival_step,
                 rtc_delay_steps=plan.arrival_step - plan.obs_step, robot_command_sent=True)
        self.get_logger().info(
            f"chunk {self.chunks_done}: latency={latency:.3f}s, delay={plan.arrival_step - plan.obs_step} steps, "
            f"first={np.array2string(np.asarray(chunk[0]), precision=4)}")
        if self.released and self.args.finish_idle_chunks > 0:
            self.idle_chunks = self.idle_chunks + 1 if plan.translation < self.args.idle_translation else 0
            if self.idle_chunks >= self.args.finish_idle_chunks:
                self.begin_finish(f"task complete: gripper released and the policy idles "
                                  f"({self.idle_chunks} chunks under {self.args.idle_translation * 1000:.0f} mm)")
                return
        hand_over(self.plans[-1] if self.plans else None, plan, self.rtc.blend_steps, self.last_target)
        self.plans = [*self.plans[-1:], plan]

    def rtc_step(self, now: float) -> None:
        if self.args.max_actions > 0 and self.executed >= self.args.max_actions:
            self.begin_finish("budget reached"); return
        target = target_at(self.plans, self.executed)
        if target is None:
            if self.chunk_budget_spent() and self.future is None: self.begin_finish("budget reached"); return
            if self.starved_since is None:
                # The answer is later than the active chunk is long: hold the last target until it lands.
                self.starved_since = now
                self.log("rtc_starved", step=self.executed, latency_estimate_s=self.latency_estimate)
                self.get_logger().warning(f"RTC: no planned step {self.executed}; holding until the next chunk")
            return
        if self.starved_since is not None:
            self.log("rtc_resumed", step=self.executed, held_s=now - self.starved_since); self.starved_since = None
        if len(self.plans) > 1 and self.executed - self.plans[-1].arrival_step >= self.plans[-1].fade_steps:
            self.plans = self.plans[-1:]
        position, quaternion, gripper_value, info = target
        record = dict(chunk_step=info["rtc_index"] + 1, target_position=position.tolist(),
                      target_quaternion_xyzw=quaternion.tolist(), gripper_value=gripper_value, **info)
        if not inside_workspace(position, self.minimum, self.maximum):
            self.log("workspace_violation", **record)
            self.latch(f"target {np.round(position, 4).tolist()} outside workspace "
                       f"{self.minimum.tolist()}..{self.maximum.tolist()}"); return
        pose = self.checked_pose()
        if pose is None: self.latch("end-effector pose unavailable during chunk"); return
        lead = float(np.linalg.norm(position - pose[0])); lead_rotation = rotation_angle_between(quaternion, pose[1])
        record.update(measured_position=pose[0].tolist(), measured_quaternion_xyzw=pose[1].tolist(),
                      target_lead_m=lead, target_lead_rad=lead_rotation)
        if lead > self.args.max_target_lead or lead_rotation > self.args.max_target_lead_rotation:
            self.log("tracking_violation", **record)
            self.latch(f"target leads measured pose by {lead:.4f} m / {lead_rotation:.4f} rad; arm is not following"); return
        desired_open = self.gripper_request(gripper_value, pose, now, record)
        # The serial adapter's send_target bookkeeping marks chunk ends for its settle gate; RTC has none.
        if not base.EefAdapter.send_target(self, position, quaternion): return
        self.last_target = (position, quaternion); self.executed += 1
        if self.args.enable_gripper and desired_open is not None: self.request_gripper(desired_open)
        self.log("action", robot_command_sent=True, **record)
        self.next_action_at = advance_action_deadline(self.next_action_at, now, self.args.action_hz)

    def gripper_request(self, value: float, pose: tuple, now: float, record: dict) -> Optional[bool]:
        """The gripper rules of EefAdapter.execute_step: hysteresis, reversal hold, close-height guard."""
        desired_open = self.hysteresis.desired_open(value, self.commanded_gripper_open)
        hold = self.args.gripper_close_hold if desired_open and not self.commanded_gripper_open else self.args.gripper_min_dwell
        if desired_open is not None and now - self.last_gripper_request_at < hold:
            record["gripper_suppressed"] = "open" if desired_open else "close"; desired_open = None
        if desired_open is False and self.args.gripper_close_max_z > 0 and pose[0][2] > self.args.gripper_close_max_z:
            record["gripper_suppressed"] = "close_above_max_z"; desired_open = None
        record["gripper_request"] = None if desired_open is None else ("open" if desired_open else "close")
        if desired_open is not None:
            self.last_gripper_request_at = now
            if not desired_open: self.grasp_seen = True
            elif self.grasp_seen: self.released = True
        return desired_open

    def begin_finish(self, reason: str) -> None:
        """Stop planning and let the tracker bring the arm to rest on the last target before holding."""
        if self.finishing is not None: return
        self.finishing = reason; self.plans = []
        self.log("rtc_finishing", reason=reason, step=self.executed)
        if self.last_target is None: base.EefAdapter.finish(self, reason); return
        self.final_target = (self.last_target[0].copy(), self.last_target[1].copy())
        self.completion_gate = None; self.settling = True; self.settle_started = time.monotonic()


def parse_rtc_args() -> argparse.Namespace:
    """RTC-only options; the rest goes to the direct and base parsers."""
    parser = argparse.ArgumentParser(add_help=False)
    add = parser.add_argument
    add("--rtc-blend-steps", dest="blend_steps", type=int, default=4,
        help="steps over which a new chunk takes over from the previous one")
    add("--rtc-delay-margin", dest="delay_margin", type=int, default=1,
        help="extra steps added to the measured inference delay when timing the next observation")
    add("--rtc-initial-latency", dest="initial_latency", type=float, default=0.6,
        help="inference latency assumed before the first measurement (s)")
    rtc, remaining = parser.parse_known_args(sys.argv[1:])
    sys.argv = [sys.argv[0], *remaining]
    if not 1 <= rtc.blend_steps <= 8: parser.error("rtc-blend-steps must be in [1, 8]")
    if not 0 <= rtc.delay_margin <= 5: parser.error("rtc-delay-margin must be in [0, 5]")
    if not 0 < rtc.initial_latency <= 2.0: parser.error("rtc-initial-latency must be in (0, 2] s")
    return rtc


def main() -> None:
    rtc = parse_rtc_args(); direct = direct_base.parse_direct_args(); args = base.parse_args(settle_timeout_default=8.0)
    if args.settle_timeout <= 0 or args.settle_position <= 0 or args.settle_rotation <= 0:
        raise SystemExit("direct execute requires positive settle position/rotation tolerances and timeout")
    if args.execute and direct.confirm_direct != direct_base.DIRECT_CONFIRM_TOKEN:
        raise SystemExit(f"direct execute requires --confirm-direct {direct_base.DIRECT_CONFIRM_TOKEN}")
    rclpy.init(); node = EefRtcAdapter(args, direct, rtc)
    try:
        node.arm()
        while rclpy.ok() and not node.finished and not node.fault: rclpy.spin_once(node, timeout_sec=.1)
        for _ in range(5): rclpy.spin_once(node, timeout_sec=.02)
    finally: node.destroy_node(); rclpy.shutdown()
    if node.fault: raise SystemExit(1)


if __name__ == "__main__": main()
