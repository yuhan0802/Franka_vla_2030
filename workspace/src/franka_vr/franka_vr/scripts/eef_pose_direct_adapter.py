#!/usr/bin/env python3
"""EEF pose policy adapter for the Servo-free direct joint velocity stack.

Everything above the motion interface is eef_pose_policy_adapter.EefAdapter
unchanged: policy protocol, chunk integration, workspace/tracking/jump checks,
gripper logic, settling and logging. Only /set_target_pose (MoveIt Servo) is
replaced: targets go to an in-process DirectPoseTracker, which reproduces the
Servo pose tracker used for VR collection and streams fr3_joint1..7 velocities
at 100 Hz to pi05_direct_velocity_executor (/pi05/direct_joint_velocity_cmd).
Dry-run creates no JointJog publisher.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from control_msgs.msg import JointJog
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eef_pose_policy_adapter as base  # noqa: E402
from eef_direct_kinematics import (  # noqa: E402
    JOINT_NAMES, DirectPoseTracker, forward_kinematics, joints_vector, pose_discrepancy,
)

DIRECT_CONFIRM_TOKEN = "DIRECT_EEF_VELOCITY"


class _Done:
    """Completed stand-in for the rclpy service future returned by /set_target_pose."""

    def __init__(self, success: bool, message: str) -> None:
        self.response = type("Response", (), {"success": success, "message": message})()

    def result(self):
        return self.response

    def add_done_callback(self, callback) -> None:
        callback(self)


class DirectTargetClient:
    """Drop-in for the SetTargetPose client: feeds the in-process tracker."""

    def __init__(self, node: "EefDirectAdapter") -> None:
        self.node = node

    def service_is_ready(self) -> bool:
        return self.node.executor_ready and not self.node.executor_fault

    def wait_for_service(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.service_is_ready(): return True
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.node.get_logger().error(f"direct executor not ready: {self.node.executor_message!r}")
        return False

    def call_async(self, request) -> _Done:
        if request.stop:
            self.node.stop_tracking()
            return _Done(True, "direct stop: zero joint velocity")
        if request.target_pose.header.frame_id != "world":
            return _Done(False, "target pose must be expressed in world")
        p, q = request.target_pose.pose.position, request.target_pose.pose.orientation
        self.node.set_tracking_target(np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
        return _Done(True, "direct target set")


class EefDirectAdapter(base.EefAdapter):
    def __init__(self, args: argparse.Namespace, direct: argparse.Namespace) -> None:
        self.direct = direct
        self.executor_ready = False; self.executor_fault = False; self.executor_message = "no status yet"
        self.track_target: Optional[tuple] = None; self.stopped = True; self.last_track_at: Optional[float] = None
        self.track_count = 0; self.fk_worst = (0.0, 0.0)
        super().__init__(args)
        self.tracker = DirectPoseTracker(direct.max_joint_velocity, direct.max_joint_acceleration,
                                         direct.ik_damping, direct.min_singular_value)
        self.create_subscription(DiagnosticArray, "/pi05/direct_executor/status", self.executor_status_cb, 10)
        self.jog_pub = None
        if args.execute:
            self.destroy_client(self.target_client)
            self.target_client = DirectTargetClient(self)
            self.jog_pub = self.create_publisher(JointJog, "/pi05/direct_joint_velocity_cmd", 10)
        self.create_timer(1.0 / direct.tracker_hz, self.track)
        self.get_logger().warning(
            f"DIRECT (no MoveIt Servo): tracker {direct.tracker_hz:.0f} Hz, joint envelope "
            f"{direct.max_joint_velocity} rad/s / {direct.max_joint_acceleration} rad/s^2; "
            f"JointJog publisher {'created' if self.jog_pub else 'NOT created (dry-run)'}")
        self.log("direct_start", direct=vars(direct))

    # ----------------------------------------------------------------- executor
    def executor_status_cb(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "pi05_direct_velocity_executor": continue
            self.executor_message = status.message
            self.executor_ready = status.level == DiagnosticStatus.OK and status.message == "armed"
            if status.level == DiagnosticStatus.ERROR and not self.executor_fault:
                self.executor_fault = True
                if self.args.execute: self.latch(f"direct executor fault: {status.message}")

    # ----------------------------------------------------------------- tracker
    def set_tracking_target(self, position: np.ndarray, quaternion: np.ndarray) -> None:
        if self.stopped: self.tracker.reset(); self.last_track_at = None
        self.track_target = (position, quaternion); self.stopped = False

    def stop_tracking(self) -> None:
        self.stopped = True; self.track_target = None; self.tracker.reset()
        self.publish_jog(np.zeros(7), stamped=False)

    def publish_jog(self, velocity: np.ndarray, stamped: bool = True) -> None:
        if self.jog_pub is None: return
        command = JointJog(); command.joint_names = list(JOINT_NAMES); command.velocities = velocity.tolist()
        # All-zero unstamped commands are the executor's fail-safe stop; motion must be stamped.
        if stamped: command.header.stamp = self.get_clock().now().to_msg()
        command.duration = 1.0 / self.direct.tracker_hz
        self.jog_pub.publish(command)

    def track(self) -> None:
        joints = joints_vector(self.real_joints)
        if joints is None or not self.real_joints_fresh():
            if self.track_target is not None and not self.stopped:
                self.direct_fault("real joint feedback stale while tracking")
            return
        position, quaternion, _ = forward_kinematics(joints)
        # The tracker runs on FK of /franka/joint_states; TF (what the adapter checks) must agree.
        tf_pose = self.pose()
        if tf_pose is not None:
            distance, angle = pose_discrepancy(position, quaternion, *tf_pose)
            self.fk_worst = (max(self.fk_worst[0], distance), max(self.fk_worst[1], angle))
            if distance > self.direct.max_fk_tf_position or angle > self.direct.max_fk_tf_rotation:
                self.direct_fault(f"FK disagrees with TF by {distance:.4f} m / {angle:.4f} rad"); return
        if self.fault and not self.stopped: self.stop_tracking()
        if self.jog_pub is None or self.stopped or self.track_target is None or self.fault: return
        now = time.monotonic()
        dt = 1.0 / self.direct.tracker_hz if self.last_track_at is None else now - self.last_track_at
        self.last_track_at = now
        if dt > 0.05:
            self.direct_fault(f"tracker period {dt:.3f} s; executor watchdog would trip"); return
        try: command = self.tracker.command(joints, *self.track_target, max(dt, 1e-3))
        except ValueError as error: self.direct_fault(f"tracker: {error}"); return
        self.publish_jog(command.velocity)
        self.track_count += 1
        if self.track_count % max(1, int(self.direct.tracker_hz / 10)) == 0:
            self.log("track", twist=command.twist.tolist(), joint_velocity=command.velocity.tolist(),
                     sigma_min=command.min_singular_value, speed_scaled=command.speed_scaled,
                     acceleration_scaled=command.acceleration_scaled, fk_position=position.tolist(),
                     target_position=self.track_target[0].tolist())

    def direct_fault(self, reason: str) -> None:
        self.stop_tracking()
        self.latch(reason)

    def arm(self) -> None:
        super().arm()
        if self.direct.max_fk_tf_position and self.fk_worst[0] > self.direct.max_fk_tf_position:
            raise RuntimeError(f"FK/TF mismatch {self.fk_worst} before execution")
        self.log("direct_armed", fk_tf_worst_m=self.fk_worst[0], fk_tf_worst_rad=self.fk_worst[1],
                 executor_status=self.executor_message)

    def destroy_node(self):
        if self.jog_pub is not None:
            for _ in range(4): self.publish_jog(np.zeros(7), stamped=False); time.sleep(.01)
        self.log("direct_complete", fk_tf_worst_m=self.fk_worst[0], fk_tf_worst_rad=self.fk_worst[1])
        return super().destroy_node()


def parse_direct_args() -> argparse.Namespace:
    """Direct-only options; everything else is parsed by eef_pose_policy_adapter.parse_args()."""
    parser = argparse.ArgumentParser(add_help=False)
    add = parser.add_argument
    add("--tracker-hz", type=float, default=100.0)
    add("--max-joint-velocity", type=float, default=0.5, help="tracker cap (rad/s), uniform scaling")
    add("--max-joint-acceleration", type=float, default=1.8, help="tracker cap (rad/s^2), uniform scaling")
    add("--ik-damping", type=float, default=0.02); add("--min-singular-value", type=float, default=0.02)
    add("--max-fk-tf-position", type=float, default=0.008, help="FK vs TF limit (m); TF lags during motion")
    add("--max-fk-tf-rotation", type=float, default=0.05, help="FK vs TF limit (rad)")
    add("--confirm-direct", default="")
    direct, remaining = parser.parse_known_args(sys.argv[1:])
    sys.argv = [sys.argv[0], *remaining]
    if not 50 <= direct.tracker_hz <= 200: parser.error("tracker-hz must be in [50, 200]")
    return direct


def main() -> None:
    direct = parse_direct_args(); args = base.parse_args()
    if args.execute and direct.confirm_direct != DIRECT_CONFIRM_TOKEN:
        raise SystemExit(f"direct execute requires --confirm-direct {DIRECT_CONFIRM_TOKEN}")
    rclpy.init(); node = EefDirectAdapter(args, direct)
    try:
        node.arm()
        while rclpy.ok() and not node.finished and not node.fault: rclpy.spin_once(node, timeout_sec=.1)
        for _ in range(5): rclpy.spin_once(node, timeout_sec=.02)
    finally: node.destroy_node(); rclpy.shutdown()
    if node.fault: raise SystemExit(1)


if __name__ == "__main__": main()
