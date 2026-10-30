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
from typing import Any, Dict, Optional

import numpy as np
import rclpy
from control_msgs.msg import JointJog
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eef_pose_policy_adapter as base  # noqa: E402
from eef_direct_completion import CompletionSample, FreshObservationBarrier, StrictCompletionGate  # noqa: E402
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
        self.real_joint_velocities: Dict[str, float] = {}
        self.feedback_sequence = 0; self.feedback_stamp_ros_ns = 0; self.feedback_received_at = 0.0
        self.image_acquisition_ros_ns: Dict[str, Optional[int]] = {}
        self.gripper_acquisition_ros_ns: Optional[int] = None
        self.completion_gate: Optional[StrictCompletionGate] = None
        self.observation_barrier: Optional[FreshObservationBarrier] = None
        self.completion_monotonic: Optional[float] = None; self.completion_ros_ns: Optional[int] = None
        self.final_target: Optional[tuple] = None; self.last_completion_sample: Optional[CompletionSample] = None
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

    # ----------------------------------------------------- timestamped feedback
    @staticmethod
    def _message_stamp_ns(message: Any) -> Optional[int]:
        stamp = message.header.stamp
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return value if value > 0 else None

    def _stamp_matches_ros_clock(self, stamp_ns: Optional[int], max_age: float) -> bool:
        if stamp_ns is None: return False
        age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
        return -self.direct.max_stamp_future_skew <= age <= max_age

    def image_cb(self, key: str, message: Any) -> None:
        previous = self.image_times.get(key)
        super().image_cb(key, message)
        if self.image_times.get(key) == previous:
            return
        stamp_ns = self._message_stamp_ns(message)
        self.image_acquisition_ros_ns[key] = (stamp_ns if self._stamp_matches_ros_clock(
            stamp_ns, self.args.max_observation_age) else None)

    def joint_cb(self, message: Any, source: str = "gripper") -> None:
        super().joint_cb(message, source)
        if any(name in message.name for name in base.FINGERS):
            stamp_ns = self._message_stamp_ns(message)
            self.gripper_acquisition_ros_ns = (stamp_ns if self._stamp_matches_ros_clock(
                stamp_ns, self.args.max_observation_age) else None)

    def real_joint_cb(self, message: Any) -> None:
        super().real_joint_cb(message)
        values = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        stamp_ns = self._message_stamp_ns(message)
        valid = (all(name in values and name in velocities for name in base.ARM_JOINTS)
                 and all(np.isfinite(values[name]) and np.isfinite(velocities[name]) for name in base.ARM_JOINTS)
                 and self._stamp_matches_ros_clock(stamp_ns, self.direct.feedback_max_age))
        # A replayed message may refresh a callback receipt time, but it is not
        # new physical evidence and must never advance the stability window.
        if valid and stamp_ns > self.feedback_stamp_ros_ns:
            self.real_joint_velocities = {name: float(velocities[name]) for name in base.ARM_JOINTS}
            self.feedback_stamp_ros_ns = stamp_ns
            self.feedback_received_at = time.monotonic()
            self.feedback_sequence += 1

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

    def send_target(self, position: np.ndarray, quaternion: np.ndarray) -> bool:
        sent = super().send_target(position, quaternion)
        if sent and self.armed and not self.chunk:
            self.final_target = (position.copy(), quaternion.copy())
            self.completion_gate = None; self.observation_barrier = None
            self.log("chunk_final_target_sent", target_position=position.tolist(),
                     target_quaternion_xyzw=quaternion.tolist(), physical_completion=False,
                     gripper_gate_included=False, pending_gripper=self.pending_gripper_open)
        return sent

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
        feedback_age = time.monotonic() - self.feedback_received_at
        if (self.track_target is not None and not self.stopped
                and feedback_age > self.direct.feedback_max_age):
            if self.settling and self.final_target is not None:
                self.settled(time.monotonic())
            else:
                self.direct_fault(f"real joint feedback stale while tracking ({feedback_age:.3f}s)")
            return
        joints = joints_vector(self.real_joints)
        if joints is None:
            if self.track_target is not None and not self.stopped:
                self.direct_fault("real joint feedback unavailable while tracking")
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

    def _completion_sample(self) -> Optional[CompletionSample]:
        joints = joints_vector(self.real_joints)
        velocities = joints_vector(self.real_joint_velocities)
        if joints is None or velocities is None or self.final_target is None: return None
        position, quaternion, jacobian = forward_kinematics(joints)
        twist = jacobian @ velocities
        distance, angle = pose_discrepancy(position, quaternion, *self.final_target)
        valid = self._stamp_matches_ros_clock(self.feedback_stamp_ros_ns, self.direct.feedback_max_age)
        sample = CompletionSample(
            self.feedback_sequence, self.feedback_received_at, distance, angle,
            float(np.linalg.norm(twist[:3])), float(np.linalg.norm(twist[3:])), valid)
        self.last_completion_sample = sample
        return sample

    def settled(self, now: float) -> bool:
        """Direct-only strict block-end gate; timeout latches a stop instead of releasing."""
        if self.final_target is None:
            self.direct_fault("strict completion has no final chunk target"); return False
        if self.completion_gate is None:
            self.completion_gate = StrictCompletionGate(
                self.settle_started, self.args.settle_position, self.args.settle_rotation,
                self.direct.settle_linear_speed, self.direct.settle_angular_speed,
                self.direct.settle_stable_duration, self.direct.feedback_max_age, self.args.settle_timeout)
        sample = self._completion_sample()
        decision = self.completion_gate.evaluate(now, sample)
        if decision.complete:
            self.settling = False
            self.completion_monotonic = sample.received_at
            self.completion_ros_ns = self.feedback_stamp_ros_ns
            self.need_fresh_after = max(self.need_fresh_after, self.completion_monotonic)
            self.observation_barrier = FreshObservationBarrier(
                self.completion_ros_ns, self.completion_monotonic, self.args.max_observation_age)
            position, quaternion, _ = forward_kinematics(joints_vector(self.real_joints))
            self.log("chunk_physically_complete", reason="reached",
                     target_position=self.final_target[0].tolist(),
                     target_quaternion_xyzw=self.final_target[1].tolist(),
                     measured_position=position.tolist(), measured_quaternion_xyzw=quaternion.tolist(),
                     position_error_m=sample.position_error, rotation_error_rad=sample.rotation_error,
                     linear_speed_m_s=sample.linear_speed, angular_speed_rad_s=sample.angular_speed,
                     waited_s=decision.waited_for, stable_s=decision.stable_for,
                     completion_ros_s=self.completion_ros_ns * 1e-9,
                     gripper_gate_included=False, pending_gripper=self.pending_gripper_open)
            self.get_logger().info(
                f"chunk physically reached: error {sample.position_error:.4f} m / "
                f"{sample.rotation_error:.4f} rad, speed {sample.linear_speed:.4f} m/s / "
                f"{sample.angular_speed:.4f} rad/s, stable {decision.stable_for:.3f}s")
            return True
        if decision.must_stop:
            fields = dict(reason=decision.reason, waited_s=decision.waited_for,
                          stable_s=decision.stable_for, physical_completion=False,
                          target_position=self.final_target[0].tolist(),
                          target_quaternion_xyzw=self.final_target[1].tolist())
            if sample is not None:
                fields.update(position_error_m=sample.position_error, rotation_error_rad=sample.rotation_error,
                              linear_speed_m_s=sample.linear_speed, angular_speed_rad_s=sample.angular_speed)
                joints = joints_vector(self.real_joints)
                if joints is not None:
                    position, quaternion, _ = forward_kinematics(joints)
                    fields.update(measured_position=position.tolist(), measured_quaternion_xyzw=quaternion.tolist())
            self.log("chunk_completion_failed", **fields)
            self.direct_fault(f"chunk completion {decision.reason} after {decision.waited_for:.2f}s")
        return False

    def observation(self) -> Optional[Dict[str, Any]]:
        if self.observation_barrier is None: return super().observation()
        # Refresh TF first. Its ROS stamp is checked by base.pose() against the
        # node ROS clock; never compare it directly with monotonic callback time.
        pose = self.checked_pose()
        now = time.monotonic(); now_ros_ns = self.get_clock().now().nanoseconds
        if not self._stamp_matches_ros_clock(self.last_pose_stamp_ros_ns or None, self.args.max_tf_age):
            pose = None
        acquisition = {"exterior_image": self.image_acquisition_ros_ns.get("exterior"),
                       "wrist_image": self.image_acquisition_ros_ns.get("wrist"),
                       "arm_joint_state": self.feedback_stamp_ros_ns or None,
                       "eef_tf": self.last_pose_stamp_ros_ns or None}
        received = {"exterior_image": self.image_times.get("exterior", 0.0),
                    "wrist_image": self.image_times.get("wrist", 0.0),
                    "arm_joint_state": self.feedback_received_at,
                    # Map the validated TF ROS age into the local monotonic domain.
                    "eef_tf": now - max(0.0, (now_ros_ns - self.last_pose_stamp_ros_ns) * 1e-9)}
        if self.args.gripper_state_source in ("measured", "binary"):
            acquisition["gripper_state"] = self.gripper_acquisition_ros_ns
            received["gripper_state"] = self.gripper_time
        ok, reason = self.observation_barrier.check(now, acquisition, received)
        if not ok or pose is None:
            self.observation_blocker = reason if not ok else "fresh post-completion EEF pose unavailable"
            return None
        observation = super().observation()
        if observation is not None:
            ages = {name: (now_ros_ns - stamp) * 1e-9 for name, stamp in acquisition.items() if stamp is not None}
            self.log("post_completion_observation", completion_ros_s=self.completion_ros_ns * 1e-9,
                     acquisition_ros_s={name: stamp * 1e-9 for name, stamp in acquisition.items()},
                     acquisition_age_s=ages, camera_skew_s=abs(
                         self.image_times["exterior"] - self.image_times["wrist"]))
            self.observation_barrier = None
        return observation

    def tick(self) -> None:
        super().tick()
        if (self.fault or self.finished or self.observation_barrier is None
                or self.future is not None or self.chunk or self.settling or self.args.test_axis): return
        budget_reached = ((self.args.max_actions > 0 and self.executed >= self.args.max_actions)
                          or (self.args.max_chunks > 0 and self.chunks_done >= self.args.max_chunks))
        if not budget_reached and self.observation_barrier.timed_out(
                time.monotonic(), self.direct.observation_timeout):
            self.log("post_completion_observation_failed", reason="timeout",
                     waited_s=time.monotonic() - self.completion_monotonic,
                     blocker=self.observation_blocker)
            self.direct_fault(f"fresh post-completion observation timeout: {self.observation_blocker}")

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
    add("--settle-linear-speed", type=float, default=0.01, help="strict gate EEF linear speed (m/s)")
    add("--settle-angular-speed", type=float, default=0.05, help="strict gate EEF angular speed (rad/s)")
    add("--settle-stable-duration", type=float, default=0.30, help="continuous qualifying feedback (s)")
    add("--feedback-max-age", type=float, default=0.20, help="strict gate real joint feedback age (s)")
    add("--observation-timeout", type=float, default=5.0, help="fresh post-completion observation wait limit (s)")
    add("--max-stamp-future-skew", type=float, default=0.05, help="allowed ROS stamp future skew (s)")
    add("--confirm-direct", default="")
    direct, remaining = parser.parse_known_args(sys.argv[1:])
    sys.argv = [sys.argv[0], *remaining]
    if not 50 <= direct.tracker_hz <= 200: parser.error("tracker-hz must be in [50, 200]")
    strict = (direct.settle_linear_speed, direct.settle_angular_speed, direct.settle_stable_duration,
              direct.feedback_max_age, direct.observation_timeout)
    if (any(not np.isfinite(value) or value <= 0 for value in strict)
            or not np.isfinite(direct.max_stamp_future_skew) or direct.max_stamp_future_skew < 0):
        parser.error("direct settle/feedback/observation parameters must be finite and positive")
    return direct


def main() -> None:
    direct = parse_direct_args(); args = base.parse_args(settle_timeout_default=8.0)
    if args.settle_timeout <= 0 or args.settle_position <= 0 or args.settle_rotation <= 0:
        raise SystemExit("direct execute requires positive settle position/rotation tolerances and timeout")
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
