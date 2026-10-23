#!/usr/bin/env python3
"""Offline tests for the Servo-free EEF path: FR3 kinematics and the Servo-equivalent tracker."""

from __future__ import annotations

import pathlib
import sys
import unittest
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "workspace/src/franka_vr/franka_vr/scripts"
sys.path.insert(0, str(SCRIPTS))

from eef_direct_kinematics import (  # noqa: E402
    SERVO_MAX_ANGULAR_SPEED, SERVO_MAX_LINEAR_SPEED, DirectPoseTracker, forward_kinematics, pose_discrepancy,
    servo_twist,
)
from eef_pose_safety import (  # noqa: E402
    ChunkTargetIntegrator, physical_step, quaternion_multiply, quaternion_to_rotvec, rotation_angle_between,
    rotvec_to_quaternion,
)

# Real /franka/joint_states at the 2026-10-22 run start, and the TF world -> fr3_link8 logged with it.
REAL_JOINTS = np.array([-0.002113885711878538, 0.19214166700839996, -0.0013124389806762338, -2.156665563583374,
                        0.0035700700245797634, 2.292140483856201, 0.7718386054039001])
REAL_TF_POSITION = (0.554552099288285, -0.0018151359399325112, 0.3048668912948322)
REAL_TF_QUATERNION = (0.9249765036848686, -0.3789649417107625, -0.02670849722811018, 0.00952348470312498)


def simulate(tracker, joints, target_position, target_quaternion, seconds, dt=0.01):
    """Ideal velocity-controlled arm: integrate the commanded joint velocities."""
    joints = joints.copy(); history = []
    for _ in range(int(round(seconds / dt))):
        command = tracker.command(joints, target_position, target_quaternion, dt)
        joints = joints + command.velocity * dt; history.append(command.velocity)
    return joints, np.array(history)


class KinematicsTests(unittest.TestCase):
    def test_fk_matches_robot_tf(self):
        # robot_state_publisher's world -> fr3_link8 for the same /franka/joint_states sample
        position, quaternion, _ = forward_kinematics(REAL_JOINTS)
        distance, angle = pose_discrepancy(position, quaternion, REAL_TF_POSITION, REAL_TF_QUATERNION)
        self.assertLess(distance, 1e-4); self.assertLess(angle, 1e-3)

    def test_jacobian_matches_finite_differences(self):
        position, quaternion, jacobian = forward_kinematics(REAL_JOINTS)
        epsilon = 1e-6
        for index in range(7):
            shifted = REAL_JOINTS.copy(); shifted[index] += epsilon
            p, q, _ = forward_kinematics(shifted)
            np.testing.assert_allclose((p - position) / epsilon, jacobian[:3, index], atol=1e-5)
            # world-frame angular velocity: rotvec(q_new * q_old^-1) / epsilon
            rotation = quaternion_to_rotvec(quaternion_multiply(q, quaternion * [-1, -1, -1, 1]))
            np.testing.assert_allclose(rotation / epsilon, jacobian[3:, index], atol=1e-5)

    def test_rejects_bad_joint_vectors(self):
        with self.assertRaises(ValueError): forward_kinematics(np.zeros(6))
        with self.assertRaises(ValueError): forward_kinematics([np.nan] * 7)


class TrackerTests(unittest.TestCase):
    def setUp(self):
        self.position, self.quaternion, _ = forward_kinematics(REAL_JOINTS)

    def test_twist_follows_servo_gain_and_caps(self):
        twist = servo_twist(self.position, self.quaternion, self.position + [0.01, 0, 0], self.quaternion)
        np.testing.assert_allclose(twist, [0.02, 0, 0, 0, 0, 0], atol=1e-12)      # 2/s * 1 cm
        far = servo_twist(self.position, self.quaternion, self.position + [0, 0.5, 0], self.quaternion)
        self.assertAlmostEqual(np.linalg.norm(far[:3]), SERVO_MAX_LINEAR_SPEED)
        turned = quaternion_multiply(rotvec_to_quaternion([0, 0, 0.1]), self.quaternion)   # world-frame 0.1 rad
        twist = servo_twist(self.position, self.quaternion, self.position, turned)
        np.testing.assert_allclose(twist[3:], [0, 0, 0.2], atol=1e-9)
        big = quaternion_multiply(rotvec_to_quaternion([1.5, 0, 0]), self.quaternion)
        self.assertAlmostEqual(np.linalg.norm(servo_twist(self.position, self.quaternion, self.position, big)[3:]),
                               SERVO_MAX_ANGULAR_SPEED)

    def test_converges_like_servo_first_order_lag(self):
        tracker = DirectPoseTracker()
        goal_position = self.position + [0.03, -0.02, -0.01]
        goal_quaternion = quaternion_multiply(self.quaternion, rotvec_to_quaternion([0.0, 0.0, 0.1]))
        initial = np.linalg.norm(goal_position - self.position)
        joints, _ = simulate(tracker, REAL_JOINTS, goal_position, goal_quaternion, 1.0)
        p, q, _ = forward_kinematics(joints)
        # e(t) = e0 * exp(-2 t): about 13.5 % left after 1 s (acceleration ramp adds a little)
        self.assertAlmostEqual(np.linalg.norm(goal_position - p) / initial, np.exp(-2.0), delta=0.03)
        joints, _ = simulate(tracker, joints, goal_position, goal_quaternion, 3.0)
        p, q, _ = forward_kinematics(joints)
        self.assertLess(np.linalg.norm(goal_position - p), 1e-4)
        self.assertLess(rotation_angle_between(goal_quaternion, q), 1e-3)

    def test_envelope_scales_uniformly(self):
        tracker = DirectPoseTracker(max_joint_velocity=0.2, max_joint_acceleration=1.0)
        _, history = simulate(tracker, REAL_JOINTS, self.position + [0.2, 0.2, -0.1], self.quaternion, 1.0)
        self.assertLessEqual(np.abs(history).max(), 0.2 + 1e-9)
        self.assertLessEqual(np.abs(np.diff(history, axis=0)).max(), 1.0 * 0.01 + 1e-9)
        # first step is acceleration-limited but points the same way as the unlimited command
        _, _, jacobian = forward_kinematics(REAL_JOINTS)
        twist = servo_twist(self.position, self.quaternion, self.position + [0.2, 0.2, -0.1], self.quaternion)
        reference = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.02 ** 2 * np.eye(6), twist)
        cosine = history[0] @ reference / np.linalg.norm(history[0]) / np.linalg.norm(reference)
        self.assertGreater(cosine, 0.999999)

    def test_policy_chunk_execution_tracks_targets(self):
        # One 15-step chunk of 4 mm +x steps, targets fed at 15 Hz like the adapter does.
        tracker = DirectPoseTracker(); integrator = ChunkTargetIntegrator("tool")
        integrator.start(self.position, self.quaternion); joints = REAL_JOINTS.copy()
        for _ in range(15):
            step = physical_step(np.array([0.004 / 0.075, 0, 0, 0, 0, 0, 1]), 0.006, 0.035)
            target_position, target_quaternion = integrator.step(step.delta_position, step.delta_rotvec)
            for _ in range(7):   # ~1/15 s at 100 Hz
                joints = joints + tracker.command(joints, target_position, target_quaternion, 0.01).velocity * 0.01
        p, q, _ = forward_kinematics(joints)
        lag = np.linalg.norm(target_position - p)
        # Servo gain 2/s at 6 cm/s settles to ~3 cm lag; the recordings show a 19-30 mm median lag.
        self.assertGreater(lag, 0.015); self.assertLess(lag, 0.035)
        self.assertLess(abs(p[1] - self.position[1]) + abs(p[2] - self.position[2]), 0.001)  # straight along +x

    def test_singularity_is_refused(self):
        tracker = DirectPoseTracker()
        stretched = np.zeros(7)   # arm straight up: joints 1, 3, 5, 7 share one axis
        with self.assertRaisesRegex(ValueError, "singularity"):
            tracker.command(stretched, self.position, self.quaternion, 0.01)

    def test_envelope_cannot_exceed_executor(self):
        with self.assertRaises(ValueError): DirectPoseTracker(max_joint_velocity=0.7)
        with self.assertRaises(ValueError): DirectPoseTracker(max_joint_acceleration=2.5)


if __name__ == "__main__":
    unittest.main()
