#!/usr/bin/env python3
"""Offline acceptance tests for the end-effector pose policy protocol."""

from __future__ import annotations

import pathlib
import sys
import unittest
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "workspace/src/franka_vr/franka_vr/scripts"
sys.path.insert(0, str(SCRIPTS))

from eef_pose_safety import (  # noqa: E402
    ACTION_SHAPE, ANGULAR_SCALE_RAD, LINEAR_SCALE_M, ChunkTargetIntegrator, GripperHysteresis,
    axis_test_chunk, build_state, gripper_open_from_width, inside_workspace, physical_step,
    quaternion_multiply, quaternion_to_rotvec, rotation_angle_between, rotvec_to_quaternion,
    validate_action_chunk,
)

try:
    from scipy.spatial.transform import Rotation
except ImportError:  # the franka-vr image has no scipy; skip cross-checks there
    Rotation = None

# FR3 flange pointing straight down: 180 deg about world x.
DOWN = np.array([1.0, 0.0, 0.0, 0.0])


def recorder_delta(previous, current):
    """Verbatim copy of lerobot_episode_recorder.target_delta rotation math."""
    px, py, pz, pw = previous
    cx, cy, cz, cw = current
    x = cw * -px + cx * pw - cy * -pz + cz * -py
    y = cw * -py + cx * -pz + cy * pw - cz * -px
    z = cw * -pz - cx * -py + cy * -px + cz * pw
    w = cw * pw + cx * px + cy * py + cz * pz
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    norm = float(np.linalg.norm([x, y, z]))
    return np.zeros(3) if norm < 1e-8 else np.array([x, y, z]) * (2.0 * np.arctan2(norm, w) / norm)


class EefPoseTests(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(0)

    def random_quaternions(self, count):
        q = self.rng.normal(size=(count, 4))
        return q / np.linalg.norm(q, axis=1, keepdims=True)

    def test_rotvec_roundtrip(self):
        for q in self.random_quaternions(500):
            back = rotvec_to_quaternion(quaternion_to_rotvec(q))
            self.assertLess(rotation_angle_between(q, back), 1e-9)
        np.testing.assert_allclose(quaternion_to_rotvec([0, 0, 0, 1]), 0)
        np.testing.assert_allclose(rotvec_to_quaternion([0, 0, 0]), [0, 0, 0, 1])

    @unittest.skipIf(Rotation is None, "scipy unavailable")
    def test_matches_scipy_conventions(self):
        for q in self.random_quaternions(500):
            np.testing.assert_allclose(quaternion_to_rotvec(q), Rotation.from_quat(q).as_rotvec(), atol=1e-9)
        for a, b in zip(self.random_quaternions(200), self.random_quaternions(200)):
            expected = (Rotation.from_quat(a) * Rotation.from_quat(b)).as_quat()
            self.assertLess(rotation_angle_between(quaternion_multiply(a, b), expected), 1e-9)
        for r in self.rng.normal(scale=1.0, size=(200, 3)):
            self.assertLess(rotation_angle_between(rotvec_to_quaternion(r), Rotation.from_rotvec(r).as_quat()), 1e-9)

    def test_state_layout(self):
        state = build_state([0.5, 0.0, 0.3], [0, 0, np.sin(0.25), np.cos(0.25)], 1.0)
        self.assertEqual(state.shape, (7,)); self.assertEqual(state.dtype, np.float32)
        np.testing.assert_allclose(state, [0.5, 0, 0.3, 0, 0, 0.5, 1.0], atol=1e-6)
        with self.assertRaises(ValueError): build_state([0, 0, 0], [0, 0, 0, 0], 1.0)
        with self.assertRaises(ValueError): build_state([0, 0, 0], [0, 0, 0, 1], 1.2)

    def test_top_down_pose_sits_on_rotvec_flip(self):
        # Two orientations 0.02 rad apart on either side of the pi boundary
        # produce rotvecs pointing in opposite directions.
        a = quaternion_multiply(rotvec_to_quaternion([0.01, 0, 0]), DOWN)
        b = quaternion_multiply(rotvec_to_quaternion([-0.01, 0, 0]), DOWN)
        self.assertLess(rotation_angle_between(a, b), 0.021)
        self.assertGreater(np.linalg.norm(quaternion_to_rotvec(a) - quaternion_to_rotvec(b)), 1.0)

    def test_scale_and_norm_limit_preserve_direction(self):
        step = physical_step([1, 0, 0, 0, 0, 1, 0.7], 1.0, 1.0)
        np.testing.assert_allclose(step.delta_position, [LINEAR_SCALE_M, 0, 0])
        np.testing.assert_allclose(step.delta_rotvec, [0, 0, ANGULAR_SCALE_RAD])
        self.assertAlmostEqual(step.gripper_open, 0.7)
        step = physical_step([1, 1, 0, 0, 1, 1, 1.4], 0.006, 0.035)
        self.assertTrue(step.translation_clipped and step.rotation_clipped)
        self.assertAlmostEqual(np.linalg.norm(step.delta_position), 0.006)
        self.assertAlmostEqual(np.linalg.norm(step.delta_rotvec), 0.035)
        np.testing.assert_allclose(step.delta_position[0], step.delta_position[1])
        self.assertEqual(step.gripper_open, 1.0)

    def test_translation_is_world_frame_for_any_orientation(self):
        for q in [DOWN, *self.random_quaternions(20)]:
            integrator = ChunkTargetIntegrator(); integrator.start([0.5, 0, 0.3], q)
            position, quaternion = integrator.step([0.003, 0, 0], [0, 0, 0])
            np.testing.assert_allclose(position, [0.503, 0, 0.3])
            self.assertLess(rotation_angle_between(quaternion, q), 1e-12)

    def test_recorder_rotation_label_is_tool_frame(self):
        # The recorder computes inv(q_prev) * q_cur: only right multiply
        # (tool frame) reproduces its label; left multiply does not.
        for q in self.random_quaternions(100):
            delta = self.rng.normal(scale=0.02, size=3)
            tool = ChunkTargetIntegrator("tool"); tool.start([0, 0, 0], q)
            _, target = tool.step([0, 0, 0], delta)
            np.testing.assert_allclose(recorder_delta(q, target), delta, atol=1e-9)
        world = ChunkTargetIntegrator("world"); world.start([0, 0, 0], DOWN)
        _, target = world.step([0, 0, 0], [0, 0, 0.02])
        np.testing.assert_allclose(recorder_delta(DOWN, target), [0, 0, -0.02], atol=1e-9)

    @unittest.skipIf(Rotation is None, "scipy unavailable")
    def test_world_and_tool_frame_execution(self):
        for q in self.random_quaternions(50):
            delta = self.rng.normal(scale=0.05, size=3)
            for frame, expected in (("world", Rotation.from_rotvec(delta) * Rotation.from_quat(q)),
                                    ("tool", Rotation.from_quat(q) * Rotation.from_rotvec(delta))):
                integrator = ChunkTargetIntegrator(frame); integrator.start([0, 0, 0], q)
                _, target = integrator.step([0, 0, 0], delta)
                self.assertLess(rotation_angle_between(target, expected.as_quat()), 1e-9)
        with self.assertRaises(ValueError): ChunkTargetIntegrator("base")

    def test_two_30hz_deltas_compose_into_one_15hz_step(self):
        q = DOWN
        r0, r1 = self.rng.normal(scale=0.03, size=(2, 3))
        sequential = ChunkTargetIntegrator(); sequential.start([0, 0, 0], q)
        sequential.step([0, 0, 0], r0); _, expected = sequential.step([0, 0, 0], r1)
        combined = quaternion_to_rotvec(quaternion_multiply(rotvec_to_quaternion(r1), rotvec_to_quaternion(r0)))
        once = ChunkTargetIntegrator(); once.start([0, 0, 0], q)
        _, actual = once.step([0, 0, 0], combined)
        self.assertLess(rotation_angle_between(actual, expected), 1e-12)

    def test_chunk_accumulates_from_single_start(self):
        integrator = ChunkTargetIntegrator(); integrator.start([0.5, 0, 0.3], DOWN)
        for _ in range(15):
            position, _ = integrator.step([0.002, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(position, [0.53, 0, 0.3])

    def test_chunk_validation(self):
        self.assertEqual(validate_action_chunk(np.zeros(ACTION_SHAPE)).shape, ACTION_SHAPE)
        with self.assertRaises(ValueError): validate_action_chunk(np.zeros((16, 8)))
        bad = np.zeros(ACTION_SHAPE); bad[3, 2] = np.nan
        with self.assertRaises(ValueError): validate_action_chunk(bad)

    def test_gripper_mapping_and_hysteresis(self):
        self.assertEqual(gripper_open_from_width(0.01), 0.0)
        self.assertEqual(gripper_open_from_width(0.08), 1.0)
        self.assertAlmostEqual(gripper_open_from_width(0.045), 0.5)
        self.assertEqual(gripper_open_from_width(0.0), 0.0)
        hysteresis = GripperHysteresis(0.6, 0.4)
        self.assertIs(hysteresis.desired_open(0.3, True), False)
        self.assertIsNone(hysteresis.desired_open(0.3, False))
        self.assertIsNone(hysteresis.desired_open(0.5, False))
        self.assertIsNone(hysteresis.desired_open(0.5, True))
        self.assertIs(hysteresis.desired_open(0.65, False), True)

    def test_workspace_and_axis_chunks(self):
        self.assertTrue(inside_workspace([0.5, 0, 0.3], [0.38, -0.2, 0.18], [0.75, 0.2, 0.78]))
        self.assertFalse(inside_workspace([0.5, 0, 0.1], [0.38, -0.2, 0.18], [0.75, 0.2, 0.78]))
        chunk = axis_test_chunk("-rz", 0.05, 5)
        self.assertEqual(chunk.shape, (5, 7)); np.testing.assert_allclose(chunk[:, 5], -0.05)
        with self.assertRaises(ValueError): axis_test_chunk("+q", 0.05, 5)


if __name__ == "__main__":
    unittest.main()
