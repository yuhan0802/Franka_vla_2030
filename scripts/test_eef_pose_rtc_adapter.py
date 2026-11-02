#!/usr/bin/env python3
"""Offline tests for real-time chunking: eef_rtc_plan math and the eef_pose_rtc_adapter loop.

The closed loop reuses the simulated FR3 of test_eef_pose_direct_adapter.py; the policy answers
after a simulated latency, so chunks arrive while the previous one is still executing.
"""

from __future__ import annotations

import pathlib
import sys
import tempfile
import types
import unittest
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "workspace/src/franka_vr/franka_vr/scripts"))

from eef_rtc_plan import build_plan, delay_steps, hand_over, observe_due, slerp, target_at  # noqa: E402

DOWN = np.array([1.0, 0.0, 0.0, 0.0])


def chunk(dx=0.0, dy=0.0, gripper=1.0, steps=15):
    actions = np.zeros((steps, 7)); actions[:, 0] = dx; actions[:, 1] = dy; actions[:, 6] = gripper
    return actions


def plan(actions, start=(0.5, 0.0, 0.3), obs_step=0, arrival_step=0):
    return build_plan(actions, np.array(start), DOWN, "world", [1.0] * 6, 0.0, 0.035, obs_step, arrival_step)


class PlanTests(unittest.TestCase):
    def test_waypoints_match_serial_integration(self):
        p = plan(chunk(dx=0.04))                       # 3 mm per step
        np.testing.assert_allclose(p.positions[:, 0] - 0.5, 0.003 * np.arange(1, 16), atol=1e-12)
        self.assertAlmostEqual(p.translation, 0.045)

    def test_steps_that_passed_during_inference_are_skipped(self):
        p = plan(chunk(dx=0.04), obs_step=10, arrival_step=16)
        position, _, _, info = target_at([p], 16)
        self.assertEqual(info["rtc_index"], 6)
        self.assertAlmostEqual(position[0], 0.5 + 0.003 * 7)
        self.assertIsNone(target_at([p], 25))         # index 15: beyond the chunk

    def test_overlap_cross_fades_into_the_new_chunk(self):
        old = plan(chunk(dx=0.04))
        new = plan(chunk(), start=old.positions[5], obs_step=6, arrival_step=10)   # new chunk: stand still
        hand_over(old, new, 4, held=None)
        weights = [target_at([old, new], step)[3]["rtc_blend_weight"] for step in range(10, 15)]
        np.testing.assert_allclose(weights, [0.2, 0.4, 0.6, 0.8, 1.0])
        path = np.array([target_at([old, new], step)[0] for step in range(10, 15)])
        # The old plan keeps advancing 3 mm/step, the new one holds: every step moves less than without blending.
        jumps = np.linalg.norm(np.diff(np.vstack([old.positions[9], path]), axis=0), axis=1)
        self.assertLess(jumps.max(), 0.012)
        np.testing.assert_allclose(path[-1], new.positions[4])

    def test_fade_never_outlasts_the_previous_plan(self):
        old = plan(chunk(dx=0.04))
        new = plan(chunk(dx=0.04), start=old.positions[11], obs_step=12, arrival_step=13)
        hand_over(old, new, 4, held=None)
        self.assertEqual(new.fade_steps, 2)            # old covers steps 13 and 14 only
        self.assertEqual(target_at([old, new], 14)[3]["rtc_blend_source"], "previous")

    def test_first_plan_does_not_fade(self):
        first = plan(chunk(dx=0.04))
        hand_over(None, first, 4, held=(np.zeros(3), DOWN))
        self.assertEqual(first.fade_steps, 0)

    def test_new_chunk_fades_in_from_a_held_target(self):
        old = plan(chunk(dx=0.04))
        new = plan(chunk(dx=0.04), start=old.positions[14], obs_step=10, arrival_step=20)
        held = (old.positions[14], old.quaternions[14])
        hand_over(old, new, 4, held)
        position, _, _, info = target_at([old, new], 20)
        self.assertEqual(info["rtc_blend_source"], "held")
        # the new plan put the arm at its waypoint 9 before arriving; 80% of the gap to the held target remains
        np.testing.assert_allclose(position, new.positions[10] + 0.8 * (held[0] - new.positions[9]))

    def test_plan_continuing_from_the_held_target_keeps_its_speed(self):
        old = plan(chunk(dx=0.04))
        new = plan(chunk(dx=0.04), start=old.positions[11], obs_step=12, arrival_step=15)   # held after step 14
        hand_over(old, new, 4, (old.positions[14], old.quaternions[14]))
        path = np.array([target_at([old, new], step)[0] for step in range(15, 20)])
        steps = np.linalg.norm(np.diff(np.vstack([old.positions[14], path]), axis=0), axis=1)
        np.testing.assert_allclose(steps, 0.003, atol=1e-9)

    def test_slerp_takes_the_short_way(self):
        q = slerp(DOWN, -DOWN, 0.5)
        self.assertAlmostEqual(abs(float(np.dot(q, DOWN))), 1.0)

    def test_observation_timing(self):
        self.assertEqual(delay_steps(0.41, 15.0, 1), 8)
        p = plan(chunk(), obs_step=20)
        self.assertFalse(observe_due(p, 21, 10, 8))
        self.assertTrue(observe_due(p, 22, 10, 8))
        self.assertTrue(observe_due(None, 0, 10, 8))
        late = plan(chunk(), obs_step=20, arrival_step=25)
        self.assertFalse(observe_due(late, 30, 10, 8, observe_after=6))
        self.assertTrue(observe_due(late, 31, 10, 8, observe_after=6))


class LatentFuture:
    """Policy future that completes after a simulated latency."""

    def __init__(self, value, ready_ns):
        self.value, self.ready_ns = value, ready_ns

    def done(self):
        from test_eef_pose_direct_adapter import Sim
        return Sim.now_ns >= self.ready_ns

    def result(self): return self.value


class LatentPool:
    latency_s = 0.4

    def submit(self, function, *args):
        from test_eef_pose_direct_adapter import Sim
        return LatentFuture(function(*args), Sim.now_ns + int(LatentPool.latency_s * 1e9))

    def shutdown(self, **kwargs): pass


class RtcAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_eef_pose_direct_adapter as sim
        sim.install_direct_fakes()
        harness = sim.harness

        def publish(node, topic, names, positions):
            # Current direct adapters read joint velocities and ROS header stamps.
            stamp = types.SimpleNamespace(sec=sim.Sim.now_ns // 10**9, nanosec=sim.Sim.now_ns % 10**9)
            velocity = list(sim.Sim.velocity) if topic == "/franka/joint_states" else [0.0] * len(names)
            message = harness._Namespace(name=list(names), position=list(positions), velocity=velocity,
                                         header=harness._Namespace(stamp=stamp))
            for callback in node._subscriptions.get(topic, []): callback(message)
        harness.FakeNode.publish = publish
        import eef_pose_policy_adapter, eef_pose_direct_adapter, eef_pose_rtc_adapter
        clock = types.SimpleNamespace(monotonic=lambda: sim.Sim.now_ns * 1e-9, sleep=lambda s: None)
        eef_pose_policy_adapter.time = eef_pose_direct_adapter.time = eef_pose_rtc_adapter.time = clock
        eef_pose_policy_adapter.EefPolicyClient = harness.FakePolicy
        cls.sim, cls.harness = sim, harness
        cls.base, cls.direct, cls.rtc = eef_pose_policy_adapter, eef_pose_direct_adapter, eef_pose_rtc_adapter

    def setUp(self):
        self.sim.Sim.reset(); self.harness.FakePolicy.chunks = []; LatentPool.latency_s = 0.4

    def node(self, *extra, execute=True):
        bounds = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        bounds.write("workspace_bounds_world:\n  min: [0.2, -0.4, 0.1]\n  max: [0.8, 0.4, 0.9]\n"); bounds.close()
        sys.argv = ["adapter", "--bounds-config", bounds.name, "--gripper-state-source", "binary",
                    "--chunk-start", "last_target", "--execute-horizon", "10", "--max-chunks", "4",
                    "--max-actions", "0", "--startup-timeout", "5", "--max-translation", "0", *extra]
        if execute:
            sys.argv += ["--execute", "--confirm-real-robot", "EEF_POSE_TARGET", "--confirm-direct", "DIRECT_EEF_VELOCITY"]
        rtc = self.rtc.parse_rtc_args(); direct = self.direct.parse_direct_args(); args = self.base.parse_args()
        node = self.rtc.EefRtcAdapter(args, direct, rtc)
        node.pool = LatentPool()
        node.events = []
        log = node.log
        node.log = lambda event, **fields: (node.events.append((event, fields)), log(event, **fields))
        return node

    def run_node(self, node, spins=3000):
        node.arm()
        for _ in range(spins):
            if node.finished or node.fault: break
            self.sim.sim_spin(node)
        for _ in range(5): self.sim.sim_spin(node)
        node.destroy_node()

    def actions(self, node):
        return [fields for event, fields in node.events if event == "action"]

    def test_chunks_hand_over_without_stopping(self):
        self.harness.FakePolicy.chunks = [chunk(dx=0.02)] * 4         # 1.5 mm per step
        node = self.node()
        self.run_node(node)
        self.assertFalse(node.fault, node.events[-3:])
        self.assertTrue(node.finished)
        self.assertEqual(node.chunks_done, 4)
        self.assertFalse(any(event == "rtc_starved" for event, _ in node.events))
        delays = [fields["rtc_delay_steps"] for event, fields in node.events if event == "chunk"]
        self.assertTrue(all(5 <= d <= 8 for d in delays[1:]), delays)
        targets = np.array([a["target_position"] for a in self.actions(node)])
        steps = np.linalg.norm(np.diff(targets, axis=0), axis=1)
        # Constant 1.5 mm/step command: no step goes backwards or jumps across a hand-over.
        self.assertLess(steps.max(), 0.0016); self.assertGreater(steps.min(), 0.0014)
        self.assertTrue(any(a["rtc_blend_weight"] < 1.0 for a in self.actions(node)))

    def test_observe_after_runs_each_chunk_before_asking_again(self):
        self.harness.FakePolicy.chunks = [chunk(dx=0.02)] * 4
        node = self.node("--rtc-observe-after", "6")
        self.run_node(node)
        self.assertFalse(node.fault, node.events[-3:])
        chunks = [fields for event, fields in node.events if event == "chunk"]
        for previous, current in zip(chunks, chunks[1:]):
            self.assertEqual(current["rtc_obs_step"] - previous["rtc_arrival_step"], 6)
        targets = np.array([a["target_position"] for a in self.actions(node)])
        steps = np.linalg.norm(np.diff(targets, axis=0), axis=1)
        self.assertLess(steps.max(), 0.0016)

    def test_hand_over_to_a_stopping_chunk_is_smooth(self):
        self.harness.FakePolicy.chunks = [chunk(dx=0.04), chunk(), chunk(), chunk()]
        node = self.node()
        self.run_node(node)
        self.assertFalse(node.fault, node.events[-3:])
        targets = np.array([a["target_position"] for a in self.actions(node)])
        steps = np.linalg.norm(np.diff(targets, axis=0), axis=1)
        # The second chunk was planned d steps earlier and holds still, so without blending
        # the target would jump back by ~d x 3 mm in one step.
        self.assertLess(steps.max(), 0.0075)

    def test_late_answer_holds_then_resumes(self):
        LatentPool.latency_s = 1.4                                      # 21 steps > one chunk
        self.harness.FakePolicy.chunks = [chunk(dx=0.02)] * 3
        node = self.node("--max-chunks", "3")
        self.run_node(node)
        self.assertFalse(node.fault, node.events[-3:])
        names = [event for event, _ in node.events]
        self.assertIn("rtc_starved", names); self.assertIn("rtc_resumed", names)

    def test_dry_run_keeps_the_serial_loop(self):
        node = self.node(execute=False)
        self.assertFalse(node.rtc_active)
        self.run_node(node)
        self.assertFalse(node.fault)
        self.assertFalse(any(event == "rtc_finishing" for event, _ in node.events))


if __name__ == "__main__":
    unittest.main()
