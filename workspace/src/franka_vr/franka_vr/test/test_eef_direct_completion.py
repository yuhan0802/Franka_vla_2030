#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from eef_direct_completion import CompletionSample, FreshObservationBarrier, StrictCompletionGate
from eef_pose_safety import ACTION_HZ, advance_action_deadline


def gate(timeout=2.0, stable=0.3):
    return StrictCompletionGate(0.0, 0.003, 0.02, 0.01, 0.05, stable, 0.1, timeout)


def sample(sequence, t, position=0.001, rotation=0.01, linear=0.005, angular=0.02, valid=True):
    return CompletionSample(sequence, t, position, rotation, linear, angular, valid)


class StrictCompletionGateTest(unittest.TestCase):
    def test_not_reached_does_not_release_next_inference(self):
        decision = gate().evaluate(0.1, sample(1, 0.1, position=0.004))
        self.assertFalse(decision.complete)
        self.assertEqual(decision.reason, "waiting")

    def test_briefly_qualified_resets_before_stable_duration(self):
        subject = gate(stable=0.3)
        self.assertFalse(subject.evaluate(0.1, sample(1, 0.1)).complete)
        self.assertFalse(subject.evaluate(0.25, sample(2, 0.25, linear=0.02)).complete)
        self.assertFalse(subject.evaluate(0.5, sample(3, 0.5)).complete)
        self.assertFalse(subject.evaluate(0.7, sample(4, 0.7)).complete)

    def test_consecutive_new_feedback_releases_after_stable_duration(self):
        subject = gate(stable=0.3)
        self.assertFalse(subject.evaluate(0.1, sample(1, 0.1)).complete)
        self.assertFalse(subject.evaluate(0.25, sample(2, 0.25)).complete)
        decision = subject.evaluate(0.41, sample(3, 0.41))
        self.assertTrue(decision.complete)
        self.assertEqual(decision.reason, "reached")

    def test_timeout_is_failure_not_success(self):
        subject = gate(timeout=1.0)
        decision = subject.evaluate(1.01, sample(1, 1.01, position=0.02))
        self.assertFalse(decision.complete)
        self.assertEqual(decision.reason, "timeout")
        self.assertTrue(decision.must_stop)

    def test_stale_or_repeated_feedback_cannot_advance_stability(self):
        subject = gate(timeout=1.0, stable=0.3)
        first = sample(7, 0.1)
        self.assertFalse(subject.evaluate(0.1, first).complete)
        self.assertFalse(subject.evaluate(0.2, first).complete)
        self.assertFalse(subject.evaluate(0.3, first).complete)
        decision = subject.evaluate(1.01, first)
        self.assertFalse(decision.complete)
        self.assertEqual(decision.reason, "feedback_stale")
        self.assertTrue(decision.must_stop)


class FreshObservationBarrierTest(unittest.TestCase):
    def test_pre_completion_image_is_rejected_even_if_read_later(self):
        barrier = FreshObservationBarrier(1_000, 1.0, 0.2)
        ok, reason = barrier.check(
            1.1,
            {"exterior_image": 999, "wrist_image": 1_010, "arm_state": 1_020},
            {"exterior_image": 1.05, "wrist_image": 1.06, "arm_state": 1.07})
        self.assertFalse(ok)
        self.assertIn("not acquired after", reason)
        self.assertTrue(barrier.timed_out(6.0, 5.0))

    def test_all_post_completion_inputs_are_accepted(self):
        barrier = FreshObservationBarrier(1_000, 1.0, 0.2)
        ok, _ = barrier.check(
            1.1,
            {"exterior_image": 1_001, "wrist_image": 1_010, "arm_state": 1_020},
            {"exterior_image": 1.05, "wrist_image": 1.06, "arm_state": 1.07})
        self.assertTrue(ok)

    def test_old_images_timeout_requires_stop_and_never_infers(self):
        barrier = FreshObservationBarrier(1_000, 1.0, 0.2)
        ok, _ = barrier.check(
            6.0, {"exterior_image": 999, "wrist_image": 998},
            {"exterior_image": 5.99, "wrist_image": 5.99})
        inference_requested = ok
        stop_required = not ok and barrier.timed_out(6.0, 5.0)
        self.assertFalse(inference_requested)
        self.assertTrue(stop_required)


class ActionCadenceTest(unittest.TestCase):
    def test_chunk_internal_cadence_remains_15_hz_and_never_bursts(self):
        period = 1.0 / ACTION_HZ
        deadline = 10.0
        deadline = advance_action_deadline(deadline, 10.0, ACTION_HZ)
        self.assertAlmostEqual(deadline, 10.0 + period)
        deadline = advance_action_deadline(deadline, 10.5, ACTION_HZ)
        self.assertEqual(deadline, 10.5)


if __name__ == "__main__":
    unittest.main()
