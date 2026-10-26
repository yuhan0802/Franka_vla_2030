"""Offline regression checks: no ROS publishers, hardware, or network."""
import ast
import collections
import os
import pathlib
import sys
import types
import typing
import unittest
from unittest.mock import Mock

import numpy as np

SCRIPTS = pathlib.Path(os.environ.get("PI05_SCRIPTS_DIR", str(
    pathlib.Path(__file__).resolve().parents[1] / "workspace/src/franka_vr/franka_vr/scripts"
)))
sys.path.insert(0, str(SCRIPTS))
from pi05_joint_velocity_safety import (
    JointVelocityGuard, validate_action_chunk, gripper_closedness, gripper_target_width,
)


class SerialTests(unittest.TestCase):
    def method(self, name, script="pi05_joint_velocity_adapter.py", class_name="Pi05JointVelocityAdapter"):
        tree = ast.parse((SCRIPTS / script).read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        namespace = {
            "time": types.SimpleNamespace(monotonic=lambda: 10.0),
            "JointState": object,
            "Optional": typing.Optional,
            "np": np,
            "RCLError": RuntimeError,
            "rclpy": types.SimpleNamespace(spin_once=lambda node, timeout_sec: setattr(node, "gripper_time", 10.1)),
            "JOINT_NAMES": tuple(f"fr3_joint{i}" for i in range(1, 8)),
            "closedness_from_width": lambda width, open_width, closed_width: max(
                0.0, min(1.0, (open_width - width) / (open_width - closed_width))),
        }
        exec(compile(ast.Module(body=[method], type_ignores=[]), "adapter_method", "exec"), namespace)
        return namespace[name]

    def direct_method(self, name):
        return self.method(name, "pi05_direct_policy_adapter.py", "DirectAdapter")

    def test_no_extra_scale_or_normalized_clipping(self):
        chunk = np.zeros((16, 8))
        chunk[0, 0] = 1.2
        self.assertEqual(validate_action_chunk(chunk)[0, 0], 1.2)
        action = np.array([0.01] * 7 + [0.5])
        guard = JointVelocityGuard(max_velocity=0.15, max_acceleration=0.5,
                                   action_units="direct-rad-s")
        result = guard.apply(action, [0, 0, 0, -1.5, 0, 2, 0], 1 / 15)
        np.testing.assert_allclose(result.raw_velocity, action[:7])
        np.testing.assert_allclose(result.velocity, action[:7])
        chunk[0, 0] = np.nan
        with self.assertRaises(ValueError):
            validate_action_chunk(chunk)

    def test_training_velocity_and_gripper_inverse(self):
        physical = np.array([0.01, -0.02, 0.03, -0.01, 0.02, -0.03, 0.01])
        action = np.r_[physical / 3.0, 0.75]
        result = JointVelocityGuard().apply(action, [0, 0, 0, -1.5, 0, 2, 0], 1 / 15)
        np.testing.assert_allclose(result.raw_velocity, physical)
        np.testing.assert_allclose(result.velocity, physical)
        for width in [0.0, 0.01, 0.02, 0.04, 0.08]:
            self.assertAlmostEqual(gripper_target_width(gripper_closedness(width)), width)

    def test_inference_waits_for_full_chunk_and_fresh_observations(self):
        node = types.SimpleNamespace(
            finished=False, fault_latched=False, gripper_busy=False,
            inference_future=None, args=types.SimpleNamespace(execute=True, max_actions=240, action_hz=15),
            executed_actions=16, actions=collections.deque([1]), last_action_time=9.0,
            gap_halted=True, joint_time=10.0, image_times={"left": 10.0, "wrist": 10.0},
            _observation=Mock(return_value={"fresh": True}),
            inference_pool=Mock(), policy=types.SimpleNamespace(infer=Mock()),
        )
        tick = self.method("_policy_tick")
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.actions.clear()
        node.last_action_time = 9.99
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.last_action_time = 9.0
        node.gap_halted = False
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.gap_halted = True
        node.image_times["left"] = 9.0
        tick(node)
        node.inference_pool.submit.assert_not_called()
        node.image_times["left"] = 10.0
        tick(node)
        node.inference_pool.submit.assert_called_once()

    def test_last_action_gets_full_period_before_stop(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(execute=True, max_actions=16, action_hz=15),
            finished=False, fault_latched=False, gripper_busy=False,
            actions=collections.deque(), executed_actions=16, last_action_time=9.99,
            _finish_execution=Mock(),
        )
        tick = self.method("_action_tick")
        tick(node)
        node._finish_execution.assert_not_called()
        node.last_action_time = 9.0
        tick(node)
        node._finish_execution.assert_called_once()

    def test_direct_adapter_keeps_arm_and_gripper_feedback_sources_separate(self):
        arm_cb = self.direct_method("arm_joint_cb")
        gripper_cb = self.direct_method("gripper_joint_cb")
        arm_names = [f"fr3_joint{i}" for i in range(1, 8)]
        node = types.SimpleNamespace(joints={}, joint_time=0.0, fingers={}, gripper_time=0.0)

        # A merged/robot-state message containing misleading mimic fingers must
        # never overwrite the dedicated gripper measurement.
        arm_message = types.SimpleNamespace(
            name=arm_names + ["fr3_finger_joint1", "fr3_finger_joint2"],
            position=[float(i) for i in range(7)] + [0.035, 0.035],
        )
        arm_cb(node, arm_message)
        self.assertEqual(node.fingers, {})
        self.assertEqual(len(node.joints), 7)

        gripper_message = types.SimpleNamespace(
            name=["fr3_finger_joint1", "fr3_finger_joint2"],
            position=[0.071, 0.072],
        )
        gripper_cb(node, gripper_message)
        self.assertAlmostEqual(sum(node.fingers.values()), 0.143)
        arm_cb(node, arm_message)
        self.assertAlmostEqual(sum(node.fingers.values()), 0.143)

    def test_direct_adapter_replans_after_empty_grasp_instead_of_latching(self):
        result_cb = self.direct_method("gripper_result")
        node = types.SimpleNamespace(
            fingers={"fr3_finger_joint1": 0.001, "fr3_finger_joint2": 0.001},
            args=types.SimpleNamespace(
                gripper_open_width=0.08, gripper_closed_width=0.01,
                gripper_feedback_open_width=0.08, gripper_feedback_closed_width=0.01,
                gripper_close_threshold=0.6, gripper_open_tolerance=0.01,
                gripper_max_plausible_width=0.085, gripper_verify_timeout=2.0,
                gripper_require_grasp_success=1),
            pending_gripper_closed=True, last_gripper_closed=False, gripper_busy=True,
            gripper_time=9.9, gripper_verification=None,
            queue=Mock(), need_fresh_after=0.0, log=Mock(), latch=Mock(),
            get_logger=lambda: types.SimpleNamespace(warning=Mock()),
        )
        future = Mock()
        future.result.return_value = types.SimpleNamespace(
            result=types.SimpleNamespace(success=False, error=""))
        result_cb(node, future)
        self.assertTrue(node.gripper_busy)
        node.queue.clear_for_gripper.assert_not_called()
        # Only feedback received after the action-completion settling window
        # can trigger recovery. Test clock is 10.0 seconds.
        node.gripper_verification = (9.6, future.result.return_value.result)
        node.gripper_feedback_error = types.MethodType(self.direct_method("gripper_feedback_error"), node)
        self.direct_method("verify_gripper_result")(node)
        node.latch.assert_not_called()
        node.queue.clear_for_gripper.assert_called_once()
        self.assertFalse(node.gripper_busy)
        self.assertTrue(node.last_gripper_closed)
        self.assertIsNone(node.pending_gripper_closed)
        self.assertEqual(node.need_fresh_after, 10.0)

    def test_direct_timers_do_nothing_before_arm_finishes(self):
        node = types.SimpleNamespace(startup_ready=False)
        self.direct_method("policy_tick")(node)
        self.direct_method("action_tick")(node)

    def test_disabled_grasp_check_preserves_remaining_chunk(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(gripper_verify_timeout=2.0, gripper_require_grasp_success=0),
            gripper_verification=(9.6, types.SimpleNamespace(success=False, error="")),
            gripper_time=9.9, gripper_busy=True, pending_gripper_closed=True,
            fingers={"fr3_finger_joint1": 0.001, "fr3_finger_joint2": 0.001},
            gripper_feedback_error=Mock(return_value=None), latch=Mock(),
            queue=collections.deque([1, 2, 3]), log=Mock(),
            get_logger=lambda: types.SimpleNamespace(info=Mock(), warning=Mock()),
        )
        self.direct_method("verify_gripper_result")(node)
        self.assertEqual(list(node.queue), [1, 2, 3])
        self.assertFalse(node.gripper_busy)
        self.assertTrue(node.last_gripper_closed)
        node.latch.assert_not_called()

    def test_direct_feedback_range_check_disabled_but_nonfinite_rejected(self):
        check = self.direct_method("gripper_feedback_error")
        node = types.SimpleNamespace(args=types.SimpleNamespace(gripper_max_plausible_width=0.085))
        for values in ((-0.0005300146294757724,) * 2, (0.07085, 0.07085)):
            node.fingers = dict(zip(("fr3_finger_joint1", "fr3_finger_joint2"), values))
            self.assertIsNone(check(node))
        for values in ((float("nan"), 0.01), (0.01, float("inf"))):
            node.fingers = dict(zip(("fr3_finger_joint1", "fr3_finger_joint2"), values))
            self.assertIsNotNone(check(node))

    def test_direct_arm_accepts_negative_width_with_range_check_disabled(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(execute=True, enable_gripper=True,
                                       gripper_max_plausible_width=0.085,
                                       gripper_open_width=0.07, gripper_closed_width=0.0,
                                       gripper_feedback_open_width=0.14, gripper_feedback_closed_width=0.0,
                                       gripper_close_threshold=0.6),
            count_subscribers=lambda topic: 1,
            move_client=Mock(), grasp_client=Mock(),
            fingers={"fr3_finger_joint1": -0.0005, "fr3_finger_joint2": -0.0005},
            gripper_time=0.0, startup_ready=False, fault=False,
            get_logger=lambda: types.SimpleNamespace(info=Mock()),
        )
        node.gripper_feedback_error = types.MethodType(self.direct_method("gripper_feedback_error"), node)
        self.direct_method("arm")(node)
        self.assertTrue(node.startup_ready)
        self.assertTrue(node.last_gripper_closed)
        node.move_client.send_goal_async.assert_not_called()
        node.grasp_client.send_goal_async.assert_not_called()

    def test_direct_open_waits_for_new_feedback_and_times_out_if_stuck(self):
        verify = self.direct_method("verify_gripper_result")
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(gripper_verify_timeout=2.0, gripper_open_width=0.08,
                                       gripper_verify_open_width=1,
                                       gripper_feedback_open_width=0.08,
                                       gripper_open_tolerance=0.01),
            gripper_verification=(9.6, types.SimpleNamespace(success=True)),
            gripper_time=9.5, gripper_busy=True, pending_gripper_closed=False,
            fingers={"fr3_finger_joint1": 0.0355, "fr3_finger_joint2": 0.0355},
            gripper_feedback_error=Mock(return_value=None), latch=Mock(), queue=[],
            get_logger=lambda: types.SimpleNamespace(warning=Mock()),
        )
        verify(node)
        self.assertTrue(node.gripper_busy, "old in-range feedback must not release the arm")
        node.gripper_time = 9.9
        verify(node)
        self.assertFalse(node.gripper_busy)
        self.assertIsNone(node.gripper_verification)
        node.gripper_verification = (9.6, types.SimpleNamespace(success=True))
        node.pending_gripper_closed = False
        node.gripper_busy = True
        node.fingers = {"fr3_finger_joint1": 0.001, "fr3_finger_joint2": 0.001}
        verify(node)
        self.assertTrue(node.gripper_busy)
        node.latch.assert_not_called()
        node.gripper_verification = (7.0, types.SimpleNamespace(success=True))
        verify(node)
        node.latch.assert_called_once()

    def test_direct_open_feedback_uses_014_scale_separate_from_motion_target(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(gripper_verify_timeout=2.0, gripper_open_width=0.07,
                                       gripper_verify_open_width=1,
                                       gripper_feedback_open_width=0.14, gripper_open_tolerance=0.005),
            gripper_verification=(9.6, types.SimpleNamespace(success=True)),
            gripper_time=9.9, gripper_busy=True, pending_gripper_closed=False,
            fingers={"fr3_finger_joint1": 0.035, "fr3_finger_joint2": 0.035},
            gripper_feedback_error=Mock(return_value=None), latch=Mock(), queue=[],
            get_logger=lambda: types.SimpleNamespace(warning=Mock()),
        )
        verify = self.direct_method("verify_gripper_result")
        verify(node)
        self.assertTrue(node.gripper_busy, "0.07 m feedback is not full opening on the 0.14 m scale")
        node.fingers = {name: 0.07 for name in node.fingers}
        verify(node)
        self.assertFalse(node.gripper_busy)
        self.assertFalse(node.last_gripper_closed)
        self.assertEqual(node.args.gripper_open_width, 0.07)
        node.latch.assert_not_called()

    def test_disabled_open_width_check_accepts_negative_feedback_after_success(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(gripper_verify_timeout=2.0, gripper_verify_open_width=0),
            gripper_verification=(9.6, types.SimpleNamespace(success=True)),
            gripper_time=9.5, gripper_busy=True, pending_gripper_closed=False,
            fingers={"fr3_finger_joint1": -0.0009, "fr3_finger_joint2": -0.0009},
            latch=Mock(), queue=[1, 2],
            get_logger=lambda: types.SimpleNamespace(warning=Mock()),
        )
        node.gripper_feedback_error = types.MethodType(self.direct_method("gripper_feedback_error"), node)
        verify = self.direct_method("verify_gripper_result")
        verify(node)
        self.assertTrue(node.gripper_busy, "disabled width checking must still wait for fresh feedback")
        node.gripper_time = 9.9
        node.fingers["fr3_finger_joint1"] = float("nan")
        verify(node)
        node.latch.assert_called_once()
        self.assertTrue(node.gripper_busy)
        node.latch.reset_mock()
        node.fingers["fr3_finger_joint1"] = -0.0009
        verify(node)
        self.assertFalse(node.gripper_busy)
        self.assertFalse(node.last_gripper_closed)
        self.assertIsNone(node.gripper_verification)
        self.assertEqual(node.queue, [1, 2])
        node.latch.assert_not_called()

    def test_direct_observation_normalizes_raw_width_with_feedback_endpoints(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(enable_gripper=True, max_observation_age=0.5,
                                       max_camera_skew=0.15, max_state_image_skew=0.15,
                                       gripper_open_width=0.07, gripper_closed_width=0.01,
                                       gripper_feedback_open_width=0.14,
                                       gripper_feedback_closed_width=0.0, instruction="test"),
            images={key: np.zeros((2, 2, 3), dtype=np.uint8) for key in ("left", "wrist")},
            image_times={"left": 9.9, "wrist": 9.9}, joint_time=9.9, gripper_time=9.9,
            joints={f"fr3_joint{i}": 0.0 for i in range(1, 8)}, need_fresh_after=0.0,
        )
        observation = self.direct_method("observation")
        for raw_width, expected_closedness in ((0.14, 0.0), (0.07, 0.5), (0.0, 1.0)):
            with self.subTest(width=raw_width):
                node.fingers = {name: raw_width / 2 for name in ("fr3_finger_joint1", "fr3_finger_joint2")}
                state = observation(node)
                self.assertAlmostEqual(float(state["observation/gripper_position"][0]), expected_closedness)

    def test_direct_startup_detects_014_feedback_as_open_without_moving(self):
        node = types.SimpleNamespace(
            args=types.SimpleNamespace(execute=True, enable_gripper=True,
                                       gripper_feedback_open_width=0.14,
                                       gripper_feedback_closed_width=0.0,
                                       gripper_close_threshold=0.6),
            count_subscribers=lambda topic: 1, move_client=Mock(), grasp_client=Mock(),
            fingers={"fr3_finger_joint1": 0.071, "fr3_finger_joint2": 0.071},
            gripper_time=0.0, startup_ready=False, fault=False,
            get_logger=lambda: types.SimpleNamespace(info=Mock()),
        )
        node.gripper_feedback_error = types.MethodType(self.direct_method("gripper_feedback_error"), node)
        self.direct_method("arm")(node)
        self.assertTrue(node.startup_ready)
        self.assertFalse(node.last_gripper_closed)
        node.move_client.send_goal_async.assert_not_called()
        node.grasp_client.send_goal_async.assert_not_called()

    def test_direct_hardware_error_still_latches(self):
        node = types.SimpleNamespace(pending_gripper_closed=True, latch=Mock())
        future = Mock()
        future.result.return_value = types.SimpleNamespace(
            result=types.SimpleNamespace(success=False, error="Command failed!"))
        self.direct_method("gripper_result")(node, future)
        node.latch.assert_called_once()

    def test_direct_reports_and_times_out_missing_observations(self):
        tick = self.direct_method("policy_tick")
        node = types.SimpleNamespace(
            startup_ready=True, finished=False, fault=False, gripper_verification=None,
            gripper_busy=False, future=None, queue=[], executed=0,
            args=types.SimpleNamespace(execute=True, max_actions=2000, max_observation_wait=10),
            observation=Mock(return_value=None), observation_wait_started=None,
            observation_warned_at=0.0, observation_blocker="joint_age=12.00s", latch=Mock(),
            pool=Mock(), get_logger=lambda: types.SimpleNamespace(warning=Mock()))
        tick(node)
        self.assertEqual(node.observation_wait_started, 10.0)
        node.latch.assert_not_called()
        node.observation_wait_started = 0.0
        tick(node)
        node.latch.assert_called_once_with("observation wait timeout: joint_age=12.00s")
        node.pool.submit.assert_not_called()

    def test_stop_command_skips_invalid_context(self):
        node = types.SimpleNamespace(publisher=Mock(), context=types.SimpleNamespace(ok=lambda: False))
        self.direct_method("stop_command")(node)
        node.publisher.publish.assert_not_called()

    def test_stop_publish_shutdown_race_is_ignored_but_live_errors_raise(self):
        stop = self.direct_method("stop_command")
        # Provide the message constructor without importing ROS on the host.
        stop.__globals__["JointJog"] = lambda: types.SimpleNamespace(header=types.SimpleNamespace())
        node = types.SimpleNamespace(
            publisher=Mock(), context=Mock(), args=types.SimpleNamespace(action_hz=15),
            get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None)))
        node.publisher.publish.side_effect = RuntimeError("publisher context invalid")
        node.context.ok.side_effect = [True, False]
        stop(node)
        node.context.ok.side_effect = [True, True]
        with self.assertRaisesRegex(RuntimeError, "publisher context invalid"):
            stop(node)

    def test_main_cleans_up_interrupt_without_double_shutdown(self):
        tree = ast.parse((SCRIPTS / "pi05_direct_policy_adapter.py").read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
        class ExternalShutdown(Exception):
            pass
        for error in (KeyboardInterrupt(), ExternalShutdown()):
            with self.subTest(error=type(error).__name__):
                ros = Mock()
                ros.ok.return_value = True
                ros.spin_once.side_effect = error
                node = types.SimpleNamespace(arm=Mock(), finished=False, fault=False, destroy_node=Mock())
                namespace = dict(parse_args=Mock(), rclpy=ros, DirectAdapter=lambda args: node,
                                 ExternalShutdownException=ExternalShutdown)
                exec(compile(ast.Module(body=[main], type_ignores=[]), "main", "exec"), namespace)
                with self.assertRaises(SystemExit) as exit_status:
                    namespace["main"]()
                self.assertEqual(exit_status.exception.code, 130)
                node.destroy_node.assert_called_once()
                ros.try_shutdown.assert_called_once()
                ros.shutdown.assert_not_called()


if __name__ == "__main__":
    unittest.main()
