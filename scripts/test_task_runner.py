"""Offline launcher tests: all backend service and robot calls are replaced."""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from resolve_direct_task import resolve_task

ROOT = Path(__file__).resolve().parents[1]


class TaskRunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / "runner project"
        (self.root / "scripts").mkdir(parents=True)
        for name in ("run_task.sh", "task1.sh", "task2.sh", "task6.sh"):
            shutil.copy(ROOT / name, self.root / name)
        for name in ("resolve_direct_task.py", "select_vla_task.py"):
            shutil.copy(ROOT / "scripts" / name, self.root / "scripts" / name)
        self.config = json.loads((ROOT / "vla_instruction_weights.json").read_text())
        self.config_path = self.root / "vla_instruction_weights.json"
        self.config_path.write_text(json.dumps(self.config))
        self.trace = self.root / "trace.jsonl"
        backend = self.root / "run_pi05_direct.sh"
        backend.write_text('''#!/usr/bin/env python3
import json, os, sys
keys=('VLA_REMOTE_USER','PI05_SERVER_GPU','PI05_CHECKPOINT_ROOT','PI05_CHECKPOINT_STEP','PI05_POLICY_CONFIG','PI05_DIRECT_GRIPPER_OPEN_WIDTH','PI05_DIRECT_GRIPPER_CLOSED_WIDTH','PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH','PI05_DIRECT_GRIPPER_FEEDBACK_CLOSED_WIDTH','PI05_DIRECT_GRIPPER_VERIFY_OPEN_WIDTH')
with open(os.environ['VLA_TASK_CONFIG']) as f: config=json.load(f)
row={'action':sys.argv[1],'instruction':os.environ['VLA_INSTRUCTION'],'config':config,'config_path':os.environ['VLA_TASK_CONFIG'],'env':{k:os.environ.get(k) for k in keys}}
with open(os.environ['TEST_TRACE'],'a') as f:f.write(json.dumps(row)+'\\n')
sys.exit(int(os.environ.get('TEST_BACKEND_EXIT','0')))
''')
        backend.chmod(0o755)
        init = self.root / "fci_init.sh"
        init.write_text('#!/bin/sh\necho \'{"action":"init"}\' >> "$TEST_TRACE"\n')
        init.chmod(0o755)
        self.env = dict(os.environ, VLA_TASK_CONFIG=str(self.config_path), TEST_TRACE=str(self.trace),
                        VLA_REMOTE_USER="xmu", PI05_SERVER_GPU="1")
        for name in ("PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH", "PI05_DIRECT_GRIPPER_FEEDBACK_CLOSED_WIDTH",
                     "PI05_DIRECT_GRIPPER_VERIFY_OPEN_WIDTH", "PI05_DIRECT_GRIPPER_OPEN_WIDTH",
                     "PI05_DIRECT_GRIPPER_CLOSED_WIDTH"):
            self.env.pop(name, None)

    def run_task(self, task, action=None, **overrides):
        args = [str(self.root / (task + ".sh"))]
        if action: args.append(action)
        return subprocess.run(args, env=dict(self.env, **overrides), capture_output=True, text=True)

    def calls(self):
        return [json.loads(x) for x in self.trace.read_text().splitlines()] if self.trace.exists() else []

    def test_run_keeps_each_checkpoint_and_effective_runtime(self):
        runtime = next(t["runtime"] for t in self.config["tasks"] if t.get("id") == "task2")
        for task_id in ("task1", "task2", "task6"):
            with self.subTest(task_id=task_id):
                result = self.run_task(task_id)
                self.assertEqual(result.returncode, 0, result.stderr)
                call = self.calls()[-1]
                source = next(t for t in self.config["tasks"] if t.get("id") == task_id)
                selected = call["config"]["tasks"][0]
                self.assertEqual(call["action"], "full-execute")
                self.assertEqual(call["instruction"], source["instruction"])
                self.assertEqual(selected["checkpoint_root"], source["checkpoint_root"])
                self.assertEqual(selected["runtime"], runtime)
                self.assertFalse(Path(call["config_path"]).exists(), "temporary config must be removed")
        self.assertEqual(len(self.calls()), 3, "run must not implicitly initialize the robot")

    def test_task1_feedback_overrides_and_stale_weights_are_handled(self):
        result = self.run_task("task1", PI05_CHECKPOINT_ROOT="/stale/task2", PI05_CHECKPOINT_STEP="1",
                               PI05_POLICY_CONFIG="stale", PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH="0.142")
        self.assertEqual(result.returncode, 0, result.stderr)
        env = self.calls()[0]["env"]
        self.assertEqual(env["PI05_DIRECT_GRIPPER_FEEDBACK_OPEN_WIDTH"], "0.142")
        self.assertEqual(env["PI05_DIRECT_GRIPPER_FEEDBACK_CLOSED_WIDTH"], "0.0")
        self.assertEqual(env["PI05_DIRECT_GRIPPER_VERIFY_OPEN_WIDTH"], "0")
        self.assertEqual(env["PI05_DIRECT_GRIPPER_OPEN_WIDTH"], "0.07")
        self.assertIsNone(env["PI05_CHECKPOINT_ROOT"])
        self.assertIsNone(env["PI05_CHECKPOINT_STEP"])
        self.assertIsNone(env["PI05_POLICY_CONFIG"])

    def test_show_does_not_call_services(self):
        result = self.run_task("task1", "show")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["tasks"][0]["runtime"]["max_actions"], 2000)
        self.assertEqual(self.calls(), [])

    def test_stop_preserves_existing_task_specific_init_policy(self):
        for task in ("task1", "task2", "task6"):
            with self.subTest(task=task):
                if self.trace.exists(): self.trace.unlink()
                result = self.run_task(task, "stop")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual([r["action"] for r in self.calls()], ["stop", "init"] if task == "task1" else ["stop"])

    def test_backend_failure_is_propagated_without_init(self):
        result = self.run_task("task1", "stop", TEST_BACKEND_EXIT="7")
        self.assertEqual(result.returncode, 7)
        self.assertEqual([r["action"] for r in self.calls()], ["stop"])

    def test_new_task_requires_only_a_config_entry(self):
        task = copy.deepcopy(next(t for t in self.config["tasks"] if t.get("id") == "task1"))
        task.update(id="task9", instruction="new task", checkpoint_root="/new/checkpoint")
        self.config["tasks"].append(task)
        self.config_path.write_text(json.dumps(self.config))
        result = subprocess.run([str(self.root / "run_task.sh"), "task9"], env=self.env,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.calls()[0]["config"]["tasks"][0]["checkpoint_root"], "/new/checkpoint")

    def test_unknown_task_and_circular_inheritance_do_not_call_backend(self):
        with self.assertRaisesRegex(ValueError, "unknown task"):
            resolve_task(self.config, "missing")
        task2 = next(t for t in self.config["tasks"] if t.get("id") == "task2")
        task2["direct"] = {"runtime_from": "task1"}
        self.config_path.write_text(json.dumps(self.config))
        result = self.run_task("task1")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("circular", result.stderr)
        self.assertEqual(self.calls(), [])


if __name__ == "__main__":
    unittest.main()
