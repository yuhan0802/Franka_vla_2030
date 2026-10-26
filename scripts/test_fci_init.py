"""Offline FCI init/stop regressions. Docker and hardware are never invoked."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
Q = [-0.00145, 0.167096, -0.000722, -2.147145, 0.003645, 2.293680, 0.771709]


class FciInitTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.trace = self.root / "trace.jsonl"
        self.pose = self.root / "default_initial_pose.json"
        self.pose.write_text(json.dumps({"version": 1, "robot_ip": "192.0.2.1",
                                       "joint_names": [f"fr3_joint{i}" for i in range(1, 8)], "q_rad": Q}))
        self.env = dict(os.environ, PATH=str(self.bin) + ":" + os.environ["PATH"],
                        FRANKA_LOG_ROOT=str(self.root / "logs"), ROBOT_IP="192.0.2.1",
                        FCI_RESET_POSE_FILE=str(self.pose), TEST_TRACE=str(self.trace))
        self.executable("ss", "#!/bin/sh\nif [ \"${TEST_BUSY:-0}\" = 1 ]; then echo ESTAB; fi\n")
        self.executable("docker", '''#!/usr/bin/env python3
import json, os, sys
args=sys.argv[1:]
assert args[0]=='run', args
env={}
for i,arg in enumerate(args):
    if arg=='-e':
        key,value=args[i+1].split('=',1); env[key]=value
mode=env.get('FCI_ARM_MODE','gripper')
with open(os.environ['TEST_TRACE'],'a') as stream: stream.write(json.dumps({'mode':mode})+'\\n')
if mode=='gripper':
    assert 'GRIPPER_MOTION HOMING' in args[-1]
    if os.environ.get('TEST_FAIL')=='1': sys.exit(7)
    print(json.dumps({'event':'move_result','phase':'HOMING','success':True,'timed_out':False,'elapsed_s':3.4}))
    print(json.dumps({'event':'state','phase':'HOMING_SETTLED','width_m':.1419,'max_width_m':.1420}))
elif mode=='capture':
    print(json.dumps({'event':'arm_initial_state','robot_mode':1,'q':[-.00145,.167096,-.000722,-2.147145,.003645,2.293680,.771709],'dq':[0]*7}))
elif mode=='reset':
    print(json.dumps({'event':'arm_reset_completed','success':True}))
else: assert mode=='check', mode
''')

    def executable(self, name, text):
        path = self.bin / name
        path.write_text(text)
        path.chmod(0o755)

    def run_init(self, *args, **env):
        return subprocess.run([str(ROOT / "fci_init.sh"), *args], env=dict(self.env, **env),
                              text=True, capture_output=True)

    def modes(self):
        return [json.loads(x)["mode"] for x in self.trace.read_text().splitlines()] if self.trace.exists() else []

    def test_default_homes_gripper_then_returns_arm_without_changing_saved_pose(self):
        saved = self.pose.read_bytes()
        result = self.run_init()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.modes(), ["gripper", "reset"])
        self.assertEqual(self.pose.read_bytes(), saved)

    def test_gripper_failure_prevents_arm_motion(self):
        result = self.run_init(TEST_FAIL="1")
        self.assertEqual(result.returncode, 7)
        self.assertEqual(self.modes(), ["gripper"])

    def test_gripper_only_does_not_reset_arm(self):
        result = self.run_init("--gripper-only")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.modes(), ["gripper"])

    def test_save_pose_only_reads_arm(self):
        self.pose.unlink()
        result = self.run_init("--save-pose")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.modes(), ["capture"])
        self.assertEqual(json.loads(self.pose.read_text())["q_rad"], Q)

    def test_invalid_target_or_busy_client_prevents_all_motion(self):
        for condition in ("wrong_robot", "invalid_joint", "busy_client"):
            with self.subTest(condition=condition):
                pose = json.loads(self.pose.read_text())
                if condition == "wrong_robot": pose["robot_ip"] = "192.0.2.2"
                elif condition == "invalid_joint": pose["q_rad"][0] = 10
                self.pose.write_text(json.dumps(pose))
                result = self.run_init(TEST_BUSY="1" if condition == "busy_client" else "0")
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.modes(), [])
                pose["robot_ip"] = "192.0.2.1"; pose["q_rad"] = Q
                self.pose.write_text(json.dumps(pose))

    def test_task_stop_resets_only_after_successful_shutdown(self):
        task_root = self.root / "task"
        task_root.mkdir()
        shutil.copy(ROOT / "task1.sh", task_root / "task1.sh")
        shutil.copy(ROOT / "run_task.sh", task_root / "run_task.sh")
        (task_root / "scripts").mkdir()
        for name in ("resolve_direct_task.py", "select_vla_task.py"):
            shutil.copy(ROOT / "scripts" / name, task_root / "scripts" / name)
        shutil.copy(ROOT / "vla_instruction_weights.json", task_root / "vla_instruction_weights.json")
        for name, body in (("run_pi05_direct.sh", 'echo "stop:$1" >> "$TEST_STOP_TRACE"\nexit "${TEST_STOP_FAILURE:-0}"'),
                           ("fci_init.sh", 'echo init >> "$TEST_STOP_TRACE"')):
            path = task_root / name
            path.write_text("#!/bin/sh\n" + body + "\n")
            path.chmod(0o755)
        for arg, failure, expected in (("stop", "0", ["stop:stop", "init"]),
                                       ("stop", "1", ["stop:stop"]),
                                       ("stop-only", "0", ["stop:stop"])):
            trace = self.root / "stop_trace"
            if trace.exists(): trace.unlink()
            result = subprocess.run(["bash", str(task_root / "task1.sh"), arg],
                                    env=dict(self.env, VLA_TASK_CONFIG=str(task_root / "vla_instruction_weights.json"),
                                             TEST_STOP_TRACE=str(trace), TEST_STOP_FAILURE=failure))
            self.assertEqual(result.returncode, int(failure))
            self.assertEqual(trace.read_text().splitlines(), expected)


if __name__ == "__main__":
    unittest.main()
