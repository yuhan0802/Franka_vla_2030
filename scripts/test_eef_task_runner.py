"""Offline EEF task/model selection and lifecycle tests; no robot or SSH."""
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from eef_task_config import resolve, environment

ROOT = Path(__file__).resolve().parents[1]


class EefTaskRunnerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="eef_launcher_")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / "repo with spaces"
        for directory in ("scripts", "config", "bin", "logs"):
            (self.root / directory).mkdir(parents=True)
        for name in ("run_eef_task.sh", "task1-act-v2.sh", "task1-act.sh", "task1-act-1k.sh", "task1-act2.sh", "task1-droid.sh"):
            shutil.copy(ROOT / name, self.root / name)
        for name in ("eef_task_config.py", "runtime_log_root.sh"):
            shutil.copy(ROOT / "scripts" / name, self.root / "scripts" / name)
        self.config = json.loads((ROOT / "config/eef_tasks.json").read_text())
        self.config_path = self.root / "config/eef_tasks.json"
        self.config_path.write_text(json.dumps(self.config))
        self.trace = self.root / "trace.jsonl"
        backend = '''#!/usr/bin/env python3
import json, os, pathlib, sys
kind='direct' if 'direct' in pathlib.Path(sys.argv[0]).name else 'servo'
mode=sys.argv[1]
keys=('EEF_CHECKPOINT','EEF_SERVER_HOST','EEF_SERVER_PORT','EEF_SERVER_GPU','EEF_SERVER_SCRIPT','EEF_HOME_POSITION','EEF_HOME_QUATERNION','VLA_INSTRUCTION','EEF_EXECUTE_HORIZON','EEF_MAX_CHUNKS','EEF_MAX_ACTIONS','EEF_MAX_TF_AGE','EEF_ENABLE_GRIPPER')
with open(os.environ['EEF_TEST_TRACE'],'a') as f:f.write(json.dumps({'kind':kind,'mode':mode,'env':{k:os.environ.get(k) for k in keys}})+'\\n')
if mode in ('stop','server-stop') and os.environ.get('EEF_TEST_STOP_FAIL')==kind:sys.exit(9)
if mode in ('dry-run','axis-test'):
    log=pathlib.Path(os.environ['EEF_LOG_DIR']);log.mkdir(parents=True,exist_ok=True)
    pos=list(map(float,os.environ['EEF_HOME_POSITION'].split()))
    pos[0]+=float(os.environ.get('EEF_TEST_OFFSET','0'))
    rows=[{'event':'armed','position':pos,'quaternion_xyzw':list(map(float,os.environ['EEF_HOME_QUATERNION'].split()))}]
    if mode=='dry-run':rows.append({'event':'dry_run','robot_command_sent':os.environ.get('EEF_TEST_BAD_DRY')=='1'})
    else:rows.append({'event':'fault' if os.environ.get('EEF_TEST_BAD_AXIS')=='1' else 'complete'})
    (log/'policy.jsonl').write_text(''.join(json.dumps(r)+'\\n' for r in rows))
'''
        for name in ("run_eef_direct.sh", "run_eef_pose.sh"):
            self.executable(self.root / name, backend)
        self.executable(self.root / "fci_init.sh", '#!/bin/sh\necho \'{"kind":"init"}\' >> "$EEF_TEST_TRACE"\n')
        self.executable(self.root / "bin/docker", '''#!/usr/bin/env python3
import json, os, pathlib, sys
with open(os.environ['EEF_TEST_TRACE'],'a') as f:f.write(json.dumps({'kind':'docker','args':sys.argv[1:]})+'\\n')
if sys.argv[1]=='ps' and os.environ.get('EEF_TEST_ADAPTER')=='1' and not pathlib.Path(os.environ['EEF_TEST_KILLED']).exists():print('eef-adapter')
elif sys.argv[1]=='inspect':print('python3 eef_pose_direct_adapter.py --execute')
elif sys.argv[1]=='kill':pathlib.Path(os.environ['EEF_TEST_KILLED']).touch()
''')
        self.executable(self.root / "bin/tmux", "#!/bin/sh\nexit 1\n")
        self.env = dict(os.environ, PATH=str(self.root / "bin") + ":" + os.environ["PATH"],
                        EEF_TASK_CONFIG=str(self.config_path), FRANKA_LOG_ROOT=str(self.root / "logs"),
                        EEF_TEST_TRACE=str(self.trace), EEF_TEST_KILLED=str(self.root / "killed"))
        for key in ("EEF_CONTROL", "EEF_RUN_DIR", "EEF_MAX_CHUNKS", "EEF_MAX_ACTIONS", "EEF_DIRECT_SKIP_COMMISSION"):
            self.env.pop(key, None)

    def executable(self, path, text):
        path.write_text(text); path.chmod(0o755)

    def run_task(self, mode="run", task="task1", model="act-v2", extra=(), **env):
        return subprocess.run([str(self.root / "run_eef_task.sh"), task, model, mode, *extra],
                              env=dict(self.env, **env), capture_output=True, text=True, timeout=10)

    def calls(self):
        return [json.loads(x) for x in self.trace.read_text().splitlines()] if self.trace.exists() else []

    def mark_commissioned(self):
        (self.root / "logs/.eef_direct_commissioned").write_text("previous successful axis test\n")

    def test_show_and_list_do_not_start_resources(self):
        result = self.run_task("show", EEF_CHECKPOINT="/wrong/stale", EEF_SERVER_HOST="wrong")
        self.assertEqual(result.returncode, 0, result.stderr)
        profile = json.loads(result.stdout)
        self.assertEqual(profile["server"]["host"], "10.24.116.63")
        self.assertIn("v5_maxabs", profile["server"]["checkpoint"])
        self.assertEqual(profile["action_normalization"], {
            "method": "task0_maxabs", "params_path": profile["server"]["checkpoint"] + "/training_config.json",
            "params_key": "action_arm_scale"})
        result = subprocess.run([str(self.root / "run_eef_task.sh"), "list"], env=self.env, capture_output=True, text=True)
        self.assertIn("task2", result.stdout)
        self.assertIn("not configured", result.stdout)
        self.assertEqual(self.calls(), [])

    def test_all_five_wrapper_keys_resolve_existing_model_information(self):
        for script, model, host, port in (("task1-act-v2.sh", "act-v2", "10.24.116.63", 8095),
                                         ("task1-act.sh", "act", "10.24.116.20", 8095),
                                         ("task1-act-1k.sh", "act-1k", "10.24.116.63", 8095),
                                         ("task1-act2.sh", "act2", "10.24.116.20", 8096),
                                         ("task1-droid.sh", "droid", "10.24.116.20", 8097)):
            with self.subTest(model=model):
                result = subprocess.run([str(self.root / script), "show"], env=self.env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                p = json.loads(result.stdout)
                self.assertEqual((p["model_key"], p["server"]["host"], p["server"]["port"]), (model, host, port))

    def test_unknown_or_unconfigured_key_is_rejected_before_services(self):
        for task, model in (("missing", "act-v2"), ("task2", "act-v2"), ("task1", "missing")):
            result = self.run_task(task=task, model=model)
            self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_universal_mulaw9_metadata_matches_verified_checkpoint(self):
        result = self.run_task("show", model="universal-mulaw9")
        self.assertEqual(result.returncode, 0, result.stderr)
        profile = json.loads(result.stdout)
        n = profile["action_normalization"]
        self.assertEqual((n["method"], n["mu"]), ("franka_mulaw", 9))
        self.assertEqual(n["params_path"], profile["server"]["checkpoint"] + "/action_norm_stats.json")
        self.assertEqual(environment(profile)["EEF_EXPECTED_ACTION_NORMALIZATION"], "franka_mulaw")
        self.assertEqual(environment(profile)["EEF_EXPECTED_ACTION_MULAW_MU"], "9.0")

    def test_universal_maxabs_x2_metadata_matches_verified_checkpoint(self):
        result = self.run_task("show", model="universal-maxabs-x2")
        self.assertEqual(result.returncode, 0, result.stderr)
        profile = json.loads(result.stdout)
        n = profile["action_normalization"]
        self.assertEqual(n["method"], "franka_maxabs")
        self.assertEqual(n["params_path"], profile["server"]["checkpoint"] + "/action_norm_stats.json")
        self.assertEqual(profile["server"]["port"], 8099)
        self.assertEqual(environment(profile)["EEF_EXPECTED_ACTION_NORMALIZATION"], "franka_maxabs")
        self.assertEqual(environment(profile)["EEF_EXPECTED_ACTION_MULAW_MU"], "0")

    def test_direct_run_requires_successful_axis_record(self):
        result = self.run_task()
        self.assertEqual(result.returncode, 3)
        self.assertIn("axis-test", result.stderr)
        self.assertEqual(self.calls(), [])

    def test_axis_test_records_only_a_success(self):
        result = self.run_task("axis-test", extra=("+z",), EEF_TEST_BAD_AXIS="1")
        self.assertNotEqual(result.returncode, 0)
        marker = self.root / "logs/.eef_direct_commissioned"
        self.assertFalse(marker.exists())
        result = self.run_task("axis-test", extra=("+z",))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(marker.is_file())

    def test_run_dry_run_then_execute_full_horizon(self):
        self.mark_commissioned()
        result = self.run_task()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [r for r in self.calls() if r["kind"] == "direct"]
        self.assertEqual([r["mode"] for r in calls], ["server", "dry-run", "execute"])
        self.assertEqual(calls[-1]["env"]["EEF_EXECUTE_HORIZON"], "15")
        self.assertEqual(calls[-1]["env"]["EEF_MAX_ACTIONS"], "900")
        self.assertEqual(calls[-1]["env"]["EEF_MAX_TF_AGE"], "0.3")
        self.assertEqual(calls[-1]["env"]["EEF_ENABLE_GRIPPER"], "true")
        self.assertFalse(any(r["kind"] == "init" for r in self.calls()))

    def test_dry_run_pose_offset_never_homes_or_executes(self):
        result = self.run_task("dry-run", EEF_TEST_OFFSET="0.03")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([r["mode"] for r in self.calls() if r["kind"] == "direct"], ["server", "dry-run"])

    def test_run_pose_offset_homes_before_execution(self):
        self.mark_commissioned()
        result = self.run_task(EEF_TEST_OFFSET="0.03")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual([r["mode"] for r in self.calls() if r["kind"] == "direct"], ["server", "dry-run", "home", "execute"])

    def test_failed_dry_run_prevents_execution(self):
        self.mark_commissioned()
        result = self.run_task(EEF_TEST_BAD_DRY="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(r.get("mode") in ("home", "execute") for r in self.calls()))

    def test_stop_closes_adapter_and_both_stacks_before_fci_init(self):
        result = self.run_task("stop", EEF_TEST_ADAPTER="1")
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.calls()
        kill = next(i for i, r in enumerate(calls) if r["kind"] == "docker" and r["args"][0] == "kill")
        first_stop = next(i for i, r in enumerate(calls) if r["kind"] == "direct")
        self.assertLess(kill, first_stop)
        self.assertEqual([(r["kind"], r.get("mode")) for r in calls if r["kind"] != "docker"],
                         [("direct", "stop"), ("servo", "stop"), ("servo", "server-stop"), ("init", None)])

    def test_stop_failure_and_stop_only_never_initialize(self):
        for mode, fail in (("stop", "direct"), ("stop-only", "")):
            if self.trace.exists(): self.trace.unlink()
            result = self.run_task(mode, EEF_TEST_STOP_FAIL=fail)
            self.assertEqual(result.returncode, 1 if fail else 0, result.stderr)
            self.assertFalse(any(r["kind"] == "init" for r in self.calls()))

    def test_servo_control_and_new_task_pose_are_configurable(self):
        task = copy.deepcopy(self.config["tasks"]["task1"])
        task["instruction"] = "new task"
        task["start_pose"]["position"] = [.5, .1, .4]
        self.config["tasks"]["new_task"] = task
        self.config_path.write_text(json.dumps(self.config))
        result = self.run_task("home", task="new_task", EEF_CONTROL="servo")
        self.assertEqual(result.returncode, 0, result.stderr)
        call = self.calls()[0]
        self.assertEqual((call["kind"], call["mode"]), ("servo", "home"))
        self.assertEqual(call["env"]["EEF_HOME_POSITION"], "0.5 0.1 0.4")
        self.assertEqual(call["env"]["VLA_INSTRUCTION"], "new task")

    def test_invalid_model_identity_and_rotation_are_rejected(self):
        original = copy.deepcopy(self.config)
        for kind in ("rotation", "checkpoint", "port"):
            config = copy.deepcopy(original)
            if kind == "rotation": config["defaults"]["adapter"]["rotation_frame"] = "tool"
            elif kind == "checkpoint": config["models"]["act-v2"]["server"]["checkpoint"] = "/bad'path"
            else: config["models"]["act-v2"]["server"]["port"] = -1
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                resolve(config, "task1", "act-v2")


if __name__ == "__main__":
    unittest.main()
