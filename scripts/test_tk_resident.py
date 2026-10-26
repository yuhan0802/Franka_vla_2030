"""Resident-policy identity checks and robot-free launcher lifecycle tests."""
import importlib.util
import pathlib
import tempfile
import unittest
import subprocess
import os
import json

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("check_tk_policy", ROOT / "scripts/check_tk_policy.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class ResidentTests(unittest.TestCase):
    def test_unmounted_log_disk_blocks_run_but_not_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            fake_mountpoint = pathlib.Path(directory) / "mountpoint"
            fake_mountpoint.write_text("#!/bin/sh\nexit 1\n")
            fake_mountpoint.chmod(0o755)
            env = dict(os.environ, PATH=directory + ":" + os.environ["PATH"],
                       FRANKA_LOG_ROOT="/media/mac/Elements1/franka_vr_docker/logs")
            for mode, expected in (("run", 1), ("stop", 0)):
                result = subprocess.run(["bash", "-c", 'source "$1" "$2"', "log-root",
                                         str(ROOT / "scripts/runtime_log_root.sh"), mode],
                                        env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, expected, result.stderr)

    def test_wrong_gpu_or_checkpoint_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            proc = pathlib.Path(directory)
            pid = proc / "123"
            pid.mkdir()
            (pid / "cmdline").write_bytes(
                b"python\0scripts/serve_policy.py\0--port\08101\0--policy.dir=/weights/20000\0--policy.config=pi05\0")
            (pid / "environ").write_bytes(b"CUDA_VISIBLE_DEVICES=5\0")
            self.assertEqual(checker.verify(proc, "8101", "/weights/20000", "pi05", "5"), ["123"])
            for checkpoint, gpu in (("/weights/20000", "6"), ("/wrong/20000", "5")):
                with self.assertRaises(RuntimeError):
                    checker.verify(proc, "8101", checkpoint, "pi05", gpu)

    def launch(self, fail_identity=False, live_dry_run=False, fail_camera=False, fail_robot=False):
        with tempfile.TemporaryDirectory() as directory:
            temp = pathlib.Path(directory)
            repo, binaries = temp / "repo", temp / "bin"
            (repo / "scripts").mkdir(parents=True)
            binaries.mkdir()
            for file in ("tk1.sh", "scripts/run_tk_direct.sh", "scripts/select_vla_task.py",
                         "scripts/check_tk_policy.py", "scripts/runtime_log_root.sh", "vla_instruction_weights.json"):
                (repo / file).write_bytes((ROOT / file).read_bytes())
                (repo / file).chmod(0o755)
            stub = '''#!/usr/bin/env python3
import sys, os, pathlib, json
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
state = pathlib.Path(os.environ["TK_TEST_STATE"])
with (state / "calls").open("a") as out:
    out.write(json.dumps([name] + args) + "\\n")
if name == "ssh":
    sys.stdin.read()
    sys.exit(1 if os.environ.get("TK_FAIL_IDENTITY") else 0)
if name == "curl":
    sys.exit(0)
if name == "tmux":
    session = args[args.index("-t") + 1] if "-t" in args else args[args.index("-s") + 1]
    marker = state / session
    if args[0] == "has-session":
        sys.exit(0 if marker.exists() else 1)
    if args[0] == "send-keys":
        marker.unlink(missing_ok=True)
    if args[0] == "new-session":
        marker.touch()
        if session.endswith("_robot"):
            log = next((pathlib.Path.cwd() / "logs").glob("tk1_*"))
            (log / "robot.log").write_text("armed dependencies ready\\nConfigured and activated fr3_direct_velocity_controller\\n")
if name == "docker" and args[:2] == ["compose", "run"]:
    if "tk-camera-check" in args:
        sys.exit(1 if os.environ.get("TK_FAIL_CAMERA") else 0)
    mount = args[args.index("-v") + 1]
    log = pathlib.Path(mount.rsplit(":", 1)[0])
    if os.environ.get("TK_FAIL_ROBOT"):
        (state / "franka_vr_tk1_robot").unlink(missing_ok=True)
        (log / "robot.log").write_text("communication_constraints_violation\\n")
        import time
        time.sleep(1.5)
    if "--execute" not in args:
        (log / "full_dry_run_policy.jsonl").write_text('{"robot_command_sent": false}\\n')
'''
            for command in ("ssh", "curl", "tmux", "docker"):
                (binaries / command).write_text(stub)
                (binaries / command).chmod(0o755)
            env = dict(os.environ, PATH=str(binaries) + ":" + os.environ["PATH"],
                       TK_TEST_STATE=str(temp), TMPDIR=str(temp), FRANKA_LOG_ROOT=str(repo / "logs"))
            env["TK_LIVE_DRY_RUN"] = "1" if live_dry_run else "0"
            if fail_identity:
                env["TK_FAIL_IDENTITY"] = "1"
            if fail_camera:
                env["TK_FAIL_CAMERA"] = "1"
            if fail_robot:
                env["TK_FAIL_ROBOT"] = "1"
            result = subprocess.run([str(repo / "tk1.sh")], cwd=repo, env=env,
                                    capture_output=True, text=True, timeout=10)
            calls = [json.loads(line) for line in (temp / "calls").read_text().splitlines()]
            return result, calls

    def test_run_reuses_model_and_cleanup_never_sends_remote_stop(self):
        result, calls = self.launch()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        remote = [call for call in calls if call[0] == "ssh"]
        self.assertEqual(len(remote), 1, "only a read-only identity check is allowed")
        self.assertIn("python3 - 8101", remote[0][-1])
        adapters = [call for call in calls if call[:3] == ["docker", "compose", "run"] and "tk-adapter" in call]
        self.assertEqual(len(adapters), 1, "tk defaults to execute without a dry-run")
        self.assertIn("--execute", adapters[0])
        self.assertEqual(adapters[0][adapters[0].index("--server-port") + 1], "8101")
        self.assertEqual(adapters[0][adapters[0].index("--gripper-require-grasp-success") + 1], "0")
        starts = [call[call.index("-s") + 1] for call in calls
                  if call[:2] == ["tmux", "new-session"]]
        self.assertEqual(starts, ["franka_vr_tk1_cameras", "franka_vr_tk1_robot"])
        self.assertIn("kept running", result.stdout)

    def test_optional_live_dry_run_precedes_execution(self):
        result, calls = self.launch(live_dry_run=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        adapters = [call for call in calls if call[:3] == ["docker", "compose", "run"] and "tk-adapter" in call]
        self.assertEqual(len(adapters), 2)
        self.assertNotIn("--execute", adapters[0])
        self.assertIn("--execute", adapters[1])

    def test_camera_failure_never_starts_robot(self):
        result, calls = self.launch(fail_camera=True)
        self.assertNotEqual(result.returncode, 0)
        starts = [call[call.index("-s") + 1] for call in calls if call[:2] == ["tmux", "new-session"]]
        self.assertEqual(starts, ["franka_vr_tk1_cameras"])
        self.assertFalse(any("tk-adapter" in call for call in calls))

    def test_robot_exit_stops_adapter_and_reports_failure(self):
        result, calls = self.launch(fail_robot=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Robot/camera stack stopped or faulted", result.stderr)
        self.assertTrue(any(call[:3] == ["docker", "kill", "--signal=SIGINT"] for call in calls))

    def test_identity_failure_starts_no_local_resources(self):
        result, calls = self.launch(fail_identity=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(all(call[0] == "ssh" for call in calls))


if __name__ == "__main__":
    unittest.main()
