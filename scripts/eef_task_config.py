#!/usr/bin/env python3
"""Resolve fixed EEF task/model profiles and check robot-free execution logs."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re


def finite_vector(value, size):
    if not isinstance(value, list) or len(value) != size or any(
            not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"expected {size} finite numbers")
    return value


def quaternion(value):
    q = finite_vector(value, 4)
    norm = math.sqrt(sum(v * v for v in q))
    if not .5 <= norm <= 1.5:
        raise ValueError("invalid quaternion norm")
    return [v / norm for v in q]


def resolve(config, task_key, model_key, control=None):
    if config.get("version") != 1:
        raise ValueError("EEF config version must be 1")
    for key in (task_key, model_key):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", key):
            raise ValueError("task/model keys must contain only letters, digits, '_' or '-'")
    if task_key not in config["tasks"] or model_key not in config["models"]:
        raise ValueError(f"unknown task/model key: {task_key}/{model_key}; use 'list'")
    task, model = config["tasks"][task_key], config["models"][model_key]
    if model_key not in task.get("models", []):
        raise ValueError(f"no configured EEF checkpoint association for {task_key}/{model_key}")
    defaults = config["defaults"]
    adapter = {**defaults["adapter"], **model.get("adapter", {}), **task.get("adapter", {})}
    runtime = {**defaults["runtime"], **model.get("runtime", {}), **task.get("runtime", {})}
    # Model identity remains fixed by JSON. Explicit execution-budget overrides
    # are retained for the existing batch-test launchers.
    for env, field, cast in (("EEF_MAX_CHUNKS", "max_chunks", int),
                             ("EEF_MAX_ACTIONS", "max_actions", int)):
        if os.environ.get(env):
            runtime[field] = cast(os.environ[env])
    control = control or defaults["control"]
    if control not in ("direct", "servo"):
        raise ValueError("EEF_CONTROL must be direct or servo")
    if adapter["gripper_state_source"] not in ("measured", "binary", "command"):
        raise ValueError("invalid gripper_state_source")
    if adapter["exterior_camera"] not in ("left", "right") or adapter["camera_mode"] not in ("raw", "calibrated"):
        raise ValueError("invalid camera configuration")
    if not isinstance(adapter["model_image_size"], int) or adapter["model_image_size"] <= 0:
        raise ValueError("model_image_size must be positive")
    for key in ("gripper_width_offset", "gripper_open_width"):
        if not isinstance(adapter.get(key), (int, float)) or not math.isfinite(adapter[key]):
            raise ValueError(f"adapter.{key} must be finite")
    if (not 0 <= adapter["gripper_width_offset"] <= .1
            or not 0 < adapter["gripper_open_width"] <= .08
            or adapter["gripper_width_offset"] + adapter["gripper_open_width"] > .15):
        raise ValueError("invalid gripper firmware offset/open width")
    if (not isinstance(adapter["max_tf_age_s"], (int, float))
            or not math.isfinite(adapter["max_tf_age_s"])
            or not 0 < adapter["max_tf_age_s"] <= 1.0):
        raise ValueError("max_tf_age_s must be in (0, 1.0]")
    rotations = {"relative_rotvec_left_multiply": "world", "relative_rotvec_right_multiply": "tool"}
    if rotations.get(adapter["rotation_execution"]) != adapter["rotation_frame"]:
        raise ValueError("rotation frame/execution mismatch")
    if runtime["chunk_start"] not in ("measured", "last_target") or not isinstance(runtime["auto_home"], bool):
        raise ValueError("invalid chunk_start/auto_home")
    if not isinstance(runtime["execute_horizon"], int) or not 1 <= runtime["execute_horizon"] <= 15:
        raise ValueError("runtime.execute_horizon out of range")
    for key, upper in (("max_chunks", 200), ("max_actions", 2000)):
        if not isinstance(runtime[key], int) or runtime[key] < 0 or runtime[key] > upper:
            raise ValueError(f"runtime.{key} out of range (0 means unlimited)")
    if (not math.isfinite(runtime["max_translation"]) or not 0 <= runtime["max_translation"] <= .02
            or not math.isfinite(runtime["max_rotation"]) or not 0 < runtime["max_rotation"] <= .15):
        raise ValueError("translation/rotation limits out of range")
    gain = runtime.get("action_gain", [1.0] * 6)
    finite_vector(gain, 6)
    if any(not 0 < value <= 1.0 for value in gain):
        raise ValueError("runtime.action_gain values must be in (0, 1]")
    runtime["action_gain"] = gain
    pose = copy.deepcopy(task["start_pose"])
    finite_vector(pose["position"], 3)
    quaternion(pose["quaternion_xyzw"])
    for key in ("position_tolerance_m", "rotation_tolerance_rad"):
        if not math.isfinite(pose[key]) or pose[key] <= 0:
            raise ValueError("start-pose tolerances must be positive")
    instruction = task["instruction"]
    if not isinstance(instruction, str) or not instruction.strip() or any(c in instruction for c in "\0\n\r"):
        raise ValueError("invalid task instruction")
    server = copy.deepcopy(model["server"])
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.@-]*", server["ssh"]):
        raise ValueError("invalid SSH alias")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", server["host"]):
        raise ValueError("invalid server host")
    if not isinstance(server["port"], int) or not 1 <= server["port"] <= 65535:
        raise ValueError("server port out of range")
    if not isinstance(server["gpu"], int) or server["gpu"] < 0:
        raise ValueError("invalid GPU")
    for key in ("checkpoint", "actioncodec", "state_stats", "project"):
        if (not isinstance(server[key], str) or not server[key].startswith("/")
                or any(c in server[key] for c in "'\n\r\0")):
            raise ValueError(f"server.{key} must be an absolute path without quotes/control characters")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+\.py", server["script"]):
        raise ValueError("server script must be a Python basename")
    normalization = copy.deepcopy(model.get("action_normalization", {}))
    if normalization:
        if normalization.get("method") not in ("task0_maxabs", "droid_gain", "legacy_droid_symmetric_scale", "franka_maxabs", "franka_mulaw", "franka_quantile", "unverified"):
            raise ValueError("unsupported action normalization method")
        path = normalization.get("params_path", "")
        if not isinstance(path, str) or not path.startswith("/") or any(c in path for c in "\0\n\r"):
            raise ValueError("action normalization params_path must be an absolute path")
        if not isinstance(normalization.get("params_key"), str) or not normalization["params_key"]:
            raise ValueError("action normalization params_key is required")
        if normalization["method"] == "franka_mulaw":
            mu = normalization.get("mu")
            if not isinstance(mu, (int, float)) or not math.isfinite(mu) or mu <= 0:
                raise ValueError("franka_mulaw requires a finite positive mu")
    return dict(task_key=task_key, model_key=model_key, instruction=instruction,
                control=control, start_pose=pose, adapter=adapter, runtime=runtime, server=server,
                action_normalization=normalization,
                model_info={key: copy.deepcopy(model[key]) for key in (
                    "scope", "training_tasks", "training_tasks_path", "task_key_to_training_task",
                    "action_representation", "action_shape") if key in model})


def environment(profile):
    s, a, r, p = (profile[k] for k in ("server", "adapter", "runtime", "start_pose"))
    n = profile.get("action_normalization", {})
    return {"EEF_CONTROL": profile["control"], "VLA_INSTRUCTION": profile["instruction"],
            "EEF_REMOTE_SSH": s["ssh"], "EEF_SERVER_HOST": s["host"], "EEF_SERVER_PORT": str(s["port"]),
            "EEF_SERVER_GPU": str(s["gpu"]), "EEF_REMOTE_PROJECT": s["project"],
            "EEF_CHECKPOINT": s["checkpoint"], "EEF_ACTIONCODEC": s["actioncodec"],
            "EEF_STATE_STATS": s["state_stats"], "EEF_SERVER_SCRIPT": s["script"], "EEF_SERVER_ARGS": "",
            "EEF_GRIPPER_STATE_SOURCE": a["gripper_state_source"], "EEF_EXTERIOR_CAMERA": a["exterior_camera"],
            "EEF_GRIPPER_WIDTH_OFFSET": str(a["gripper_width_offset"]),
            "EEF_GRIPPER_OPEN_WIDTH": str(a["gripper_open_width"]),
            "EEF_MODEL_IMAGE_SIZE": str(a["model_image_size"]), "VLA_CAMERA_MODE": a["camera_mode"],
            "EEF_MAX_TF_AGE": str(a["max_tf_age_s"]),
            "EEF_ROTATION_FRAME": a["rotation_frame"], "EEF_ROTATION_EXECUTION": a["rotation_execution"],
            "EEF_MAX_TRANSLATION": str(r["max_translation"]), "EEF_MAX_ROTATION": str(r["max_rotation"]),
            "EEF_ACTION_GAIN": " ".join(map(str, r["action_gain"])),
            "EEF_CHUNK_START": r["chunk_start"], "EEF_EXECUTE_HORIZON": str(r["execute_horizon"]),
            "EEF_MAX_CHUNKS": str(r["max_chunks"]), "EEF_MAX_ACTIONS": str(r["max_actions"]),
            "EEF_AUTO_HOME": str(r["auto_home"]).lower(),
            "EEF_EXPECTED_ACTION_NORMALIZATION": n.get("method", "") if n.get("method") in ("franka_maxabs", "franka_mulaw", "franka_quantile") else "",
            "EEF_EXPECTED_ACTION_MULAW_MU": str(n["mu"]) if n.get("method") == "franka_mulaw" else "0",
            "EEF_HOME_POSITION": " ".join(map(str, p["position"])),
            "EEF_HOME_QUATERNION": " ".join(map(str, p["quaternion_xyzw"]))}


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    listing = sub.add_parser("list"); listing.add_argument("--config", type=Path, required=True)
    resolution = sub.add_parser("resolve")
    resolution.add_argument("--config", type=Path, required=True)
    resolution.add_argument("--task", required=True); resolution.add_argument("--model", required=True)
    resolution.add_argument("--control", default=None)
    resolution.add_argument("--output", type=Path, required=True)
    resolution.add_argument("--env-file", type=Path, required=True)
    for mode in ("check-axis", "check-dry-run", "check-start"):
        check = sub.add_parser(mode); check.add_argument("log", type=Path)
        if mode == "check-start": check.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.action == "list":
            config = json.loads(args.config.read_text())
            for key, task in config["tasks"].items():
                print(f"{key}: {task['instruction']} | models: {', '.join(task['models']) or '(not configured)'}")
            return
        if args.action == "resolve":
            profile = resolve(json.loads(args.config.read_text()), args.task, args.model, args.control)
            args.output.write_text(json.dumps(profile, indent=2, ensure_ascii=False) + "\n")
            pairs = environment(profile)
            args.env_file.write_bytes(b"".join(k.encode() + b"\0" + v.encode() + b"\0" for k, v in pairs.items()))
            return
        log = records(args.log)
        if args.action == "check-axis":
            if not any(r.get("event") == "complete" for r in log) or any(r.get("event") == "fault" for r in log):
                raise ValueError("axis-test did not complete cleanly")
        elif args.action == "check-dry-run":
            if not any(r.get("robot_command_sent") is False for r in log) or any(r.get("robot_command_sent") is True for r in log):
                raise ValueError("dry-run did not prove robot_command_sent=false")
        else:
            pose = json.loads(args.profile.read_text())["start_pose"]
            armed = next((r for r in log if r.get("event") == "armed"), None)
            if armed is None: raise ValueError("dry-run has no armed pose")
            position = finite_vector(armed["position"], 3)
            a, b = quaternion(armed["quaternion_xyzw"]), quaternion(pose["quaternion_xyzw"])
            distance = math.sqrt(sum((x-y)**2 for x, y in zip(position, pose["position"])))
            dot = min(1.0, abs(sum(x*y for x, y in zip(a, b))))
            turn = 2 * math.acos(dot)
            print(f"Start-pose offset: {distance:.6f} m / {turn:.6f} rad")
            if distance > pose["position_tolerance_m"] or turn > pose["rotation_tolerance_rad"]:
                return 3
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
