#!/usr/bin/env python3
"""Offline check of both weight registries. Never touches the robot, cameras, SSH or GPUs.

Runs the same resolvers the launchers use, for every configured pair:
  config/eef_tasks.json         -> scripts/eef_task_config.py resolve   (run_eef_task.sh)
  vla_instruction_weights.json  -> scripts/resolve_direct_task.py       (run_task.sh)

Errors (exit 1): any task/model pair the launcher would refuse.
Warnings (exit 0): models no task can select, and host:port endpoints shared by
different checkpoints (legal, but the old server must be stopped before switching).

Usage: python3 scripts/validate_task_registries.py [--eef PATH] [--joint PATH]
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from eef_task_config import resolve as resolve_eef  # noqa: E402
from resolve_direct_task import resolve_task as resolve_joint  # noqa: E402


def check_eef(path):
    errors, warnings = [], []
    config = json.loads(path.read_text(encoding="utf-8"))
    attached = set()
    for task_key, task in config.get("tasks", {}).items():
        for model_key in task.get("models", []):
            attached.add(model_key)
            try:
                resolve_eef(config, task_key, model_key)
            except (ValueError, KeyError, TypeError) as error:
                errors.append(f"EEF {task_key}/{model_key}: {error}")
    for model_key in sorted(set(config.get("models", {})) - attached):
        warnings.append(f"EEF model {model_key!r} is not listed in any tasks.<key>.models; nothing can select it")
    endpoints = defaultdict(set)
    for model_key, model in config.get("models", {}).items():
        server = model.get("server", {})
        endpoints[(server.get("host"), server.get("port"))].add((server.get("checkpoint"), model_key))
    for (host, port), entries in sorted(endpoints.items(), key=lambda item: str(item[0])):
        if len({checkpoint for checkpoint, _ in entries}) > 1:
            keys = ", ".join(sorted(key for _, key in entries))
            warnings.append(f"EEF {host}:{port} serves different checkpoints ({keys}); stop one before starting another")
    return errors, warnings, len(attached & set(config.get("models", {})))


def check_joint(path):
    errors, warnings = [], []
    config = json.loads(path.read_text(encoding="utf-8"))
    ids = [task["id"] for task in config.get("tasks", []) if "id" in task]
    for task_id in ids:
        try:
            resolve_joint(config, task_id)
        except (ValueError, KeyError, TypeError) as error:
            errors.append(f"joint {task_id}: {error}")
    for task in config.get("tasks", []):
        if "id" not in task:
            warnings.append(f"joint entry without id (instruction lookup only, not reachable by run_task.sh): "
                            f"{task.get('instruction')!r}")
    return errors, warnings, len(ids)


def main():
    root = HERE.parent
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--eef", type=Path, default=root / "config" / "eef_tasks.json")
    parser.add_argument("--joint", type=Path, default=root / "vla_instruction_weights.json")
    args = parser.parse_args()
    eef_errors, eef_warnings, eef_models = check_eef(args.eef)
    joint_errors, joint_warnings, joint_tasks = check_joint(args.joint)
    for line in eef_warnings + joint_warnings:
        print("WARN ", line)
    for line in eef_errors + joint_errors:
        print("ERROR", line)
    print(f"EEF registry: {eef_models} selectable models, {len(eef_errors)} error(s)  [{args.eef}]")
    print(f"joint registry: {joint_tasks} task IDs, {len(joint_errors)} error(s)  [{args.joint}]")
    return 1 if eef_errors or joint_errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
