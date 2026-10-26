#!/usr/bin/env python3
"""Resolve a task ID and its direct-flow overrides without starting services."""
import argparse
import copy
import json
import math
from pathlib import Path

from select_vla_task import validate_task


def resolve_task(config, task_id):
    if config.get("version") != 1 or not isinstance(config.get("tasks"), list):
        raise ValueError("config must contain version=1 and a tasks array")
    identified = [task for task in config["tasks"] if "id" in task]
    tasks = {task["id"]: task for task in identified}
    if len(tasks) != len(identified):
        raise ValueError("task IDs must be unique")
    if task_id not in tasks:
        raise ValueError(f"unknown task ID {task_id!r}; supported: {', '.join(tasks)}")

    def runtime_for(name, seen):
        if name not in tasks:
            raise ValueError(f"unknown runtime_from task ID: {name}")
        if name in seen:
            raise ValueError("circular runtime_from inheritance")
        task = tasks[name]
        direct = task.get("direct", {})
        if not isinstance(direct, dict):
            raise ValueError("direct settings must be an object")
        parent = direct.get("runtime_from")
        return runtime_for(parent, seen | {name}) if parent else task["runtime"]

    task = copy.deepcopy(tasks[task_id])
    task["runtime"] = copy.deepcopy(runtime_for(task_id, set()))
    validate_task(task)
    direct = task.get("direct", {})
    if not isinstance(direct.get("init_after_stop", False), bool):
        raise ValueError("direct.init_after_stop must be boolean")
    for key in ("gripper_feedback_open_width", "gripper_feedback_closed_width"):
        if key in direct and (not isinstance(direct[key], (int, float)) or not math.isfinite(direct[key])):
            raise ValueError(f"direct.{key} must be finite")
    if "gripper_verify_open_width" in direct and direct["gripper_verify_open_width"] not in (0, 1):
        raise ValueError("direct.gripper_verify_open_width must be 0 or 1")
    return task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        task = resolve_task(json.loads(args.config.read_text()), args.task_id)
        args.output.write_text(json.dumps({"version": 1, "tasks": [task]}, indent=2) + "\n")
        direct = task.get("direct", {})
        for value in (task["instruction"], int(direct.get("init_after_stop", False)),
                      direct.get("gripper_feedback_open_width", ""),
                      direct.get("gripper_feedback_closed_width", ""),
                      direct.get("gripper_verify_open_width", "")):
            print(value)
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
