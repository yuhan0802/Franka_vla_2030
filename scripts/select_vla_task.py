#!/usr/bin/env python3
"""Resolve one exact VLA instruction to its checkpoint and runtime settings."""

import argparse
import json
from pathlib import Path


def load_task(config_path: Path, instruction: str) -> dict:
    with config_path.open(encoding="utf-8") as config_file:
        config = json.load(config_file)
    if config.get("version") != 1 or not isinstance(config.get("tasks"), list):
        raise ValueError("config must contain version=1 and a tasks array")

    tasks = config["tasks"]
    instructions = [task.get("instruction") for task in tasks]
    if len(instructions) != len(set(instructions)):
        raise ValueError("task instructions must be unique")
    matches = [task for task in tasks if task.get("instruction") == instruction]
    if not matches:
        supported = "\n  - ".join(str(value) for value in instructions)
        raise ValueError(f"unknown VLA_INSTRUCTION: {instruction!r}\nSupported instructions:\n  - {supported}")
    return matches[0]


def validate_task(task: dict) -> None:
    checkpoint_root = task.get("checkpoint_root")
    checkpoint_step = task.get("checkpoint_step")
    policy_config = task.get("policy_config")
    runtime = task.get("runtime")
    if not isinstance(checkpoint_root, str) or not checkpoint_root.startswith("/"):
        raise ValueError("checkpoint_root must be an absolute path")
    if not isinstance(checkpoint_step, int) or checkpoint_step <= 0:
        raise ValueError("checkpoint_step must be a positive integer")
    if not isinstance(policy_config, str) or not policy_config:
        raise ValueError("policy_config must be a non-empty string")
    if not isinstance(runtime, dict):
        raise ValueError("runtime must be an object")
    if not isinstance(runtime.get("max_actions"), int) or runtime["max_actions"] <= 0:
        raise ValueError("runtime.max_actions must be a positive integer")
    for key in ("max_joint_velocity", "max_joint_acceleration", "gripper_force"):
        if not isinstance(runtime.get(key), (int, float)) or runtime[key] <= 0:
            raise ValueError(f"runtime.{key} must be positive")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    arguments = parser.parse_args()

    try:
        task = load_task(arguments.config, arguments.instruction)
        validate_task(task)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))

    runtime = task["runtime"]
    values = (
        task["checkpoint_root"],
        task["checkpoint_step"],
        task["policy_config"],
        runtime["max_actions"],
        runtime["max_joint_velocity"],
        runtime["max_joint_acceleration"],
        runtime["gripper_force"],
    )
    for value in values:
        print(value)


if __name__ == "__main__":
    main()
