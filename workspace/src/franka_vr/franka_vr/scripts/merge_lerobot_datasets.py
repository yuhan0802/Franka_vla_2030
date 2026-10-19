#!/usr/bin/env python3
"""Merge compatible locally-recorded LeRobot v3 datasets without touching inputs."""
import argparse
import json
import shutil
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq


DATA_ROOT = Path("/datasets")


def load_dataset(name: str):
    root = DATA_ROOT / name
    info_path = root / "meta/info.json"
    data_path = root / "data/chunk-000/file-000.parquet"
    tasks_path = root / "meta/tasks.parquet"
    if not (info_path.is_file() and data_path.is_file() and tasks_path.is_file()):
        raise RuntimeError(f"{name!r} is not a completed local LeRobot dataset")
    return root, json.loads(info_path.read_text()), pq.read_table(data_path), {
        item["task_index"]: item["task"] for item in pq.read_table(tasks_path).to_pylist()
    }


def main():
    parser = argparse.ArgumentParser(description="Merge local LeRobot episodes into one dataset")
    parser.add_argument("output", help="new dataset name under /datasets")
    parser.add_argument("inputs", nargs="+", help="source dataset names under /datasets")
    args = parser.parse_args()
    output = DATA_ROOT / args.output
    if output.exists():
        raise RuntimeError(f"Output path already exists: {output}; choose a new name")
    if args.output in args.inputs:
        raise RuntimeError("Output dataset must not also be an input")

    merged_rows, episodes, tasks = [], [], {}
    base_info = None
    frame_index = 0
    episode_index = 0
    for name in args.inputs:
        root, info, table, source_tasks = load_dataset(name)
        if base_info is None:
            base_info = info
        elif info["fps"] != base_info["fps"] or info["features"] != base_info["features"]:
            raise RuntimeError(f"{name!r} has different fps or features and cannot be merged")
        source_rows = table.to_pylist()
        source_episodes = sorted({row["episode_index"] for row in source_rows})
        for old_episode in source_episodes:
            episode_rows = [row for row in source_rows if row["episode_index"] == old_episode]
            episode_tasks = set()
            for frame_in_episode, row in enumerate(episode_rows):
                row = dict(row)
                task = source_tasks[row["task_index"]]
                task_index = tasks.setdefault(task, len(tasks))
                episode_tasks.add(task)
                row["index"] = frame_index
                row["episode_index"] = episode_index
                row["task_index"] = task_index
                for key, feature in base_info["features"].items():
                    if feature["dtype"] != "image":
                        continue
                    source_image = Path(row[key])
                    if not source_image.is_file():
                        raise RuntimeError(f"Missing image referenced by {name!r}: {source_image}")
                    destination = output / "images" / key / f"episode-{episode_index:06d}" / f"frame-{frame_in_episode:06d}.png"
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source_image, destination)
                    row[key] = str(destination)
                merged_rows.append(row)
                frame_index += 1
            episodes.append({
                "episode_index": episode_index,
                "tasks": sorted(episode_tasks),
                "length": len(episode_rows),
                "meta/episodes/chunk_index": 0,
                "meta/episodes/file_index": 0,
                "dataset_from_index": frame_index - len(episode_rows),
                "dataset_to_index": frame_index,
            })
            episode_index += 1

    if not merged_rows:
        raise RuntimeError("No frames found in the selected inputs")
    data_path = output / "data/chunk-000/file-000.parquet"
    data_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.Table.from_pylist(merged_rows), data_path)
    meta = output / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.Table.from_pylist(
        [{"task_index": index, "task": task} for task, index in tasks.items()]
    ), meta / "tasks.parquet")
    episodes_path = meta / "episodes/chunk-000/file-000.parquet"
    episodes_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(pa.Table.from_pylist(episodes), episodes_path)
    base_info.update({
        "total_episodes": len(episodes), "total_frames": len(merged_rows),
        "total_tasks": len(tasks), "splits": {"train": f"0:{len(episodes)}"},
    })
    (meta / "info.json").write_text(json.dumps(base_info, indent=2))
    print(f"Merged {len(episodes)} episodes / {len(merged_rows)} frames into {output}")


if __name__ == "__main__":
    main()
