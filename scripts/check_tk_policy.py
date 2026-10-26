#!/usr/bin/env python3
"""Read-only remote check of a resident policy's checkpoint, port and GPU."""
import pathlib
import sys


def verify(proc_root, port, checkpoint, config, gpu):
    matches = []
    for path in proc_root.glob("[0-9]*/cmdline"):
        try:
            args = path.read_bytes().decode().split("\0")
            if not any(arg.endswith("serve_policy.py") for arg in args):
                continue
            if "--port" not in args or args[args.index("--port") + 1] != port:
                continue
            if "--policy.dir=" + checkpoint not in args or "--policy.config=" + config not in args:
                continue
            env = (path.parent / "environ").read_bytes().decode().split("\0")
            if "CUDA_VISIBLE_DEVICES=" + gpu in env:
                matches.append(path.parent.name)
        except (OSError, UnicodeError, IndexError):
            continue
    # uv's parent process and its Python child can both match; only the Python
    # child actually owns the model. At least one exact match is required.
    if not matches:
        raise RuntimeError(f"resident model mismatch: port={port}, GPU={gpu}, checkpoint={checkpoint}")
    return matches


if __name__ == "__main__":
    port, checkpoint, config, gpu = sys.argv[1:]
    matches = verify(pathlib.Path("/proc"), port, checkpoint, config, gpu)
    print(f"Verified resident policy: port={port} GPU={gpu} checkpoint={checkpoint} PIDs={','.join(matches)}")
