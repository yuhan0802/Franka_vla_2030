#!/usr/bin/env python3
"""Check relative links in the project's own Markdown files (upstream packages are skipped).

A missing target that git ignores (e.g. logs/ on the robot host) is reported as skipped,
not broken. Exit 1 if any other link is broken.

Usage: python3 scripts/check_doc_links.py
"""
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parent.parent
UPSTREAM = ("workspace/src/moveit2", "workspace/src/franka_ros2", "workspace/src/franka_description",
            "workspace/src/joint_", "workspace/src/moveit_msgs", "workspace/src/franka_vr/franka_vr/oculus_reader",
            "vendor/", "upstream-libfranka-docker/")
LINK = re.compile(r'!?\[[^\]]*\]\(([^)\s]+)(?:\s+"[^"]*")?\)')


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)


def main():
    files = [f for f in git("ls-files", "-co", "--exclude-standard", "*.md").stdout.splitlines()
             if f and not f.startswith(UPSTREAM)]
    broken = skipped = 0
    for rel in files:
        path = ROOT / rel
        in_code = False
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if line.lstrip().startswith("```"):
                in_code = not in_code
                continue
            if in_code:
                continue
            for match in LINK.finditer(line):
                target = match.group(1)
                if re.match(r"^[a-z][a-z0-9+.-]*:|^#", target):
                    continue
                local = unquote(target.split("#", 1)[0])
                if not local or (path.parent / local).exists():
                    continue
                resolved = (path.parent / local).resolve()
                try:
                    ignored = git("check-ignore", "-q", str(resolved.relative_to(ROOT))).returncode == 0
                except ValueError:
                    ignored = False
                if ignored:
                    skipped += 1
                else:
                    broken += 1
                    print(f"{rel}:{number}: broken link -> {target}")
    print(f"{len(files)} files checked, {broken} broken, {skipped} skipped (gitignored local artifacts)")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
