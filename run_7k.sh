#!/usr/bin/env bash
# pi0.5-DROID checkpoint-7000 workflow on an isolated server port/session.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PI05_CHECKPOINT_STEP="${PI05_CHECKPOINT_STEP:-7000}"
export VLA_SERVER_PORT="${VLA_SERVER_PORT:-8095}"
export PI05_DROID_TMUX_SESSION="${PI05_DROID_TMUX_SESSION:-pi05-droid-66-7k-8095}"

exec "$root_dir/run_pi05_joint.sh" "$@"
