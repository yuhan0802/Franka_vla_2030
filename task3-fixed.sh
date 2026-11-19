#!/usr/bin/env bash
# pi0.5 task3 fixed checkpoint, prediction 15 / execute 10.
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PI05_DIRECT_PREDICTION_HORIZON=15
export PI05_DIRECT_EXECUTE_HORIZON=10
export PI05_DIRECT_CHUNK_MODE=rtc
export PI05_DIRECT_RTC_BLEND_STEPS=4
exec "$script_dir/run_task.sh" task3-fixed "$@"
