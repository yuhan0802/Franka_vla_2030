#!/usr/bin/env bash
# Reinitialize the gripper, then return the arm to the saved measured pose.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if (( $# > 1 )); then echo "Expected at most one argument." >&2; exit 2; fi
check_only=0
reset_arm=1
case "${1:-}" in
  "") ;;
  --check) check_only=1 ;;
  --gripper-only) reset_arm=0 ;;
  --save-pose) exec "$root_dir/scripts/fci_arm_reset.sh" capture ;;
  -h|--help)
    echo "Usage: $0 [--check|--gripper-only|--save-pose]"
    echo "Default: one FCI gripper homing, then return the arm to the shared default initial pose."
    echo "Shared pose: config/default_initial_pose.json"
    echo "--gripper-only: home the gripper without moving the arm."
    echo "--save-pose: save the current measured arm pose without motion."
    echo "--check: compile both helpers without connecting to hardware."
    echo "Overrides: ROBOT_IP, FRANKA_LOG_ROOT, FCI_REINIT_IMAGE, FCI_RESET_POSE_FILE"
    exit 0 ;;
  *) echo "Usage: $0 [--check|--gripper-only|--save-pose]" >&2; exit 2 ;;
esac

cd "$root_dir"
robot_ip_override="${ROBOT_IP:-}"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
robot_ip="${robot_ip_override:-${ROBOT_IP:-172.16.0.2}}"
gripper_width_offset="${FCI_GRIPPER_WIDTH_OFFSET:-0.07}"
gripper_open_width="${FCI_GRIPPER_OPEN_WIDTH:-0.07}"
source "$root_dir/scripts/runtime_log_root.sh"
log_dir="$(mktemp -d "$FRANKA_LOG_ROOT/fci_init_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
container_name="franka-fci-gripper-init-$$"
if (( reset_arm && ! check_only )); then
  # Validate the saved target and controller shutdown before any homing motion.
  python3 "$root_dir/scripts/franka_reset_pose.py" target \
    "${FCI_RESET_POSE_FILE:-$root_dir/config/default_initial_pose.json}" "$log_dir/target_joints.txt" "$robot_ip"
  if command -v ss >/dev/null && [[ -n "$(ss -Htn state established dst "$robot_ip:1337")" ]]; then
    echo "An FCI arm client is active; stop the controller before reinit/reset." >&2
    exit 1
  fi
fi

if (( ! check_only )); then
  if command -v ss >/dev/null && [[ -n "$(ss -Htn state established dst "$robot_ip:1338")" ]]; then
    echo "An active gripper client is connected to $robot_ip:1338; stop the task before reinit." >&2
    exit 1
  fi
  echo "FCI gripper homing: keep hands and objects clear of the full finger travel."
  echo "Robot IP: $robot_ip"
fi
echo "Log: $log_dir/homing.jsonl"

interrupt() {
  docker kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true
  exit 130
}
trap interrupt INT TERM

# Homing on this gripper (non-original fingers) normally calibrates to the standard
# coordinate (open ~0.0702-0.0711 m; 10/10 homings on 2026-10-30 from open and closed
# starts). The raw coordinate (open ~0.142 m) is also accepted. Anything else, e.g. the
# -0.0012 m max_width seen once, is re-homed up to FCI_GRIPPER_HOMING_ATTEMPTS times.
attempts="${FCI_GRIPPER_HOMING_ATTEMPTS:-3}"
for (( attempt = 1; attempt <= attempts; attempt++ )); do
  log_file="$log_dir/homing.jsonl"
  (( attempt == 1 )) || log_file="$log_dir/homing_$attempt.jsonl"
  set +e
  docker run --rm --init --network host --name "$container_name" \
    -v "$root_dir/scripts:/probe:ro" \
    -e "ROBOT_IP=$robot_ip" -e "FCI_REINIT_CHECK_ONLY=$check_only" \
    -e "FCI_GRIPPER_WIDTH_OFFSET=$gripper_width_offset" -e "FCI_GRIPPER_OPEN_WIDTH=$gripper_open_width" \
    "${FCI_REINIT_IMAGE:-franka-vr:humble}" bash -lc '
      set -euo pipefail
      g++ -std=c++17 -pthread /probe/probe_franka_gripper_motion.cpp \
        -o /tmp/fci_gripper_reinit -lfranka
      if [[ "$FCI_REINIT_CHECK_ONLY" == 1 ]]; then
        echo "Build check passed; no hardware connection or motion."
        exit 0
      fi
      exec timeout --signal=INT --kill-after=5s 60s \
        /tmp/fci_gripper_reinit "$ROBOT_IP" GRIPPER_MOTION HOMING
    ' 2>&1 | tee "$log_file"
  run_status=${PIPESTATUS[0]}
  set -e

  if (( run_status != 0 )); then
    echo "FCI reinit failed or was interrupted (status $run_status). Log: $log_file" >&2
    exit "$run_status"
  fi
  if (( check_only )); then exec "$root_dir/scripts/fci_arm_reset.sh" check; fi

  set +e
  python3 - "$log_file" "$gripper_width_offset" "$gripper_open_width" <<'PY'
import json
import math
import sys

offset, expected_open = map(float, sys.argv[2:4])

records = []
with open(sys.argv[1], encoding="utf-8") as stream:
    for line in stream:
        if line.startswith("{"):
            records.append(json.loads(line))
results = [r for r in records if r.get("event") == "move_result" and r.get("phase") == "HOMING"]
open_results = [r for r in records if r.get("event") == "move_result" and r.get("phase") == "OPEN_AFTER_HOMING"]
states = [r for r in records if r.get("event") == "state" and r.get("phase") == "OPEN_AFTER_HOMING_SETTLED"]
if (not results or not results[-1]["success"] or results[-1]["timed_out"]
        or not open_results or not open_results[-1]["success"] or open_results[-1]["timed_out"] or not states):
    raise SystemExit("FCI homing did not complete with settled feedback; inspect the log.")
state = states[-1]
width, max_width = state["width_m"], state["max_width_m"]
print(f"FCI homing completed in {results[-1]['elapsed_s']:.2f}s.")
print(f"Opened after homing in {open_results[-1]['elapsed_s']:.2f}s.")
print(f"Current width: {width:.9f} m; maximum width: {max_width:.9f} m")
if not math.isfinite(width) or not math.isfinite(max_width) or width > max_width + 0.001:
    raise SystemExit("FCI homing feedback is not finite or exceeds max_width.")
# The EEF adapter uses offset 0 for the standard coordinate and the configured offset for the raw one.
for name, shift in (("standard", 0.0), (f"raw +{offset:.3f} m", offset)):
    effective_max, effective_width = max_width - shift, width - shift
    if 0.05 <= effective_max <= 0.085 and effective_width >= min(expected_open, effective_max) - 0.005:
        print(f"FCI gripper calibration and open feedback confirmed ({name} coordinate, "
              f"open width {effective_width:.4f} m).")
        raise SystemExit(0)
raise SystemExit("FCI homing/open feedback is inconsistent; calibration is not confirmed.")
PY
  check_status=$?
  set -e
  (( check_status == 0 )) && break
  if (( attempt == attempts )); then
    echo "Gripper calibration not confirmed after $attempts homing attempts. Logs: $log_dir" >&2
    exit 1
  fi
  echo "Re-homing the gripper (attempt $((attempt + 1)) of $attempts)." >&2
done
if (( reset_arm )); then
  exec "$root_dir/scripts/fci_arm_reset.sh" reset
fi
