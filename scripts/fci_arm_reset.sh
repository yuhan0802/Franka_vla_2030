#!/usr/bin/env bash
# Capture never moves the arm. Reset returns to the saved measured pose.
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-reset}"
case "$mode" in reset|capture|check) ;; *) echo "Usage: $0 [reset|capture|check]" >&2; exit 2;; esac
cd "$root_dir"
robot_ip_override="${ROBOT_IP:-}"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
robot_ip="${robot_ip_override:-${ROBOT_IP:-172.16.0.2}}"
source "$root_dir/scripts/runtime_log_root.sh"
pose_file="${FCI_RESET_POSE_FILE:-$root_dir/config/default_initial_pose.json}"
log_dir="$(mktemp -d "$FRANKA_LOG_ROOT/fci_arm_${mode}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
container_name="franka-fci-arm-${mode}-$$"
if [[ "$mode" != check ]] && command -v ss >/dev/null && [[ -n "$(ss -Htn state established dst "$robot_ip:1337")" ]]; then
  echo "An FCI arm client is active; stop the controller before capture/reset." >&2
  exit 1
fi
if [[ "$mode" == reset ]]; then
  python3 "$root_dir/scripts/franka_reset_pose.py" target "$pose_file" "$log_dir/target_joints.txt" "$robot_ip"
  echo "Returning arm to the saved pose at <= 0.1 rad/s; keep its return path clear."
fi
echo "Arm $mode log: $log_dir/arm.jsonl"
interrupt() { docker kill --signal=SIGINT "$container_name" >/dev/null 2>&1 || true; exit 130; }
trap interrupt INT TERM
docker run --rm --init --network host --name "$container_name" \
  --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  --cpuset-cpus "${FCI_RESET_CPUS:-4,5}" \
  -v "$root_dir/scripts:/probe:ro" -v "$log_dir:/reset_logs:ro" \
  -e "ROBOT_IP=$robot_ip" -e "FCI_ARM_MODE=$mode" \
  "${FCI_REINIT_IMAGE:-franka-vr:humble}" bash -lc '
    set -euo pipefail
    g++ -std=c++17 -pthread /probe/franka_fci_arm_reset.cpp -o /tmp/fci_arm_reset -lfranka
    case "$FCI_ARM_MODE" in
      check) echo "Arm helper build passed; no hardware connection or motion." ;;
      capture) exec timeout --signal=INT --kill-after=5s 10s /tmp/fci_arm_reset "$ROBOT_IP" --read ;;
      reset) exec timeout --signal=INT --kill-after=5s 180s /tmp/fci_arm_reset "$ROBOT_IP" --reset /reset_logs/target_joints.txt RETURN_TO_SAVED_POSE ;;
    esac
  ' 2>&1 | tee "$log_dir/arm.jsonl"
if [[ "$mode" == capture ]]; then
  python3 "$root_dir/scripts/franka_reset_pose.py" capture "$log_dir/arm.jsonl" "$pose_file" "$robot_ip"
fi
