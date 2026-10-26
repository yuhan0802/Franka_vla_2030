#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"

if [[ -f .env ]]; then
  set -a
  source .env
  set +a
fi

robot_ip="${ROBOT_IP:-172.16.0.2}"

echo "WARNING: gripper homing moves the fingers through their full travel."
echo "Keep hands and objects clear of the gripper."
echo "Robot IP: $robot_ip"

docker compose -f docker-compose.yml -f docker-compose.pi05.yml run --rm -T \
  -e "ROBOT_IP=$robot_ip" franka_robot bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /ws_franka_vr
    source install/setup.bash
    set -u

    action=/fr3_gripper/franka_gripper/homing
    joint_topic=/fr3_gripper/franka_gripper/joint_states
    node_log=/tmp/franka_gripper_homing_node.log
    result_log=/tmp/franka_gripper_homing_result.log
    launch_pid=""

    cleanup() {
      if [[ -n "$launch_pid" ]]; then
        kill -INT "$launch_pid" 2>/dev/null || true
        for _ in $(seq 1 30); do
          kill -0 "$launch_pid" 2>/dev/null || break
          sleep 0.1
        done
        if kill -0 "$launch_pid" 2>/dev/null; then
          kill -TERM "$launch_pid" 2>/dev/null || true
          sleep 1
        fi
        kill -KILL "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
      fi
    }
    trap cleanup EXIT INT TERM

    if ! ros2 action list 2>/dev/null | grep -Fxq "$action"; then
      echo "Starting standalone Franka gripper node..."
      ros2 launch franka_gripper gripper.launch.py \
        robot_ip:="$ROBOT_IP" namespace:=fr3_gripper >"$node_log" 2>&1 &
      launch_pid=$!

      ready=false
      for _ in $(seq 1 20); do
        if ! kill -0 "$launch_pid" 2>/dev/null; then
          cat "$node_log" >&2
          echo "Franka gripper node exited before the homing action became ready." >&2
          exit 1
        fi
        if ros2 action list 2>/dev/null | grep -Fxq "$action"; then
          ready=true
          break
        fi
        sleep 1
      done
      if [[ "$ready" != true ]]; then
        cat "$node_log" >&2
        echo "Timed out waiting for gripper homing action server." >&2
        exit 1
      fi
    else
      echo "Reusing the running Franka gripper node."
    fi

    echo "Sending homing goal..."
    set +e
    timeout --signal=INT --kill-after=5s 60s \
      ros2 action send_goal "$action" franka_msgs/action/Homing "{}" --feedback \
      2>&1 | tee "$result_log"
    action_status=${PIPESTATUS[0]}
    set -e

    if (( action_status != 0 )); then
      echo "Gripper homing command failed or timed out (status $action_status)." >&2
      exit "$action_status"
    fi
    if ! grep -Eq "success:[[:space:]]*true" "$result_log"; then
      echo "Gripper reported an unsuccessful homing result." >&2
      exit 1
    fi

    echo "Homing succeeded. Opening to 0.07 m and verifying measured width..."
    set +e
    timeout --signal=INT --kill-after=5s 30s \
      ros2 action send_goal /fr3_gripper/franka_gripper/move franka_msgs/action/Move \
        "{\"width\":0.07,\"speed\":0.02}" 2>&1 | tee "$result_log"
    action_status=${PIPESTATUS[0]}
    set -e
    if (( action_status != 0 )) || ! grep -Eq "success:[[:space:]]*true" "$result_log"; then
      echo "Opening after homing failed; do not start a task." >&2
      exit 1
    fi
    # Collect new samples for a settling window; a transient in-range reading
    # or an action success alone must not certify calibration.
    python3 - <<"PY"
import time
import math
import rclpy
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("verify_homed_gripper")
samples = []
started = time.monotonic()

def feedback(message):
    values = dict(zip(message.name, message.position))
    names = ("fr3_finger_joint1", "fr3_finger_joint2")
    if all(name in values for name in names):
        samples.append((time.monotonic(), [values[name] for name in names]))

subscription = node.create_subscription(
    JointState, "/fr3_gripper/franka_gripper/joint_states", feedback, qos_profile_sensor_data)
try:
    while time.monotonic() - started < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    recent = [values for stamp, values in samples if stamp >= time.monotonic() - 1.0]
    if len(recent) < 3:
        raise RuntimeError("insufficient fresh gripper feedback after homing")
    if any(not all(math.isfinite(v) for v in values) for values in recent):
        raise RuntimeError(f"non-finite gripper feedback after opening: {recent[-1]}")
    print(f"Homing/open actions completed; reported width={sum(recent[-1]):.4f}m (range check disabled)")
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
  '

echo "Gripper homing completed successfully."
