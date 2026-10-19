#!/usr/bin/env bash
set -euo pipefail
mode="${1:-dry-run}"; [[ "$mode" == dry-run || "$mode" == execute ]] || { echo "Usage: $0 {dry-run|execute}" >&2; exit 2; }
export VLA_SERVER_HOST="${VLA_SERVER_HOST:-10.24.116.29}" VLA_SERVER_PORT="${VLA_SERVER_PORT:-8093}"
export VLA_INSTRUCTION="${VLA_INSTRUCTION:-put the red test tube into the basket}" VLA_BOUNDS_CONFIG="${VLA_BOUNDS_CONFIG:-/ws_franka_vr/src/franka_vr/franka_vr/config/vla_calibration.yaml}"
# By default use the live robot state.  Set this to an explicit JSON float[8]
# only for a deliberate reproducibility test.
export VLA_OBSERVATION_STATE="${VLA_OBSERVATION_STATE:-}"
export VLA_SAVE_OBSERVATION_DIR="${VLA_SAVE_OBSERVATION_DIR:-}"
export VLA_POLICY_HZ="${VLA_POLICY_HZ:-30.0}" VLA_ACTIONS_PER_CHUNK=15 VLA_ACTION_HZ="${VLA_ACTION_HZ:-15.0}" VLA_OBSERVATION_SYNC_TOLERANCE="${VLA_OBSERVATION_SYNC_TOLERANCE:-0.030}" VLA_MAX_TRANSLATION="${VLA_MAX_TRANSLATION:-0.010}" VLA_MAX_ROTATION="${VLA_MAX_ROTATION:-0.08}"
export VLA_HOLD_REFRESH_SEC="${VLA_HOLD_REFRESH_SEC:-0.20}" VLA_ENABLE_GRIPPER="${VLA_ENABLE_GRIPPER:-true}" VLA_GRIPPER_CLOSE_THRESHOLD="${VLA_GRIPPER_CLOSE_THRESHOLD:-0.50}" VLA_GRIPPER_HYSTERESIS="${VLA_GRIPPER_HYSTERESIS:-0.10}" VLA_GRIPPER_STATE_OPEN_WIDTH="${VLA_GRIPPER_STATE_OPEN_WIDTH:-0.10300}" VLA_GRIPPER_OPEN_WIDTH="${VLA_GRIPPER_OPEN_WIDTH:-0.10300}" VLA_GRIPPER_CLOSED_WIDTH="${VLA_GRIPPER_CLOSED_WIDTH:-0.0}" VLA_GRIPPER_SPEED="${VLA_GRIPPER_SPEED:-0.02}"
docker compose run --rm -e VLA_SERVER_HOST -e VLA_SERVER_PORT -e VLA_INSTRUCTION -e VLA_BOUNDS_CONFIG -e VLA_OBSERVATION_STATE -e VLA_SAVE_OBSERVATION_DIR -e VLA_POLICY_HZ -e VLA_ACTIONS_PER_CHUNK -e VLA_ACTION_HZ -e VLA_OBSERVATION_SYNC_TOLERANCE -e VLA_MAX_TRANSLATION -e VLA_MAX_ROTATION -e VLA_HOLD_REFRESH_SEC -e VLA_ENABLE_GRIPPER -e VLA_GRIPPER_CLOSE_THRESHOLD -e VLA_GRIPPER_HYSTERESIS -e VLA_GRIPPER_STATE_OPEN_WIDTH -e VLA_GRIPPER_OPEN_WIDTH -e VLA_GRIPPER_CLOSED_WIDTH -e VLA_GRIPPER_SPEED franka_vr bash -lc '
set -eo pipefail; source /opt/ros/humble/setup.bash; cd /ws_franka_vr; flock /ws_franka_vr/.colcon-build.lock colcon build --symlink-install --cmake-force-configure --packages-select franka_vr; source install/setup.bash
args=(--server-host "$VLA_SERVER_HOST" --server-port "$VLA_SERVER_PORT" --instruction "$VLA_INSTRUCTION" --bounds-config "$VLA_BOUNDS_CONFIG" --policy-hz "$VLA_POLICY_HZ" --actions-per-chunk "$VLA_ACTIONS_PER_CHUNK" --action-hz "$VLA_ACTION_HZ" --observation-sync-tolerance "$VLA_OBSERVATION_SYNC_TOLERANCE" --max-translation "$VLA_MAX_TRANSLATION" --max-rotation "$VLA_MAX_ROTATION" --hold-refresh-sec "$VLA_HOLD_REFRESH_SEC" --gripper-close-threshold "$VLA_GRIPPER_CLOSE_THRESHOLD" --gripper-hysteresis "$VLA_GRIPPER_HYSTERESIS" --gripper-state-open-width "$VLA_GRIPPER_STATE_OPEN_WIDTH" --gripper-open-width "$VLA_GRIPPER_OPEN_WIDTH" --gripper-closed-width "$VLA_GRIPPER_CLOSED_WIDTH" --gripper-speed "$VLA_GRIPPER_SPEED")
[[ -n "$VLA_OBSERVATION_STATE" ]] && args+=(--observation-state "$VLA_OBSERVATION_STATE")
[[ -n "$VLA_SAVE_OBSERVATION_DIR" ]] && args+=(--save-observation-dir "$VLA_SAVE_OBSERVATION_DIR")
[[ "$VLA_ENABLE_GRIPPER" == true ]] && args+=(--enable-gripper)
[[ "'"$mode"'" == execute ]] && args+=(--execute)
exec ros2 run franka_vr franka_vr_vla_bridge.py "${args[@]}"
'
