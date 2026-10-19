#!/usr/bin/env bash
# Independent, fail-safe launcher for the pi0.5-DROID service on server 66.
set -euo pipefail

mode="${1:-probe}"
case "$mode" in
  server|probe|dry-run|execute|status|stop) ;;
  *) echo "Usage: $0 {server|probe|dry-run|execute|status|stop}" >&2; exit 2 ;;
esac

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
remote_host="${VLA_REMOTE_HOST:-10.24.116.66}"
remote_user="${VLA_REMOTE_USER:-vla}"
ssh_port="${VLA_SSH_PORT:-1172}"
ssh_key="${VLA_SSH_KEY:-/home/mac/.ssh/id_rsa_66}"
server_port="${VLA_SERVER_PORT:-8094}"
deploy_script="${VLA_REMOTE_DEPLOY_SCRIPT:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA/vla-scripts/run_deploy_franka_pi05_droid_66.sh}"
remote_session="${PI05_DROID_TMUX_SESSION:-pi05-droid-66-${server_port}}"
checkpoint_step="${PI05_CHECKPOINT_STEP:-latest}"
checkpoint_root="${PI05_CHECKPOINT_ROOT:-}"
policy_config="${PI05_POLICY_CONFIG:-pi05_franka_tube_pick_v5_finetune}"
server_load_timeout="${PI05_SERVER_LOAD_TIMEOUT:-300}"
robot_session="franka_vr_pi05_droid_robot"
camera_session="franka_vr_pi05_droid_cameras"

ssh_args=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 -i "$ssh_key" -p "$ssh_port" "${remote_user}@${remote_host}")

if [[ "$checkpoint_step" != latest && ! "$checkpoint_step" =~ ^[0-9]+$ ]]; then
  echo "PI05_CHECKPOINT_STEP must be 'latest' or a positive integer, got: $checkpoint_step" >&2
  exit 2
fi
if [[ ! "$server_load_timeout" =~ ^[1-9][0-9]*$ ]]; then
  echo "PI05_SERVER_LOAD_TIMEOUT must be a positive integer, got: $server_load_timeout" >&2
  exit 2
fi

remote_deploy() {
  local action="$1" command
  printf -v command 'PORT=%q PI05_DROID_TMUX_SESSION=%q CHECKPOINT_STEP=%q CONFIG_NAME=%q' \
    "$server_port" "$remote_session" "$checkpoint_step" "$policy_config"
  if [[ -n "$checkpoint_root" ]]; then
    printf -v command '%s CHECKPOINT_ROOT=%q' "$command" "$checkpoint_root"
  fi
  printf -v command '%s bash %q %q' "$command" "$deploy_script" "$action"
  ssh "${ssh_args[@]}" "$command"
}

check_key() {
  [[ -f "$ssh_key" ]] || {
    echo "SSH key not found: $ssh_key" >&2
    echo "Set VLA_SSH_KEY to the private key for server 66; no fallback to another server is allowed." >&2
    return 1
  }
  local key_mode
  key_mode="$(stat -c '%a' "$ssh_key")"
  if (( (8#$key_mode & 8#077) != 0 )); then
    echo "Unsafe SSH key permissions ($key_mode): $ssh_key" >&2
    echo "The key must not be accessible by group or others (recommended mode: 600)." >&2
    return 1
  fi
}

check_ssh() {
  check_key
  ssh "${ssh_args[@]}" true >/dev/null || {
    echo "Cannot connect to ${remote_user}@${remote_host}:${ssh_port}; refusing to use any fallback server." >&2
    return 1
  }
}

remote_port_listening() {
  ssh "${ssh_args[@]}" "ss -ltn | awk '{print \$4}' | grep -Eq '(^|:)${server_port}\$'"
}

health_ok() {
  curl --fail --silent --show-error --max-time 5 "http://${remote_host}:${server_port}/healthz" >/dev/null
}

start_server() {
  check_ssh
  if [[ "$checkpoint_step" != latest ]]; then
    local checkpoint_command
    printf -v checkpoint_command 'bash -s -- %q %q %q %q %q %q %q %q' \
      "$checkpoint_step" "$server_port" "$remote_session" "$deploy_script" "$checkpoint_root" \
      "${PI05_SERVER_GPU:-0}" "$policy_config" "$server_load_timeout"
    ssh "${ssh_args[@]}" "$checkpoint_command" < "$root_dir/scripts/pi05_checkpoint_server.sh"
  fi
  if remote_port_listening; then
    echo "pi0.5-DROID service is already listening at ${remote_host}:${server_port}; reusing it."
  else
    echo "Starting pi0.5-DROID service at ${remote_host}:${server_port}..."
    remote_deploy start
  fi

  local attempt health_attempts
  health_attempts=$(( (server_load_timeout + 1) / 2 ))
  for attempt in $(seq 1 "$health_attempts"); do
    if remote_port_listening && health_ok; then
      echo "Service ready: TCP ${server_port} and /healthz are healthy."
      return 0
    fi
    sleep 2
  done
  echo "Service did not become healthy within approximately $server_load_timeout seconds." >&2
  echo "Inspect the remote log: /data/vla/openpi-main/logs/pi05-droid-66/${remote_session}.log" >&2
  return 1
}

run_probe() {
  start_server
  echo "Running the remote synthetic probe (zero images and zero joint state; no robot interfaces)..."
  remote_deploy probe
}

show_status() {
  check_ssh
  remote_deploy status
  printf 'HTTP healthz: '
  if health_ok; then echo healthy; else echo unavailable; fi
  local session
  for session in "$robot_session" "$camera_session"; do
    printf 'local tmux %s: ' "$session"
    if tmux has-session -t "$session" 2>/dev/null; then echo running; else echo stopped; fi
  done
}

start_local_session() {
  local session="$1" command="$2"
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "Reusing local tmux session: $session"
  else
    tmux new-session -d -s "$session" "cd '$root_dir' && exec $command"
    echo "Started local tmux session: $session"
  fi
}

stop_local_session_gracefully() {
  local session="$1" label="$2" attempt
  tmux has-session -t "$session" 2>/dev/null || return 0
  echo "Sending SIGINT through the ${label} tmux terminal: $session"
  tmux send-keys -t "$session" C-c
  for attempt in $(seq 1 150); do
    tmux has-session -t "$session" 2>/dev/null || return 0
    sleep 0.1
  done
  echo "$label is still shutting down; leaving tmux intact. SIGKILL and docker stop were not used." >&2
  return 1
}

stop_new_resources() {
  local failed=0
  stop_local_session_gracefully "$robot_session" robot || failed=1
  stop_local_session_gracefully "$camera_session" camera || failed=1
  check_ssh || return 1
  remote_deploy stop || failed=1
  return "$failed"
}

case "$mode" in
  execute)
    echo "REFUSED: pi0.5-DROID returns [16,8] joint-velocity actions, but the existing bridge accepts [15,7] EE delta-pose actions." >&2
    echo "No robot, camera, bridge, /set_target_pose, or motion interface was started." >&2
    echo "A separate joint-velocity Franka adapter must first pass simulation, replay, limiting, and unloaded tests." >&2
    exit 3
    ;;
  server) start_server ;;
  probe) run_probe ;;
  status) show_status ;;
  stop) stop_new_resources ;;
  dry-run)
    start_server
    start_local_session "$robot_session" "ROBOT_IP='${ROBOT_IP:-172.16.0.2}' LOAD_GRIPPER='${LOAD_GRIPPER:-true}' USE_RVIZ='${USE_RVIZ:-false}' ./run_robot.sh"
    start_local_session "$camera_session" "./run_vla_cameras.sh"
    echo "dry-run does not send robot motion commands"
    echo "Limitation: the existing bridge uses an incompatible action protocol, so this dry-run uses the synthetic remote probe instead of sending live camera/state data."
    run_probe
    ;;
esac
