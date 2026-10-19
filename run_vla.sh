#!/usr/bin/env bash
# Unified FrankaVR three-view VLA deployment entrypoint.
set -euo pipefail

mode="${1:-dry-run}"
case "$mode" in
  server|dry-run|execute|status|stop) ;;
  *) echo "Usage: $0 {server|dry-run|execute|status|stop}" >&2; exit 2 ;;
esac

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
remote_host="${VLA_REMOTE_HOST:-mac_20}"
server_host="${VLA_SERVER_HOST:-10.24.116.20}"
server_port="${VLA_SERVER_PORT:-8093}"
gpu="${VLA_CUDA_VISIBLE_DEVICES:-3}"
remote_project="${VLA_REMOTE_PROJECT:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA}"
remote_deploy_script="${VLA_REMOTE_DEPLOY_SCRIPT:-vla-scripts/run_deploy_franka_simulation.sh}"
robot_session="franka_vr_vla_robot"
camera_session="franka_vr_vla_cameras"
remote_session="franka_vr_vla_${server_port}"

remote_server_running() {
  ssh "$remote_host" "ss -ltn | grep -q ':${server_port}'"
}

start_server() {
  if remote_server_running; then
    echo "FrankaVR model service is already listening on ${server_host}:${server_port}."
    return
  fi
  echo "Starting FrankaVR model service on ${remote_host} (GPU ${gpu}, port ${server_port})..."
  ssh "$remote_host" "tmux has-session -t '${remote_session}' 2>/dev/null && tmux kill-session -t '${remote_session}' || true; tmux new-session -d -s '${remote_session}' \"cd '${remote_project}' && exec env CUDA_VISIBLE_DEVICES='${gpu}' PORT='${server_port}' bash '${remote_deploy_script}'\""
  for _ in $(seq 1 90); do
    if remote_server_running; then
      echo "FrankaVR model service is ready at ${server_host}:${server_port}."
      return
    fi
    sleep 2
  done
  echo "Model service did not listen on port ${server_port}. Inspect: ssh ${remote_host} 'tmux capture-pane -pt ${remote_session}'" >&2
  exit 1
}

start_local_session() {
  local session="$1"
  local command="$2"
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "Using existing local session: ${session}"
  else
    tmux new-session -d -s "$session" "cd '${root_dir}' && exec ${command}"
    echo "Started local session: ${session}"
  fi
}

show_status() {
  printf 'remote %s:%s: ' "$server_host" "$server_port"
  if remote_server_running; then echo "listening"; else echo "stopped"; fi
  for session in "$robot_session" "$camera_session"; do
    printf 'local %s: ' "$session"
    if tmux has-session -t "$session" 2>/dev/null; then echo "running"; else echo "stopped"; fi
  done
}

stop_all() {
  # The real arm's controller has a verified graceful SIGINT shutdown path.
  # Do not use `docker stop` here: SIGTERM can interrupt that path and release
  # the arm before the controller performs its normal stop sequence. Do not
  # kill tmux until the container has completed that shutdown.
  mapfile -t robot_containers < <(
    docker ps -q \
      --filter "label=com.docker.compose.project.working_dir=${root_dir}" \
      --filter "label=com.docker.compose.service=franka_robot"
  )
  for container_id in "${robot_containers[@]}"; do
    echo "Requesting graceful robot shutdown: ${container_id}"
    docker kill --signal=SIGINT "$container_id" 2>/dev/null || true
  done

  local still_running=()
  for container_id in "${robot_containers[@]}"; do
    # The launch shutdown deactivates the effort controller and releases the
    # FCI session cleanly. Waiting prevents tmux from injecting a second,
    # different signal while that handoff is in progress.
    for _ in $(seq 1 150); do
      if [[ "$(docker inspect -f '{{.State.Running}}' "$container_id" 2>/dev/null || echo false)" != "true" ]]; then
        break
      fi
      sleep 0.1
    done
    if [[ "$(docker inspect -f '{{.State.Running}}' "$container_id" 2>/dev/null || echo false)" == "true" ]]; then
      still_running+=("$container_id")
    fi
  done

  if (( ${#still_running[@]} > 0 )); then
    echo "Robot shutdown is still in progress; leaving its tmux session running to preserve control:" >&2
    printf '  %s\n' "${still_running[@]}" >&2
    echo "Do not use docker stop or SIGKILL. Check the ROS log, then retry ./run_vla.sh stop." >&2
    return 1
  fi

  tmux kill-session -t "$robot_session" 2>/dev/null || true
  tmux kill-session -t "$camera_session" 2>/dev/null || true

  # A tmux session can disappear while its `docker compose run` child remains
  # alive, and repeated VLA starts can then leave multiple camera launchers
  # competing for the same RealSense devices.  Select only camera launch
  # containers (not the VLA bridge or robot stack) and stop them gracefully.
  mapfile -t workspace_containers < <(
    docker ps -q \
      --filter "label=com.docker.compose.project.working_dir=${root_dir}" \
      --filter "label=com.docker.compose.oneoff=True"
  )
  camera_containers=()
  for container_id in "${workspace_containers[@]}"; do
    container_command="$(docker inspect "$container_id" --format '{{join .Config.Cmd " "}}' 2>/dev/null || true)"
    case "$container_command" in
      *vla_cameras.launch.py*) camera_containers+=("$container_id") ;;
    esac
  done
  for container_id in "${camera_containers[@]}"; do
    echo "Requesting graceful camera shutdown: ${container_id}"
    docker kill --signal=SIGINT "$container_id" 2>/dev/null || true
  done
  for container_id in "${camera_containers[@]}"; do
    for _ in $(seq 1 50); do
      [[ "$(docker inspect -f '{{.State.Running}}' "$container_id" 2>/dev/null || echo false)" != "true" ]] && break
      sleep 0.1
    done
    if [[ "$(docker inspect -f '{{.State.Running}}' "$container_id" 2>/dev/null || echo false)" == "true" ]]; then
      echo "Camera did not exit after SIGINT; sending SIGTERM: ${container_id}"
      docker stop --timeout 10 "$container_id" 2>/dev/null || true
    fi
  done

  ssh "$remote_host" "tmux kill-session -t '${remote_session}' 2>/dev/null || true" || true
  show_status
}

case "$mode" in
  status)
    show_status
    exit 0
    ;;
  stop)
    stop_all
    exit 0
    ;;
  server)
    start_server
    exit 0
    ;;
esac

# dry-run and execute need the real robot's state/TF, but only execute may
# pass guarded Cartesian targets to it. The bridge remains foreground so
# Ctrl-C stops inference immediately; use "$0 stop" afterwards for the stack.
start_server
start_local_session "$robot_session" "ROBOT_IP='${ROBOT_IP:-172.16.0.2}' LOAD_GRIPPER='${LOAD_GRIPPER:-true}' USE_RVIZ='${USE_RVIZ:-false}' ./run_robot.sh"
start_local_session "$camera_session" "./run_vla_cameras.sh"
echo "Waiting briefly for robot state and three camera streams..."
sleep 5
exec env \
  VLA_SERVER_HOST="$server_host" \
  VLA_SERVER_PORT="$server_port" \
  VLA_CUDA_VISIBLE_DEVICES="$gpu" \
  ./run_franka_vr_vla_bridge.sh "$mode"
