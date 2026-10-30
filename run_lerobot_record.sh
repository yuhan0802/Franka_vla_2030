#!/usr/bin/env bash
set -euo pipefail
DATASET_NAME="${1:-fr3_teleop}"
DURATION="${2:-30}"
TASK="${3:-Move the FR3 end effector along a small smooth random trajectory}"

# Recording only launches cameras and the writer.  Arm/gripper feedback must
# come from an already-running real or simulated Franka stack.  Fail early
# with an actionable message instead of leaving the recorder at a generic
# "Waiting for ..." line forever.
mapfile -t workspace_containers < <(
  docker ps -q \
    --filter "label=com.docker.compose.project.working_dir=$(pwd)" \
    --filter "label=com.docker.compose.oneoff=True"
)
robot_stack_running=0
for container_id in "${workspace_containers[@]}"; do
  container_command="$(docker inspect "$container_id" --format '{{join .Config.Cmd " "}}')"
  case "$container_command" in
    *franka_twist.launch.py*|*gazebo_vr.launch.py*)
      robot_stack_running=1
      break
      ;;
  esac
done
if (( ! robot_stack_running )); then
  echo "没有检测到 Franka 机械臂状态/控制栈，录制无法开始。" >&2
  echo "真机采集请先在单独终端运行: ./run_robot.sh" >&2
  echo "然后保持 ./run_vr.sh 运行，再执行本录制命令。" >&2
  exit 1
fi

docker compose run --rm franka_vr bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  if [[ ! -f install/setup.bash ]]; then
    echo "franka_vr has not been built. Run this once:" >&2
    echo "  docker compose run --rm franka_vr bash -lc '\''source /opt/ros/humble/setup.bash; cd /ws_franka_vr; colcon build --symlink-install --packages-select franka_vr'\''" >&2
    exit 1
  fi
  source install/setup.bash
  exec ros2 launch franka_vr lerobot_recording.launch.py \
    dataset_name:="'"${DATASET_NAME}"'" duration:="'"${DURATION}"'" task:="'"${TASK}"'"
'
