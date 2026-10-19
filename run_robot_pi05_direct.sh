#!/usr/bin/env bash
# Dedicated direct-velocity robot stack. This file never launches MoveIt Servo.
set -euo pipefail

mode="${1:-disarmed}"
case "$mode" in disarmed|armed|fake) ;; *) echo "Usage: $0 {disarmed|armed|fake}" >&2; exit 2;; esac
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"
log_dir="${PI05_DIRECT_LOG_DIR:-$root_dir/logs/pi05_direct_$(date +%Y%m%d_%H%M%S)_$$}"
mkdir -p "$log_dir"; log_dir="$(cd "$log_dir" && pwd)"
exec > >(tee -a "$log_dir/robot.log") 2>&1
echo "Direct robot logs: $log_dir"

if [[ -f .env ]]; then set -a; source .env; set +a; fi
ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
armed=false; fake=false
[[ "$mode" == armed ]] && armed=true
[[ "$mode" == fake ]] && { armed=true; fake=true; ROBOT_IP=192.0.2.1; }
load_gripper="${PI05_DIRECT_LOAD_GRIPPER:-false}"
[[ "$load_gripper" == true || "$load_gripper" == false ]] || {
  echo "PI05_DIRECT_LOAD_GRIPPER must be true or false" >&2; exit 2; }
echo "Launch mode=$mode armed=$armed use_fake_hardware=$fake robot_ip=$ROBOT_IP load_gripper=$load_gripper"

if [[ "$mode" != fake ]]; then
  FCI_INTERFACE="${FCI_INTERFACE:-enp110s0}"
  FCI_IRQ_CPU="${FCI_IRQ_CPU:-2}"
  CONTROL_CPUS="${CONTROL_CPUS:-4,5}"
  NON_CONTROL_CPUS="${NON_CONTROL_CPUS:-0-1,3,6-31}"
  NON_FCI_IRQ_PATTERN="${NON_FCI_IRQ_PATTERN:-nvme|iwlwifi|thunderbolt}"
  export FCI_INTERFACE FCI_IRQ_CPU CONTROL_CPUS NON_CONTROL_CPUS NON_FCI_IRQ_PATTERN

  if ! ip link show dev "$FCI_INTERFACE" >/dev/null 2>&1; then
    echo "Warning: FCI interface $FCI_INTERFACE not found" >&2
  elif command -v ethtool >/dev/null 2>&1 && fci_link_info="$(ethtool "$FCI_INTERFACE" 2>/dev/null)"; then
    fci_speed="$(awk -F': *' '/^[[:space:]]*Speed:/{print $2; exit}' <<<"$fci_link_info")"
    fci_duplex="$(awk -F': *' '/^[[:space:]]*Duplex:/{print $2; exit}' <<<"$fci_link_info")"
    fci_detected="$(awk -F': *' '/^[[:space:]]*Link detected:/{print $2; exit}' <<<"$fci_link_info")"
    if [[ "$fci_speed" != "1000Mb/s" || "$fci_duplex" != "Full" || "$fci_detected" != "yes" ]]; then
      echo "Warning: FCI link is speed=${fci_speed:-unknown}, duplex=${fci_duplex:-unknown}, link=${fci_detected:-unknown}" >&2
    fi
  fi
  [[ "$(cat /sys/kernel/realtime 2>/dev/null || true)" == 1 ]] || echo "Warning: PREEMPT_RT not detected" >&2
  isolated_cpus="$(cat /sys/devices/system/cpu/isolated 2>/dev/null || true)"
  if [[ "$isolated_cpus" != *"4-5"* && "$isolated_cpus" != *"4,5"* ]]; then
    echo "Warning: CPUs 4,5 are not kernel-isolated" >&2
  fi
  timeout 2 ping -c 1 -W 1 "$ROBOT_IP" >/dev/null || echo "Warning: cannot ping $ROBOT_IP" >&2

  # Reapply the transient host settings used by the already validated Franka
  # path.  isolcpus alone does not move managed device IRQs off control CPUs.
  docker run --rm --privileged --pid host \
    -e "FCI_INTERFACE=$FCI_INTERFACE" -e "FCI_IRQ_CPU=$FCI_IRQ_CPU" \
    -e "CONTROL_CPUS=$CONTROL_CPUS" -e "NON_CONTROL_CPUS=$NON_CONTROL_CPUS" \
    -e "NON_FCI_IRQ_PATTERN=$NON_FCI_IRQ_PATTERN" franka-vr:humble \
    nsenter -t 1 -m -p -- /bin/bash -lc '
      set -euo pipefail
      for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        echo performance > "$governor" 2>/dev/null || true
      done
      irq=$(grep -m1 "$FCI_INTERFACE" /proc/interrupts | cut -d: -f1 | tr -d " ")
      if [[ -z "$irq" || ! -w "/proc/irq/$irq/smp_affinity_list" ]]; then
        echo "Cannot locate a writable IRQ for FCI interface $FCI_INTERFACE" >&2
        exit 1
      fi
      echo "$FCI_IRQ_CPU" > "/proc/irq/$irq/smp_affinity_list"
      affinity=$(cat "/proc/irq/$irq/smp_affinity_list")
      [[ "$affinity" == "$FCI_IRQ_CPU" ]] || {
        echo "FCI IRQ $irq affinity verification failed: expected $FCI_IRQ_CPU, got $affinity" >&2
        exit 1
      }
      echo "Pinned FCI interface $FCI_INTERFACE IRQ $irq to CPU $affinity"

      while IFS= read -r non_fci_irq; do
        [[ -n "$non_fci_irq" && -w "/proc/irq/$non_fci_irq/smp_affinity_list" ]] || continue
        if ! echo "$NON_CONTROL_CPUS" > "/proc/irq/$non_fci_irq/smp_affinity_list"; then
          echo "Non-FCI IRQ $non_fci_irq is kernel-managed; requires boot-time irqaffinity" >&2
          continue
        fi
        effective=$(cat "/proc/irq/$non_fci_irq/effective_affinity_list")
        case ",$effective," in
          *,4,*|*,5,*)
            echo "Non-FCI IRQ $non_fci_irq remains on control CPUs $CONTROL_CPUS (effective: $effective)" >&2
            ;;
        esac
        echo "Pinned non-FCI IRQ $non_fci_irq to CPUs $effective"
      done < <(grep -E "$NON_FCI_IRQ_PATTERN" /proc/interrupts | cut -d: -f1 | tr -d " ")
    '
else
  NON_CONTROL_CPUS="${NON_CONTROL_CPUS:-0-1,3,6-31}"
fi

# Refuse another controller manager/Servo stack rather than sharing FCI or the command topic.
while read -r container_id; do
  [[ -n "$container_id" ]] || continue
  command="$(docker inspect "$container_id" --format '{{json .Config.Cmd}}')"
  case "$command" in
    *franka_pi05_direct_velocity.launch.py*) ;;
    *franka_pi05_joint_velocity.launch.py*|*franka_twist.launch.py*|*gazebo_vr.launch.py*|*gazebo.launch.py*)
      echo "Conflicting Franka stack is running: $container_id $command" >&2; exit 1;;
  esac
done < <(docker ps -q --filter "label=com.docker.compose.project.working_dir=$(pwd)" --filter "label=com.docker.compose.oneoff=True")

docker compose -f docker-compose.yml -f docker-compose.pi05.yml run --rm \
  -v "$log_dir:/pi05_logs" -e ROS_LOG_DIR=/pi05_logs/ros \
  -e "PI05_COMPANION_CPUS=$NON_CONTROL_CPUS" \
  franka_robot bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    cd /ws_franka_vr
    flock /ws_franka_vr/.colcon-build.lock colcon build --symlink-install --cmake-force-configure \
      --packages-select franka_robot_state_broadcaster franka_vr 2>&1 | tee /pi05_logs/build.log
    source install/setup.bash
    exec taskset -c "$PI05_COMPANION_CPUS" ros2 launch franka_vr franka_pi05_direct_velocity.launch.py \
      robot_ip:="'"$ROBOT_IP"'" use_fake_hardware:="'"$fake"'" fake_sensor_commands:="'"$fake"'" armed:="'"$armed"'" load_gripper:="'"$load_gripper"'" \
      max_joint_velocity:="'"${PI05_DIRECT_MAX_VELOCITY:-0.6}"'" \
      max_joint_acceleration:="'"${PI05_DIRECT_MAX_ACCELERATION:-2.0}"'" \
      csv_log_path:=/pi05_logs/executor.csv
  '
