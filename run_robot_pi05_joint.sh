#!/usr/bin/env bash
# Independent pi0.5-DROID JointJog robot stack; does not start the old Cartesian bridge.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$root_dir"
log_dir="${PI05_LOG_DIR:-$root_dir/logs/pi05_robot_$(date +%Y%m%d_%H%M%S)_$$}"
mkdir -p "$log_dir"
log_dir="$(cd "$log_dir" && pwd)"
exec > >(tee -a "$log_dir/robot.log") 2>&1
echo "Persistent robot logs: $log_dir"

# Docker Compose reads .env itself, but this shell also needs ROBOT_IP for the
# explicit preflight check below.
if [[ -f .env ]]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi
# Keep the shell preflight consistent with docker-compose.yml.  Previously an
# absent .env made this script exit before Compose could apply its documented
# default, even though the same real-robot launch had worked with that default.
ROBOT_IP="${ROBOT_IP:-172.16.0.2}"
export ROBOT_IP
echo "Using Franka FCI address: $ROBOT_IP"

# All containers in this workspace use host networking and ROS_DOMAIN_ID 0.
# Only launch commands that create a controller manager conflict with the
# robot; companion nodes such as RealSense and Quest must remain usable.
mapfile -t workspace_containers < <(
  docker ps -q \
    --filter "label=com.docker.compose.project.working_dir=$(pwd)" \
    --filter "label=com.docker.compose.oneoff=True"
)
active_stack_containers=()
for container_id in "${workspace_containers[@]}"; do
  container_command="$(docker inspect "$container_id" --format '{{json .Config.Cmd}}')"
  case "$container_command" in
    *franka_pi05_joint_velocity.launch.py*|*franka_twist.launch.py*|*gazebo_vr.launch.py*|*gazebo.launch.py*)
      active_stack_containers+=("$container_id")
      ;;
  esac
done
if (( ${#active_stack_containers[@]} > 0 )); then
  echo "A Franka ROS stack from this workspace is already running; refusing to share /controller_manager." >&2
  for container_id in "${active_stack_containers[@]}"; do
    docker inspect "$container_id" --format '  {{.Name}} ({{.Id}}, {{.State.Status}})' >&2
  done
  echo "Stop that stack with its foreground Ctrl-C path (or docker kill --signal=SIGINT <container>) before starting the real robot." >&2
  exit 1
fi

# Report Franka FCI health before startup.  Older deployments of this project
# have operated at 100 Mb/s, so these are deliberately warnings rather than a
# new launch blocker; libfranka remains the authority on whether the live link
# can sustain its 1 kHz control loop.
FCI_INTERFACE="${FCI_INTERFACE:-enp110s0}"
FCI_IRQ_CPU="${FCI_IRQ_CPU:-2}"
CONTROL_CPUS="${CONTROL_CPUS:-4,5}"
# CPU 2 is reserved for the FCI NIC interrupt; CPUs 4-5 are robot control.
NON_CONTROL_CPUS="${NON_CONTROL_CPUS:-0-1,3,6-31}"
NON_FCI_IRQ_PATTERN="${NON_FCI_IRQ_PATTERN:-nvme|iwlwifi|thunderbolt}"
# docker run's `-e NAME` only forwards variables that are already exported.
# These defaults are shell variables, so pass their values explicitly to the
# host-tuning helper below.  Otherwise it receives empty values and silently
# leaves the FCI NIC IRQ free to run on the real-time control CPUs.
export FCI_INTERFACE FCI_IRQ_CPU CONTROL_CPUS NON_CONTROL_CPUS NON_FCI_IRQ_PATTERN

if ! ip link show dev "$FCI_INTERFACE" >/dev/null 2>&1; then
  echo "Warning: FCI interface '$FCI_INTERFACE' does not exist; set FCI_INTERFACE if host tuning is needed." >&2
elif command -v ethtool >/dev/null 2>&1 && fci_link_info="$(ethtool "$FCI_INTERFACE" 2>/dev/null)"; then
  fci_speed="$(awk -F': *' '/^[[:space:]]*Speed:/{print $2; exit}' <<<"$fci_link_info")"
  fci_duplex="$(awk -F': *' '/^[[:space:]]*Duplex:/{print $2; exit}' <<<"$fci_link_info")"
  fci_detected="$(awk -F': *' '/^[[:space:]]*Link detected:/{print $2; exit}' <<<"$fci_link_info")"
  if [[ "$fci_speed" != "1000Mb/s" || "$fci_duplex" != "Full" || "$fci_detected" != "yes" ]]; then
    echo "Warning: FCI link on $FCI_INTERFACE is speed=${fci_speed:-unknown}, duplex=${fci_duplex:-unknown}, link=${fci_detected:-unknown}." >&2
    echo "A dedicated 1000Mb/s full-duplex link is recommended; investigate communication errors if libfranka rejects the connection." >&2
  fi
else
  echo "Warning: cannot read FCI link state with ethtool; continuing without a link-speed diagnostic." >&2
fi

if [[ "$(cat /sys/kernel/realtime 2>/dev/null || true)" != "1" ]]; then
  echo "Warning: PREEMPT_RT was not detected (/sys/kernel/realtime is not 1)." >&2
  echo "Continue only for an already validated setup; a non-real-time host can cause libfranka communication failures." >&2
fi

if [[ "$(cat /sys/devices/system/cpu/isolated 2>/dev/null || true)" != *"4-5"* &&
      "$(cat /sys/devices/system/cpu/isolated 2>/dev/null || true)" != *"4,5"* ]]; then
  echo "Warning: CPUs 4,5 are not kernel-isolated; host processes can preempt the FCI control loop." >&2
  echo "Run ./configure_fci_realtime_host.sh once, then reboot before VR teleoperation." >&2
fi

if ! timeout 2 ping -c 1 -W 1 "$ROBOT_IP" >/dev/null; then
  echo "Warning: cannot ping Franka at $ROBOT_IP. Check addressing and the robot network if libfranka cannot connect." >&2
fi

# Apply the host-side real-time settings that proved stable with the FR3.
# The helper enters the host namespaces from a short-lived privileged
# container; no interactive sudo password is required.  Override these for a
# different host with FCI_INTERFACE and FCI_IRQ_CPU.
docker run --rm --privileged --pid host \
  -e "FCI_INTERFACE=$FCI_INTERFACE" -e "FCI_IRQ_CPU=$FCI_IRQ_CPU" \
  -e "CONTROL_CPUS=$CONTROL_CPUS" -e "NON_CONTROL_CPUS=$NON_CONTROL_CPUS" \
  -e "NON_FCI_IRQ_PATTERN=$NON_FCI_IRQ_PATTERN" franka-vr:humble \
  nsenter -t 1 -m -p -- /bin/bash -lc '
    set -euo pipefail
    for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
      echo performance > "$governor" 2>/dev/null || true
    done
    irq=$(grep -m1 "$FCI_INTERFACE" /proc/interrupts \
      | cut -d: -f1 | tr -d " ")
    if [[ -z "$irq" || ! -w "/proc/irq/$irq/smp_affinity_list" ]]; then
      echo "Cannot locate a writable IRQ for FCI interface $FCI_INTERFACE" >&2
      exit 1
    fi
    echo "$FCI_IRQ_CPU" > "/proc/irq/$irq/smp_affinity_list"
    affinity=$(cat "/proc/irq/$irq/smp_affinity_list")
    if [[ "$affinity" != "$FCI_IRQ_CPU" ]]; then
      echo "FCI IRQ $irq affinity verification failed: expected $FCI_IRQ_CPU, got $affinity" >&2
      exit 1
    fi
    echo "Pinned FCI interface $FCI_INTERFACE IRQ $irq to CPU $affinity"

    # isolcpus does not migrate managed hardware IRQs by itself.  NVMe,
    # Wi-Fi and Thunderbolt IRQ handlers previously ran at FIFO priority 50
    # on CPUs 4/5, competing directly with the 1 kHz FCI control loop.
    while IFS= read -r non_fci_irq; do
      [[ -n "$non_fci_irq" && -w "/proc/irq/$non_fci_irq/smp_affinity_list" ]] || continue
      # Managed NVMe IRQs can only be assigned at boot through irqaffinity.
      # Do not abort a real-robot launch merely because one such IRQ rejects
      # a runtime write; the persistent host helper reports it explicitly.
      if ! echo "$NON_CONTROL_CPUS" > "/proc/irq/$non_fci_irq/smp_affinity_list"; then
        echo "Non-FCI IRQ $non_fci_irq is kernel-managed; requires boot-time irqaffinity" >&2
        continue
      fi
      effective=$(cat "/proc/irq/$non_fci_irq/effective_affinity_list")
      case ",$effective," in
        *,4,*|*,5,*)
          echo "Non-FCI IRQ $non_fci_irq remains on Franka control CPUs $CONTROL_CPUS (effective: $effective)" >&2
          ;;
      esac
      echo "Pinned non-FCI IRQ $non_fci_irq to CPUs $effective"
    done < <(grep -E "$NON_FCI_IRQ_PATTERN" /proc/interrupts | cut -d: -f1 | tr -d " ")
  '

docker compose -f docker-compose.yml -f docker-compose.pi05.yml run --rm \
  -v "$log_dir:/pi05_logs" -e ROS_LOG_DIR=/pi05_logs/ros \
  -e "PI05_COMPANION_CPUS=$NON_CONTROL_CPUS" franka_robot bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /ws_franka_vr
  flock /ws_franka_vr/.colcon-build.lock \
    colcon build --symlink-install --cmake-force-configure --packages-select franka_robot_state_broadcaster franka_vr
  source install/setup.bash
  # Make ros2 launch PID 1 in the container. SIGINT from the foreground
  # terminal or run_vla.sh then reaches the launch system directly instead of
  # relying on an intermediate shell to forward it.
  exec taskset -c "$PI05_COMPANION_CPUS" ros2 launch franka_vr franka_pi05_joint_velocity.launch.py \
    robot_ip:="'"${ROBOT_IP}"'" use_fake_hardware:=false \
    load_gripper:="'"${LOAD_GRIPPER:-true}"'" use_rviz:="'"${USE_RVIZ:-false}"'"
'
